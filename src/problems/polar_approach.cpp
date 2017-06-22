#include "problems/polar_approach.h"

#include "rosban_utils/xml_tools.h"

#include <cmath>

namespace csa_mdp
{

/**
 * Return the given angle in radian 
 * bounded between -PI and PI
 */
static double normalizeAngle(double angle)
{
  return angle - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

PolarApproach::PolarApproach()
  : max_dist(1.5),
    // State limits
    min_step_x(-0.02),
    max_step_x(0.04),
    max_step_y(0.02),
    max_step_theta(0.3),
    // Action limits
    max_step_x_diff(0.01),
    max_step_y_diff(0.01),
    max_step_theta_diff(0.1),
    // Kick
    kick_x_min(0.12),
    kick_x_max(0.22),
    kick_y_tol(0.04),
    kick_y_offset(0.08),
    kick_theta_offset(0),
    kick_theta_tol(10 * M_PI/180),
    kick_reward(0),
    kick_terminal_speed_factor(0.2),
    // Viewing the ball
    viewing_angle(M_PI/2),
    no_view_reward(0),
    // Collision
    collision_x_front(0.12),
    collision_x_back(0.2),
    collision_y(0.2),
    collision_reward(-200),
    terminal_collisions(true),
    // Misc
    out_of_space_reward(-200),
    walk_frequency(1.7),
    init_min_dist(0.4),
    init_max_dist(max_dist - 0.05)
{
  updateLimits();
}

void PolarApproach::updateLimits()
{
  Eigen::MatrixXd state_limits(6,2), action_limits(3,2);
  state_limits <<
    0, max_dist,
    -M_PI, M_PI,
    -M_PI, M_PI,
    min_step_x, max_step_x,
    -max_step_y, max_step_y,
    -max_step_theta, max_step_theta;
  action_limits <<
    -max_step_x_diff, max_step_x_diff,
    -max_step_y_diff, max_step_y_diff,
    -max_step_theta_diff, max_step_theta_diff;
  setStateLimits(state_limits);
  setActionLimits({action_limits});

  // Also ensure names are valid
  setStateNames({"ball_dist", "ball_dir", "target_angle", "step_x", "step_y", "step_theta"});
  setActionsNames({{"d_step_x","d_step_y","d_step_theta"}});
}

void PolarApproach::setMaxDist(double dist)
{
  max_dist = dist;
  updateLimits();
}

bool PolarApproach::canKickLeftFoot(const Eigen::VectorXd & state) const
{
  // Compute Basic properties
  double ball_x = getBallX(state);
  double ball_y = getBallY(state);
  double theta  = state(2);
  double kick_err = normalizeAngle(theta + kick_theta_offset);
  // Check validity
  bool x_ok = ball_x > kick_x_min && ball_x < kick_x_max;
  bool y_ok = std::fabs(ball_y - kick_y_offset) < kick_y_tol;
  bool theta_ok = std::fabs(kick_err) < kick_theta_tol;
  return x_ok && y_ok && theta_ok;
}

bool PolarApproach::canKickRightFoot(const Eigen::VectorXd & state) const
{
  // Compute Basic properties
  double ball_x = getBallX(state);
  double ball_y = getBallY(state);
  double theta  = state(2);
  double kick_err = normalizeAngle(theta - kick_theta_offset);
  // Check validity
  bool x_ok = ball_x > kick_x_min && ball_x < kick_x_max;
  bool y_ok = std::fabs(ball_y + kick_y_offset) < kick_y_tol;
  bool theta_ok = std::fabs(kick_err) < kick_theta_tol;
  return x_ok && y_ok && theta_ok;
}

bool PolarApproach::isTerminal(const Eigen::VectorXd & state) const
{
  bool collision_terminal = (terminal_collisions && isColliding(state));
  bool kick_terminal = false;
  if (kick_terminal_speed_factor > 0) {
    kick_terminal = true;
    const Eigen::MatrixXd & action_limits = getActionLimits(0);
    // Check that all orders are below the given threshold
    for (int dim = 0; dim < 3; dim++) {
      double min_value = action_limits(dim,0) * kick_terminal_speed_factor;
      double max_value = action_limits(dim,1) * kick_terminal_speed_factor;
      double order_value = state(dim+3);
      // Are we outside of bound for acceptance?
      if (order_value < min_value || order_value > max_value) {
        kick_terminal = false;
        break;
      }
    }
  }
  return kick_terminal || collision_terminal || isOutOfSpace(state);
}

double  PolarApproach::getReward(const Eigen::VectorXd & state,
                                 const Eigen::VectorXd & action,
                                 const Eigen::VectorXd & dst) const
{
  (void)state;(void)action;
  if (isColliding(dst) ) return collision_reward;
  if (isKickable(dst)  ) return kick_reward;
  if (isOutOfSpace(dst)) return out_of_space_reward;
  // During each walk cycle, the robot performs 2 steps
  double reward = -1 / (2 * walk_frequency);
  if (!seeBall(dst)    ) reward += no_view_reward;
  return reward;
}

Problem::Result PolarApproach::getSuccessor(const Eigen::VectorXd & state,
                                            const Eigen::VectorXd & action,
                                            std::default_random_engine * engine) const
{
  // Now, actions need to be (0 vx vy vtheta)
  if (action.rows() != 4) {
    std::ostringstream oss;
    oss << "PolarApproach::getSuccessor: "
        << " invalid dimension for action, expecting 4 and received "
        << action.rows();
    throw std::runtime_error(oss.str());
  }
  // Get the step which will be applied
  Eigen::VectorXd next_cmd(3);
  for (int dim = 0; dim < 3; dim++)
  {
    // Ensuring that acceleration is in the bounds
    const Eigen::MatrixXd & action_limits = getActionLimits(0);
    double min_acc = action_limits(dim, 0);
    double max_acc = action_limits(dim, 1);
    double bounded_action = std::min(max_acc, std::max(min_acc, action(dim+1)));
    // Action applies a delta on step
    next_cmd(dim) = bounded_action + state(dim + 3);
    // Ensuring that final action is inside of the bounds
    const Eigen::MatrixXd & limits = getStateLimits();
    double min_cmd = limits(dim + 3, 0);
    double max_cmd = limits(dim + 3, 1);
    next_cmd(dim) = std::min(max_cmd, std::max(min_cmd, next_cmd(dim)));
  }
  // Apply a linear modification (from theory to 'reality')
  Eigen::VectorXd real_move = odometry.getDiffFullStep(next_cmd, engine);
  // Apply the real move
  Eigen::VectorXd next_state = state;
  // Apply rotation first
  double delta_theta = real_move(2);
  next_state(2) = normalizeAngle(state(2) - delta_theta);
  next_state(1) = normalizeAngle(state(1) - delta_theta);
  // Then, apply translation
  double ball_x = getBallX(next_state) - real_move(0);
  double ball_y = getBallY(next_state) - real_move(1);
  double new_dist = std::sqrt(ball_x * ball_x + ball_y * ball_y);
  double new_dir = atan2(ball_y, ball_x);
  next_state(0) = new_dist;
  next_state(1) = new_dir;
  // Update cmd
  next_state.segment(3,3) = next_cmd;
  Problem::Result result;
  result.successor = next_state;
  result.reward = getReward(state, action, next_state);
  result.terminal = isTerminal(next_state);
  return result;
}

Eigen::VectorXd PolarApproach::getStartingState(std::default_random_engine * engine) const
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  // Creating the distribution
  std::uniform_real_distribution<double> dist_distrib(init_min_dist,
                                                      init_max_dist);
  std::uniform_real_distribution<double> viewable_distrib(-viewing_angle,
                                                          viewing_angle);
  std::uniform_real_distribution<double> angle_distrib(-M_PI, M_PI);
  // Generating random values
  double dist = dist_distrib(*engine);
  double ball_theta = viewable_distrib(*engine);
  double target_theta = angle_distrib(*engine);
  // Updating state
  state(0) = dist;
  state(1) = ball_theta;
  state(2) = target_theta;

  return state;
}

bool PolarApproach::isKickable(const Eigen::VectorXd & state) const
{
  return canKickLeftFoot(state) || canKickRightFoot(state);
}

bool PolarApproach::isColliding(const Eigen::VectorXd & state) const
{
  double ball_x = getBallX(state);
  double ball_y = getBallY(state);
  bool x_ko = ball_x > - collision_x_back && ball_x < collision_x_front;
  bool y_ko = std::fabs(ball_y) < collision_y;
  return x_ko && y_ko;
}

bool PolarApproach::isOutOfSpace(const Eigen::VectorXd & state) const
{
  const Eigen::MatrixXd & space_limits = getStateLimits();
  for (int dim = 0; dim < state.rows(); dim++)
  {
    if (state(dim) < space_limits(dim, 0) || state(dim) > space_limits(dim, 1))
    {
      return true;
    }
  }
  return false;
}

bool PolarApproach::seeBall(const Eigen::VectorXd & state) const
{
  double angle = state(1);
  return angle < viewing_angle && angle > -viewing_angle;
}

void PolarApproach::to_xml(std::ostream & out) const {
  (void)out;
  throw std::logic_error("PolarApproach::to_xml: not implemented");
}

void PolarApproach::from_xml(TiXmlNode * node)
{
  std::string odometry_path;
  rosban_utils::xml_tools::try_read<std::string>(node,
                                                 "odometry_path",
                                                 odometry_path);

  // If coefficients have been properly read, use them
  if (odometry_path != "")
  {
    try {
      odometry.loadFromFile(odometry_path);
    }
    catch (const std::runtime_error & err) {
      std::ostringstream oss;
      oss << "PolarApproach::from_xml: failed to read odometry file '"
          << odometry_path << "'";
      throw std::runtime_error(oss.str());
    }
  }
  // Read internal properties
  rosban_utils::xml_tools::try_read<double>(node,"max_dist", max_dist);
  rosban_utils::xml_tools::try_read<double>(node,"min_step_x", min_step_x);
  rosban_utils::xml_tools::try_read<double>(node,"max_step_x", max_step_x);
  rosban_utils::xml_tools::try_read<double>(node,"max_step_y", max_step_y);
  rosban_utils::xml_tools::try_read<double>(node,"max_step_theta", max_step_theta);
  rosban_utils::xml_tools::try_read<double>(node,"max_step_x_diff", max_step_x_diff);
  rosban_utils::xml_tools::try_read<double>(node,"max_step_y_diff", max_step_y_diff);
  rosban_utils::xml_tools::try_read<double>(node,"max_step_theta_diff", max_step_theta_diff);
  rosban_utils::xml_tools::try_read<double>(node,"kick_x_min", kick_x_min);
  rosban_utils::xml_tools::try_read<double>(node,"kick_x_max", kick_x_max);
  rosban_utils::xml_tools::try_read<double>(node,"kick_y_tol", kick_y_tol);
  rosban_utils::xml_tools::try_read<double>(node,"kick_y_offset", kick_y_offset);
  rosban_utils::xml_tools::try_read<double>(node,"kick_theta_tol", kick_theta_tol);
  rosban_utils::xml_tools::try_read<double>(node,"kick_theta_offset", kick_theta_offset);
  rosban_utils::xml_tools::try_read<double>(node,"kick_reward", kick_reward);
  rosban_utils::xml_tools::try_read<double>(node,"kick_terminal_speed_factor", kick_terminal_speed_factor);
  rosban_utils::xml_tools::try_read<double>(node,"viewing_angle", viewing_angle);
  rosban_utils::xml_tools::try_read<double>(node,"no_view_reward", no_view_reward);
  rosban_utils::xml_tools::try_read<double>(node,"collision_x_front", collision_x_front);
  rosban_utils::xml_tools::try_read<double>(node,"collision_x_back", collision_x_back);
  rosban_utils::xml_tools::try_read<double>(node,"collision_y", collision_y);
  rosban_utils::xml_tools::try_read<double>(node,"collision_reward", collision_reward);
  rosban_utils::xml_tools::try_read<bool>  (node,"terminal_collisions", terminal_collisions);
  rosban_utils::xml_tools::try_read<double>(node,"out_of_space_reward", out_of_space_reward);
  rosban_utils::xml_tools::try_read<double>(node,"walk_frequency", walk_frequency);
  rosban_utils::xml_tools::try_read<double>(node,"init_min_dist", init_min_dist);
  rosban_utils::xml_tools::try_read<double>(node,"init_max_dist", init_max_dist);

  // Update limits according to the new parameters
  updateLimits();
}

std::string PolarApproach::class_name() const
{
  return "polar_approach";
}

double PolarApproach::getBallX(const Eigen::VectorXd & state)
{
  return cos(state(1)) * state(0);
}

double PolarApproach::getBallY(const Eigen::VectorXd & state)
{
  return sin(state(1)) * state(0);
}

}
