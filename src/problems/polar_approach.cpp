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
  : max_dist(1.0),
    // State limits
    min_step_x(-0.02),
    max_step_x(0.04),
    max_step_y(0.02),
    max_step_theta(0.3),
    // Action limits
    max_step_x_diff(0.01),
    max_step_y_diff(0.01),
    max_step_theta_diff(0.1),
    // Step noise
    step_x_noise(0.01),
    step_y_noise(0.01),
    step_theta_noise(2 * M_PI / 180),
    // Kick
    kick_x_min(0.12),
    kick_x_max(0.22),
    kick_y_tol(0.04),
    kick_y_offset(0.08),
    kick_theta_offset(0),
    kick_theta_tol(10 * M_PI/180),
    kick_reward(0),
    // Viewing the ball
    viewing_angle(2*M_PI/3),
    no_view_reward(0),
    // Collision
    collision_x_front(0.12),
    collision_x_back(0.12),
    collision_y( 0.25),
    collision_reward(-3),
    // Misc
    out_of_space_reward(-100),
    step_reward(-1),
    init_min_dist(0.4),
    init_max_dist(0.95)
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
  bool theta_ok = -kick_theta_tol < kick_err && kick_err < kick_theta_tol;
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
  bool theta_ok = -kick_theta_tol < kick_err && kick_err < kick_theta_tol;
  return x_ok && y_ok && theta_ok;
}
  
void PolarApproach::setOdometry(const Eigen::MatrixXd& model)
{
  if (model.rows() != 3 && model.cols() != 4) {
    throw std::logic_error(
      "PolarApproach::setOdometry Invalid format");
  }
  odometry_coefficients = model;
}

bool PolarApproach::isTerminal(const Eigen::VectorXd & state) const
{
  return isOutOfSpace(state);
}

double  PolarApproach::getReward(const Eigen::VectorXd & state,
                                 const Eigen::VectorXd & action,
                                 const Eigen::VectorXd & dst) const
{
  (void)state;(void)action;
  if (isKickable(dst)  ) return kick_reward;
  if (isColliding(dst) ) return collision_reward;
  if (isOutOfSpace(dst)) return out_of_space_reward;
  double reward = step_reward;
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
  /// Initialize noise distributions
  std::uniform_real_distribution<double> step_x_noise_distrib    (-step_x_noise,
                                                                  step_x_noise);
  std::uniform_real_distribution<double> step_y_noise_distrib    (-step_y_noise,
                                                                  step_y_noise);
  std::uniform_real_distribution<double> step_theta_noise_distrib(-step_theta_noise,
                                                                  step_theta_noise);
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
  Eigen::VectorXd predicted_move = predictMotion(next_cmd);
  // Apply noise to get the real move
  Eigen::VectorXd real_move(3);
  real_move(0) = predicted_move(0) + step_x_noise_distrib(*engine);
  real_move(1) = predicted_move(1) + step_y_noise_distrib(*engine);
  real_move(2) = predicted_move(2) + step_theta_noise_distrib(*engine);
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
  (void)engine;
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  // Creating the distribution
  std::uniform_real_distribution<double> dist_distrib(init_min_dist, init_max_dist);
  std::uniform_real_distribution<double> angle_distrib(-M_PI, M_PI);
  // Generating random values
  double dist = dist_distrib(*engine);
  double ball_theta = angle_distrib(*engine);
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
  std::vector<double> odometry_coefficients_read;
  rosban_utils::xml_tools::try_read_vector<double>(node,
                                                   "odometry_coefficients",
                                                   odometry_coefficients_read);

  // If coefficients have been properly read, use them
  if (odometry_coefficients_read.size() == 12)
  {
    odometry_coefficients = Eigen::Map<Eigen::MatrixXd>(odometry_coefficients_read.data(), 3, 4);
  }
  else if (odometry_coefficients_read.size() != 0)
  {
    std::ostringstream oss;
    oss << "PolarApproach::from_xml: "
        << "invalid number of coefficients for 'odometry_coefficients'. "
        << "read: " << odometry_coefficients_read.size() << " expecting 12.";
    throw std::runtime_error(oss.str());
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
  rosban_utils::xml_tools::try_read<double>(node,"step_x_noise", step_x_noise);
  rosban_utils::xml_tools::try_read<double>(node,"step_y_noise", step_y_noise);
  rosban_utils::xml_tools::try_read<double>(node,"step_theta_noise", step_theta_noise);
  rosban_utils::xml_tools::try_read<double>(node,"kick_x_min", kick_x_min);
  rosban_utils::xml_tools::try_read<double>(node,"kick_x_max", kick_x_max);
  rosban_utils::xml_tools::try_read<double>(node,"kick_y_tol", kick_y_tol);
  rosban_utils::xml_tools::try_read<double>(node,"kick_y_offset", kick_y_offset);
  rosban_utils::xml_tools::try_read<double>(node,"kick_theta_tol", kick_theta_tol);
  rosban_utils::xml_tools::try_read<double>(node,"kick_theta_offset", kick_theta_offset);
  rosban_utils::xml_tools::try_read<double>(node,"kick_reward", kick_reward);
  rosban_utils::xml_tools::try_read<double>(node,"viewing_angle", viewing_angle);
  rosban_utils::xml_tools::try_read<double>(node,"no_view_reward", no_view_reward);
  rosban_utils::xml_tools::try_read<double>(node,"collision_x_front", collision_x_front);
  rosban_utils::xml_tools::try_read<double>(node,"collision_x_back", collision_x_back);
  rosban_utils::xml_tools::try_read<double>(node,"collision_y", collision_y);
  rosban_utils::xml_tools::try_read<double>(node,"collision_reward", collision_reward);
  rosban_utils::xml_tools::try_read<double>(node,"out_of_space_reward", out_of_space_reward);
  rosban_utils::xml_tools::try_read<double>(node,"step_reward", step_reward);
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

Eigen::VectorXd PolarApproach::predictMotion(const Eigen::VectorXd & walk_orders) const
{
  // In theory, robot makes 2 steps
  Eigen::VectorXd theoric_move = 2 * walk_orders;
  // If no odometry has been loaded, return walk orders
  if (odometry_coefficients.rows() <= 0)
  {
    return theoric_move;
  }

  // [1, walk_x, walk_y, walk_theta]
  Eigen::VectorXd augmented_state(4);
  augmented_state(0) = 1;
  augmented_state.segment(1,3) = theoric_move;
  // Apply odometry on augmented state
  return odometry_coefficients * augmented_state;
}

}
