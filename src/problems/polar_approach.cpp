#include "problems/polar_approach.h"

#include "kick_model/kick_model_collection.h"

#include "rhoban_utils/xml_tools.h"

#include <cmath>

using namespace rhoban_utils::xml_tools;

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

void PolarApproach::clearKickZones()
{
  kick_zones.clear();
}

void PolarApproach::addKickZone(const KickZone & kz)
{
  kick_zones.push_back(kz);
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

const Odometry & PolarApproach::getOdometry() const
{
  return odometry;
}
void PolarApproach::setOdometry(const Odometry & new_odometry)
{
  odometry = new_odometry;
}

bool PolarApproach::isTerminal(const Eigen::VectorXd & state) const
{
  bool collision_terminal = (terminal_collisions && isColliding(state));
  bool kick_terminal = false;
  if (kick_terminal_speed_factor > 0 && isKickable(state)) {
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
  if (kick_zones.size() == 0) {
    throw std::logic_error("PolarApproach::isKickable: No kick zones");
  }

  Eigen::Vector3d ball_state;
  ball_state << getBallX(state), getBallY(state), state(2);
  for (const KickZone & zone : kick_zones) {
    if (zone.isKickable(ball_state)) {
      return true;
    }
  }
  return false;
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

void PolarApproach::toJson(std::ostream & out) const {
  (void)out;
  throw std::logic_error("PolarApproach::toJson: not implemented");
}

void PolarApproach::fromJson(TiXmlNode * node)
{
  std::string odometry_path;
  try_read<std::string>(node, "odometry_path", odometry_path);

  // If coefficients have been properly read, use them
  if (odometry_path != "")
  {
    try {
      odometry.loadFromFile(odometry_path);
    }
    catch (const std::runtime_error & err) {
      std::ostringstream oss;
      oss << "PolarApproach::fromJson: failed to read odometry file '"
          << odometry_path << "'";
      throw std::runtime_error(oss.str());
    }
  }
  // Read kicks
  try_read_serializable_vector<KickZone>(node, "kick_zones", kick_zones);
  // Read internal properties
  try_read<double>(node,"max_dist", max_dist);
  try_read<double>(node,"min_step_x", min_step_x);
  try_read<double>(node,"max_step_x", max_step_x);
  try_read<double>(node,"max_step_y", max_step_y);
  try_read<double>(node,"max_step_theta", max_step_theta);
  try_read<double>(node,"max_step_x_diff", max_step_x_diff);
  try_read<double>(node,"max_step_y_diff", max_step_y_diff);
  try_read<double>(node,"max_step_theta_diff", max_step_theta_diff);
  try_read<double>(node,"kick_reward", kick_reward);
  try_read<double>(node,"kick_terminal_speed_factor", kick_terminal_speed_factor);
  try_read<double>(node,"viewing_angle", viewing_angle);
  try_read<double>(node,"no_view_reward", no_view_reward);
  try_read<double>(node,"collision_x_front", collision_x_front);
  try_read<double>(node,"collision_x_back", collision_x_back);
  try_read<double>(node,"collision_y", collision_y);
  try_read<double>(node,"collision_reward", collision_reward);
  try_read<bool>  (node,"terminal_collisions", terminal_collisions);
  try_read<double>(node,"out_of_space_reward", out_of_space_reward);
  try_read<double>(node,"walk_frequency", walk_frequency);
  try_read<double>(node,"init_min_dist", init_min_dist);
  try_read<double>(node,"init_max_dist", init_max_dist);

  std::vector<std::string> kick_zone_names;
  try_read_vector<std::string>(node, "kick_zone_names", kick_zone_names);

  if (kick_zone_names.size() > 0) {
    KickModelCollection kmc;
    kmc.load_file();
    for (const std::string & name : kick_zone_names) {
      kick_zones.push_back(kmc.getKickModel(name).getKickZone());
    }
  }

  // Update limits according to the new parameters
  updateLimits();
}

std::string PolarApproach::getClassName() const
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
