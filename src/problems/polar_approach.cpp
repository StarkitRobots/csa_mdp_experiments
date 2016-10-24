#include "problems/polar_approach.h"


// State limits
double PolarApproach::min_step_x     = -0.02;
double PolarApproach::max_step_x     =  0.04;
double PolarApproach::max_step_y     =  0.03;
double PolarApproach::max_step_theta =  0.4 ;
// Action limits
double PolarApproach::max_step_x_diff     = 0.03;
double PolarApproach::max_step_y_diff     = 0.03;
double PolarApproach::max_step_theta_diff = 0.4;
// Step noise
double PolarApproach::step_x_noise     = 0.01;
double PolarApproach::step_y_noise     = 0.01;
double PolarApproach::step_theta_noise = 0.02;
// Kick
double PolarApproach::kick_x_min     = 0.05   ;
double PolarApproach::kick_x_max     = 0.25   ;
double PolarApproach::kick_y_tol     = 0.10   ;
double PolarApproach::kick_theta_tol = M_PI/12;
double PolarApproach::kick_reward = 0;
// Viewing the ball
double PolarApproach::viewing_angle  = 2*M_PI/3;
double PolarApproach::no_view_reward = -0.01   ;
// Collision
double PolarApproach::collision_x      =  0.05;
double PolarApproach::collision_y      =  0.25;
double PolarApproach::collision_reward = -3;
// Misc
double PolarApproach::out_of_space_reward = -100;
double PolarApproach::step_reward         = -1;
double PolarApproach::init_min_dist = 0.4;
double PolarApproach::init_max_dist = 0.95;
double PolarApproach::walk_gain = 3;

// TODO: externalize
static double normalizeAngle(double value)
{
  while (value > M_PI)
  {
    value -= 2 * M_PI;
  }
  while (value < -M_PI)
  {
    value += 2 * M_PI;
  }
  return value;
}


PolarApproach::PolarApproach()
  : max_dist(1.0)
{
  updateLimits();

  setStateNames({"ball_dist", "ball_dir", "target_angle", "step_x", "step_y", "step_theta"});
  setActionNames({"d_step_x","d_step_y","d_step_theta"});
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
  setActionLimits(action_limits);
}

void PolarApproach::setMaxDist(double dist)
{
  max_dist = dist;
  updateLimits();
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

Eigen::VectorXd PolarApproach::getSuccessor(const Eigen::VectorXd & state,
                                            const Eigen::VectorXd & action,
                                            std::default_random_engine * engine) const
{
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
    // Action applies a delta on step
    next_cmd(dim) = action(dim) + state(dim + 3);
    // Ensuring that final action is inside of the bounds
    const Eigen::MatrixXd & limits = getStateLimits();
    double min_cmd = limits(dim + 3, 0);
    double max_cmd = limits(dim + 3, 1);
    next_cmd(dim) = std::min(max_cmd, std::max(min_cmd, next_cmd(dim)));
  }
  // Apply a linear modification (from theory to 'reality') and noise
  Eigen::VectorXd real_move(3);
  real_move(0) = next_cmd(0) * walk_gain + step_x_noise_distrib(*engine);
  real_move(1) = next_cmd(1) * walk_gain + step_y_noise_distrib(*engine);
  real_move(2) = next_cmd(2) + step_theta_noise_distrib(*engine);// No walk gain for theta
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
  return next_state;
}

Eigen::VectorXd PolarApproach::getStartingState()
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  // Creating the distribution
  std::uniform_real_distribution<double> dist_distrib(init_min_dist, init_max_dist);
  // Ball is always in sight at the beginning of an experiment
  std::uniform_real_distribution<double> angle_distrib(-viewing_angle,
                                                       viewing_angle);
  // Generating random values
  double dist = dist_distrib(random_engine);
  double ball_theta = angle_distrib(random_engine);
  double target_theta = angle_distrib(random_engine);
  // Updating state
  state(0) = dist;
  state(1) = ball_theta;
  state(2) = target_theta;

  return state;
}

bool PolarApproach::isKickable(const Eigen::VectorXd & state) const
{
  double ball_x = getBallX(state);
  double ball_y = getBallY(state);
  double theta  = state(2);
  bool x_ok = ball_x > kick_x_min && ball_x < kick_x_max;
  bool y_ok = std::fabs(ball_y) < kick_y_tol;
  bool theta_ok = - kick_theta_tol < theta && theta < kick_theta_tol;
  return x_ok && y_ok && theta_ok;
}

bool PolarApproach::isColliding(const Eigen::VectorXd & state) const
{
  double ball_x = getBallX(state);
  double ball_y = getBallY(state);
  bool x_ko = std::fabs(ball_x) < collision_x;
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

void PolarApproach::to_xml(std::ostream & out) const {(void)out;}

void PolarApproach::from_xml(TiXmlNode * node) {(void)node;}

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

