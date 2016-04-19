#include "problems/approach.h"


// State limits
double Approach::max_pos        =  3   ;
double Approach::min_step_x     = -0.02;
double Approach::max_step_x     =  0.04;
double Approach::max_step_y     =  0.03;
double Approach::max_step_theta =  0.2 ;
// Action limits
double Approach::max_step_x_diff     = 0.02;
double Approach::max_step_y_diff     = 0.02;
double Approach::max_step_theta_diff = 0.05;
// Step noise
double Approach::step_x_noise     = 0.01;
double Approach::step_y_noise     = 0.01;
double Approach::step_theta_noise = 0.02;
// Kick
double Approach::kick_x_min     = 0.1    ;
double Approach::kick_x_max     = 0.2    ;
double Approach::kick_y_tol     = 0.05   ;
double Approach::kick_theta_tol = M_PI/30;
double Approach::kick_reward = 0;
// Viewing the ball
double Approach::viewing_angle  = 2*M_PI/3;
double Approach::no_view_reward = -0.01   ;
// Collision
double Approach::collision_x      =  0.1;
double Approach::collision_y      =  0.3;
double Approach::collision_reward = -100;
// Misc
double Approach::out_of_space_reward = -100;
double Approach::step_reward         = -1;
double Approach::init_min_dist = 0.4;
double Approach::walk_gain = 3;

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


Approach::Approach()
{
  Eigen::MatrixXd state_limits(6,2), action_limits(3,2);
  state_limits <<
    -max_pos, max_pos,
    -max_pos, max_pos,
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

  setStateNames({"ball_x", "ball_y", "target_angle", "step_x", "step_y", "step_theta"});
  setActionNames({"d_step_x","d_step_y","d_step_theta"});

  step_x_noise_distrib     = std::uniform_real_distribution<double>(-step_x_noise,
                                                                    step_x_noise);
  step_y_noise_distrib     = std::uniform_real_distribution<double>(-step_y_noise,
                                                                    step_y_noise);
  step_theta_noise_distrib = std::uniform_real_distribution<double>(-step_theta_noise,
                                                                    step_theta_noise);
}

bool Approach::isTerminal(const Eigen::VectorXd & state) const
{
  return isKickable(state) || isColliding(state) || isOutOfSpace(state);
}

double  Approach::getReward(const Eigen::VectorXd & state,
                            const Eigen::VectorXd & action,
                            const Eigen::VectorXd & dst)
{
  (void)state;(void)action;
  if (isKickable(dst)  ) return kick_reward;
  if (isColliding(dst) ) return collision_reward;
  if (isOutOfSpace(dst)) return out_of_space_reward;
  double reward = step_reward;
  if (!seeBall(dst)    ) reward += no_view_reward;
  return reward;
}

Eigen::VectorXd Approach::getSuccessor(const Eigen::VectorXd & state,
                                       const Eigen::VectorXd & action)
{
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
  real_move(0) = next_cmd(0) * walk_gain + step_x_noise_distrib(random_engine);
  real_move(1) = next_cmd(1) * walk_gain + step_y_noise_distrib(random_engine);
  real_move(2) = next_cmd(2) + step_theta_noise_distrib(random_engine);// No walk gain for theta
  // Apply the real move
  Eigen::VectorXd next_state(6);
  // Apply rotation first
  double delta_theta = real_move(2);
  next_state(2) = normalizeAngle(state(2) - delta_theta);
  next_state(0) = state(0) * cos(-delta_theta) - state(1) * sin(-delta_theta);
  next_state(1) = state(0) * sin(-delta_theta) + state(1) * cos(-delta_theta);
  // Then, apply translation
  next_state.segment(0,2) = next_state.segment(0,2) - real_move.segment(0,2);
  // Update cmd
  next_state.segment(3,3) = next_cmd;
  return next_state;
}

Eigen::VectorXd Approach::getStartingState()
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  // Creating the distribution
  std::uniform_real_distribution<double> dist_distrib(init_min_dist, max_pos);
  std::uniform_real_distribution<double> angle_distrib(-M_PI,M_PI);
  // Generating random values
  double dist = dist_distrib(random_engine);
  double ball_theta = angle_distrib(random_engine);
  double target_theta = angle_distrib(random_engine);
  // Updating state
  state(0) = cos(ball_theta) * dist;
  state(1) = sin(ball_theta) * dist;
  state(2) = target_theta;

  return state;
}

bool Approach::isKickable(const Eigen::VectorXd & state) const
{
  double ball_x = state(0);
  double ball_y = state(1);
  double theta  = state(2);
  bool x_ok = ball_x > kick_x_min && ball_x < kick_x_max;
  bool y_ok = std::fabs(ball_y) < kick_y_tol;
  bool theta_ok = - kick_theta_tol < theta && theta < kick_theta_tol;
  return x_ok && y_ok && theta_ok;
}

bool Approach::isColliding(const Eigen::VectorXd & state) const
{
  double ball_x = state(0);
  double ball_y = state(1);
  bool x_ko = std::fabs(ball_x) < collision_x;
  bool y_ko = std::fabs(ball_y) < collision_y;
  return x_ko && y_ko;
}

bool Approach::isOutOfSpace(const Eigen::VectorXd & state) const
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

bool Approach::seeBall(const Eigen::VectorXd & state) const
{
  double angle = atan2(state(1), state(0));
  return angle < viewing_angle && angle > -viewing_angle;
}
