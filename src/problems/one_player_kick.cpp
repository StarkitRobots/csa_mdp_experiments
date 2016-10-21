#include "problems/one_player_kick.h"

#include "rosban_random/tools.h"

OnePlayerKick::OnePlayerKick()
  : kick_power_min(0.2),
    kick_power_max(2.0),
    kick_dist_rel_noise(0.1),
    kick_direction_noise(0.3),
    kick_range(0.1),
    kick_initial_noise(0.5),
    kick_reward(-5),
    goal_reward(0),
    failure_reward(-100),
    field_width(6),
    field_length(9),
    goal_width(2.1),
    goal_area_size_x(1),
    goal_area_size_y(5),
    goalkeeper_success_rate(0.5)
{
}

bool OnePlayerKick::isTerminal(const Eigen::VectorXd & state) const
{
  return std::fabs(state(0)) > field_width / 2;
}

double OnePlayerKick::getReward(const Eigen::VectorXd & state,
                                const Eigen::VectorXd & action,
                                const Eigen::VectorXd & dst) const
{
  double total_reward = 0;
  // Add cost of scoring a goal / failure if state is terminal
  if (isTerminal(dst)) {
    if (dst(0) > 0)
      total_reward += goal_reward;
    else
      total_reward += failure_reward;
  }
  // Add moving reward and kick reward if player had to move
  if (dst(2) < field_length / 2) {
    total_reward += getApproachReward(state, action);
    total_reward += kick_reward;
  }
  return total_reward;
}

Eigen::VectorXd OnePlayerKick::getSuccessor(const Eigen::VectorXd & state,
                                            const Eigen::VectorXd & action,
                                            std::default_random_engine * engine) const
{
  Eigen::VectorXd dst(5);
  // Checking state dimension
  if (state.rows() != 5) {
    std::ostringstream oss;
    oss << "OnePlayerKick::getSuccessor: invalid dimension for state "
        << state.rows() << " (expected 5)";
    throw std::logic_error(oss.str());
  }
  // Checking action dimension
  if (action.rows() != 2) {
    std::ostringstream oss;
    oss << "OnePlayerKick::getSuccessor: invalid dimension for action "
        << action.rows() << " (expected 2)";
    throw std::logic_error(oss.str());
  }
  // Importing the parameters of the state
  double ball_x = state(0);
  double ball_y = state(1);
  // Importing the parameters of the action
  double kick_theta = action(0);
  double kick_power = action(1);
  // T1: Adding noise to get ball_real
  double ball_real_x, ball_real_y;
  initialBallNoise(ball_x, ball_y, &ball_real_x, &ball_real_y, engine);
  // T2: Handle cases where ball was outside of the field
  if (std::fabs(ball_real_x) > field_length / 2 ||
      std::fabs(ball_real_y) > field_width / 2)
  {
    // Approach is not necessary and state is terminal
    dst(1) = 0;
    dst(2) = field_length;
    dst(3) = 0;
    dst(4) = 0;
    // Has a goal been scored?
    if (isGoal(ball_x, ball_y, ball_real_x, ball_real_y)) {
      dst(0) = field_length;
    }
    else {
      dst(0) = -field_length;
    }
    // No need to perform additional computations
    return dst;
  }
  // T3: Position of the kicker
  double cos_kick = cos(kick_theta);
  double sin_kick = sin(kick_theta);
  dst(2) = ball_x - cos_kick * kick_range;
  dst(3) = ball_y - sin_kick * kick_range;
  dst(4) = kick_theta;
  // T4: Probability of failure if inside goal area
  if (isGoalArea(dst(2), dst(3)))
  {
    std::uniform_real_distribution<double> failure_distrib(0,1);
    if (failure_distrib(*engine) < goalkeeper_success_rate)
    {
      dst(0) = -field_length;
      dst(1) = 0;
      return dst;
    }
  }
  // T5: Random noise on shoot
  double ball_final_x, ball_final_y;
  applyKick(ball_real_x, ball_real_y, kick_power, kick_theta, &ball_final_x, &ball_final_y, engine);
  dst(0) = ball_final_x;
  dst(1) = ball_final_y;
  // T6: Testing if ball left the field after kick
  if (std::fabs(ball_final_x) > field_length / 2 ||
      std::fabs(ball_final_y) > field_width / 2)
  {
    // State is terminal
    dst(1) = 0;
    // Has a goal been scored?
    if (isGoal(ball_real_x, ball_real_y, ball_final_x, ball_final_y)) {
      dst(0) = field_length;
    }
    else {
      dst(0) = -field_length;
    }
  }  
  return dst;
}

Eigen::VectorXd OnePlayerKick::getStartingState()
{
  Eigen::MatrixXd limits(3,2);
  limits <<
    -field_length /2, field_length/2,
    -field_width/2, field_width/2,
    -M_PI, M_PI; 
  std::vector<Eigen::VectorXd> random_positions;
  random_positions = rosban_random::getUniformSamples(limits, 3, &random_engine);
  Eigen::VectorXd state(8);
  state.segment(0,2) = random_positions[0].segment(0,2);// Ball pos
  state.segment(2,3) = random_positions[1];// P1 pos
  state.segment(5,3) = random_positions[2];// P2 pos
  return state;
}

double OnePlayerKick::getApproachReward(const Eigen::VectorXd & state,
                                        const Eigen::VectorXd & action) const
{
  // Computing basic properties
  double ball_x_field = state(0);
  double ball_y_field = state(1);
  double player_x_field = state(2);
  double player_y_field = state(3);
  double player_theta   = state(4);
  double kick_theta_field = action(0);
  // Computing position in robot referential
  double dx = ball_x_field - player_x_field;
  double dy = ball_y_field - player_y_field;
  double ball_dist = std::sqrt(dx * dx + dy * dy);
  double ball_dir_field = atan2(dy, dx);
  double ball_dir_robot = ball_dir_field - player_theta;
  double kick_theta_robot = kick_theta_field - player_theta;
  // Building the input for the approach problem (initial speed is 0)
  Eigen::VectorXd input = Eigen::VectorXd::Zero(6);
  input(0) = ball_dist;
  input(1) = ball_dir_robot;
  input(2) = kick_theta_robot;
  // Getting output
  return approach_cost->predict(input, 0);
}

void OnePlayerKick::to_xml(std::ostream & out) const
{
  (void) out;
  throw std::logic_error("OnePlayerKick::to_xml: unimplemented");
}

void OnePlayerKick::from_xml(TiXmlNode * node)
{
  (void) node;
  throw std::logic_error("OnePlayerKick::from_xml: unimplemented");
}

std::string OnePlayerKick::class_name() const
{
  return "OnePlayerKick";
}

void OnePlayerKick::initialBallNoise(double ball_x, double ball_y,
                                     double * ball_real_x, double * ball_real_y,
                                     std::default_random_engine * engine) const
{
  std::uniform_real_distribution<double> noise_distrib(0, kick_initial_noise);
  double dist = 2 * kick_initial_noise;
  double dx, dy;
  while (dist > kick_initial_noise)
  {
    dx = noise_distrib(*engine);
    dy = noise_distrib(*engine);
    dist = dx * dx + dy * dy;
  }
  *ball_real_x = ball_x + dx;
  *ball_real_y = ball_y + dy;
}

void OnePlayerKick::applyKick(double ball_real_x, double ball_real_y,
                              double kick_power, double kick_theta,
                              double * ball_final_x, double * ball_final_y,
                              std::default_random_engine * engine) const
{
  // Distribution initialization
  std::uniform_real_distribution<double> dist_noise(1.0 - kick_dist_rel_noise,
                                                    1.0 + kick_dist_rel_noise);
  std::uniform_real_distribution<double> theta_noise(-kick_direction_noise,
                                                     kick_direction_noise);
  // Sampling real parameters
  double ball_real_dist = kick_power * dist_noise(*engine);
  double ball_real_angle = kick_theta + theta_noise(*engine);

  // Estimating ball final position
  double cos_kick = cos(ball_real_angle);
  double sin_kick = sin(ball_real_angle);
  *ball_final_x = ball_real_x + cos_kick * ball_real_dist;
  *ball_final_y = ball_real_y + sin_kick * ball_real_dist;
}

bool OnePlayerKick::isGoal(double src_x, double src_y, double dst_x, double dst_y) const
{
  double dx = dst_x - src_x;
  // Impossible to score a goal by kicking backward or if ball did not cross the line
  if (dx <= 0 || dst_x < field_length / 2) return false;
  // Finding equation a*x +b;
  double dy = dst_y - src_x;
  double a = dy / dx;
  double b = src_y - a * src_x;
  // Finding intersection with goal line
  double intercept_y = a * field_length / 2 + b;
  // Is intersection inside the goal
  return std::fabs(intercept_y) < goal_width / 2;
}

bool OnePlayerKick::isGoalArea(double player_x, double player_y) const
{
  bool xOk = player_x > (field_length/2 - goal_area_size_x) && player_x < field_length/2;
  bool yOk = player_y < (field_width - goal_area_size_y) / 2;
}