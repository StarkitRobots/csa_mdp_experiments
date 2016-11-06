#include "problems/kick_optimizer.h"

#include "rosban_random/tools.h"

KickOptimizer::KickOptimizer()
  : kick_power_min(0.2),
    kick_power_max(2.0),
    kick_dist_rel_noise(0.1),
    kick_direction_noise(10),
    kick_range(0.1),
    kick_reward(-5),
    goal_reward(0),
    field_width(6),
    field_length(9),
    goal_width(2.1),
    goal_area_size_x(1),
    goal_area_size_y(5)
{
}

bool KickOptimizer::isTerminal(const Eigen::VectorXd & state) const
{
  bool x_ko = std::fabs(state(0)) > field_width / 2;
  bool y_ko = std::fabs(state(1)) > field_length / 2;
  return x_ko || y_ko;
}

double KickOptimizer::getReward(const Eigen::VectorXd & state,
                                const Eigen::VectorXd & action,
                                const Eigen::VectorXd & dst) const
{
  (void) dst;
  double cost1 = getApproachReward(state, action, 1);
  double cost2 = getApproachReward(state, action, 2);
  return std::max(cost1, cost2);
}

Eigen::VectorXd KickOptimizer::getSuccessor(const Eigen::VectorXd & state,
                                            const Eigen::VectorXd & action,
                                            std::default_random_engine * engine) const
{
  // Importing the parameters of the state
  if (state.rows() != 8) {
    std::ostringstream oss;
    oss << "KickOptimizer::getSuccessor: invalid dimension for state "
        << state.rows() << " (expected 8)";
    throw std::logic_error(oss.str());
  }
  double ball_x = state(0);
  double ball_y = state(1);
  // Importing the parameters of the kick from action
  if (action.rows() != 2) {
    std::ostringstream oss;
    oss << "KickOptimizer::getSuccessor: invalid dimension for action "
        << action.rows() << " (expected 2)";
    throw std::logic_error(oss.str());
  }
  double kick_theta = action(0);
  double kick_power = action(1);
  // Estimating shoot parameters with noise
  std::uniform_real_distribution<double> dist_noise(1.0 - kick_dist_rel_noise,
                                                    1.0 + kick_dist_rel_noise);
  std::uniform_real_distribution<double> theta_noise(-kick_direction_noise,
                                                     kick_direction_noise);
  double ball_real_dist = kick_power * dist_noise(*engine);
  double ball_real_angle = kick_theta + theta_noise(*engine);
  double cos_kick = cos(ball_real_angle);
  double sin_kick = sin(ball_real_angle);
  // Estimating ball final position
  double ball_final_x = ball_x + cos_kick * ball_real_dist;
  double ball_final_y = ball_y + sin_kick * ball_real_dist;
  // Estimating final position of the kicker
  double kicker_x = ball_x - cos_kick * kick_range;
  double kicker_y = ball_y - sin_kick * kick_range;
  // Choosing kicker
  double kicker;
  if (getApproachReward(state, action, 1) > getApproachReward(state, action, 2))
    kicker = 1;
  else
    kicker = 2;
  // Next state is similar to current state by default
  Eigen::VectorXd dst = state;
  dst(0) = ball_final_x;
  dst(1) = ball_final_y;
  int offset = kicker == 1 ? 0 : 3;
  dst(offset + 2) = kicker_x;
  dst(offset + 3) = kicker_y;
  dst(offset + 4) = ball_real_angle;
  return dst;
}

Eigen::VectorXd KickOptimizer::getStartingState()
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

double KickOptimizer::getApproachReward(const Eigen::VectorXd & state,
                                        const Eigen::VectorXd & action,
                                        int kicker) const
{
  // Computing basic properties
  double ball_x_field = state(0);
  double ball_y_field = state(1);
  int offset = kicker == 1 ? 0 : 3;
  double player_x_field = state(2 + offset);
  double player_y_field = state(3 + offset);
  double player_theta   = state(4 + offset);
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

void KickOptimizer::to_xml(std::ostream & out) const
{
  (void) out;
  //TODO
}

void KickOptimizer::from_xml(TiXmlNode * node)
{
  //TODO
  (void) node;
}

std::string KickOptimizer::class_name() const
{
  return "kick_optimizer";
}
