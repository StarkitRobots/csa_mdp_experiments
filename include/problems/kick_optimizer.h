#pragma once

#include "problems/blackbox_problem.h"

#include "rosban_fa/function_approximator.h"

#include <memory>

/// This problem consist of choosing the best kick direction and power to
/// reduce the time spent before scoring a goal for a humanoid robot. Each time
/// step consists of: an approach toward the kick position and then a kick with
/// the provided kick power
///
/// Referentials:
/// - x axis: from our goal to the opposite goal
/// - y axis: moving laterally on the field (z aims to the roof of the field)
/// - (0,0)
///
/// The state space is the following:
/// 0. ball_x
/// 1. ball_y
/// 2. player1_x
/// 3. player1_y
/// 4. player1_theta
/// 5. player2_x
/// 6. player2_y
/// 7. player2_theta
///
/// The action space is the following
/// 0. kick_direction in [-pi, pi] (according to the coordinate)
/// 1. kick_power in [kick_power_min, kick_power_max]
class KickOptimizer : public BlackBoxProblem
{
public:
  KickOptimizer();

  /// Ball outside of the field 
  bool isTerminal(const Eigen::VectorXd & state) const override;

  /// To think: if simulation is used, it is not possible to retrieve the
  ///           move cost directly
  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) const override;

  /// Successor is sampled using noise on the kick (for the ball position)
  /// The kicker is the closest (the one with the smallest *getApproachReward*)
  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  /// At starting state:
  /// Player 1 and Player 2 are placed randomly on the field
  /// Ball is placed randomly
  Eigen::VectorXd getStartingState() override;

  /// Return the expected reward for the given kicker during approach
  /// toward kicking the ball with target angle
  double getApproachReward(const Eigen::VectorXd & state,
                           const Eigen::VectorXd & action,
                           int kicker) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:
  /// #KICK PROPERTIES
  /// Average distance of the minimum shoot in m
  double kick_power_min;
  /// Average distance of the maximum shoot in m
  double kick_power_max;
  /// Relative noise on the kick: traveled_dist = kick_power * (1 - uniform_rand(-kpn,kpn))
  double kick_dist_rel_noise;
  /// Angular noise on the kick: real_theta = theoric_theta + uniform_rand(-kdn, kdn)
  double kick_direction_noise;
  /// Distance between the ball and the robot when the robot is kicking
  double kick_range;

  /// #REWARD FUNCTION
  /// The reward of an approach:
  /// (ball_dist, ball_dir, target_angle, last_step_x, last_step_y, last_step_theta) -> reward
  std::unique_ptr<rosban_fa::FunctionApproximator> approach_cost;
  /// The reward of a kick
  double kick_reward;
  /// Goal reward
  double goal_reward;

  /// #FIELD PROPERTIES
  /// Total width of the field [m]
  double field_width;
  /// Total length of the field [m]
  double field_length;
  /// Goal width [m]
  double goal_width;
  /// Goal area: size along x-axis [m]
  double goal_area_size_x;
  /// Goal area: size along y-axis [m]
  double goal_area_size_y;
};
