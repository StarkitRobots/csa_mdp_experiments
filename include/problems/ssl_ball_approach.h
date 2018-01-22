#pragma once

#include "rosban_csa_mdp/core/black_box_problem.h"

#include <Eigen/Core>

#include <functional>
#include <random>
#include <vector>

namespace csa_mdp
{

/// This problem consists of approaching a ball with an omni-directional wheeled
/// robot. The robot moves in a 2 dimensional map and every dimension is
/// expressed in the robot referential. Since the robot needs to shoot toward a
/// given target, it needs to end in a position near the ball, while facing the
/// target. Final move toward the robot is not handled here
///
/// The robot has limited acceleration and speed.
///
/// The state space is the following (everything is expressed in robot referential)
/// - ball_dist
/// - ball_dir
/// - target_angle
/// - vel_x
/// - vel_y
/// - vel_theta
///
/// The action space is the following (everything is expressed in robot referential)
/// - acc_x
/// - acc_y
/// - acc_theta
class SSLBallApproach : public BlackBoxProblem {
public:
  SSLBallApproach();

  bool isTerminal(const Eigen::VectorXd & state) const;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) const;

  Problem::Result getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  Eigen::VectorXd getStartingState(std::default_random_engine * engine) const override;

  /// Is the ball kickable
  bool isKickable(const Eigen::VectorXd & state) const;
  /// Is the robot colliding with the ball
  bool isColliding(const Eigen::VectorXd & state) const;
  /// Is the ball outside of the given limits
  bool isOutOfSpace(const Eigen::VectorXd & state) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const override;

  static double getBallX(const Eigen::VectorXd & state);
  static double getBallY(const Eigen::VectorXd & state);

  /// Ensure that limits are consistent with the parameters
  void updateLimits();
  /// Update maximal distance at which the ball is accepted
  void setMaxDist(double dist);

protected:
  // STATE LIMITS
  /// The maximal distance to the ball along one of the axis
  double max_dist;
  /// The maximal cartesian speed [m/s]
  double max_speed;
  /// The maximal rotation by step [rad/s]
  double max_speed_theta;

  // ACTION_LIMITS
  /// The maximal cartesian acceleration
  double max_acc;
  /// The maximal angular acceleration
  double max_acc_theta;

  // KICK PROPERTIES
  /// The tolerance for kick direction [rad]
  double kick_dir_tol;
  /// The maximal distance allowed for entering in kick phase [m]
  double kick_x_max;
  /// The tolerance for y position for the robot [m]
  double kick_y_tol;
  /// x speed has to be in [0,kick_speed_x_max] to allow kicks
  double kick_speed_x_max;
  /// Y speed has to be in [-kick_speed_y_max, kick_speed_y_max] to allow kicks
  double kick_speed_y_max;
  /// Angular speed has to be in [-kick_speed_theta_max, kick_speed_theta_max] to allow kicks
  double kick_speed_theta_max;
  /// Reward received when inside a kick position
  double kick_reward;

  // BALL COLLISION
  /// The distance at which the ball collide with the robot [m]
  double collision_radius;
  /// The reward received when colliding with the ball
  double collision_reward;

  /// The reward received when getting out of space
  double out_of_space_reward;

  /// The time step for control [s]
  double dt;

  /// Minimal distance at the beginning [m]
  double init_min_dist;

  /// Maximal distance at the beginning [m]
  /// (lower than max_dist to ensure the robot does not loose during the first steps)
  double init_max_dist;

  // Simple noise model (not even linear)
  // TODO: Ideally, noise model should be based on speed and acceleration (not supported by data)

  /// Noise on cartesian position for 1 second (independent on x and y) [cart]
  double cart_stddev;

  /// Noise on angular orientation  for 1 second [rad]
  double angular_stddev;
};

}
