#pragma once

#include "rosban_csa_mdp/core/black_box_problem.h"

#include "kick_model/rolling_ball_model.h"

#include <Eigen/Core>

#include <functional>
#include <random>
#include <vector>

namespace csa_mdp
{

/// This problem consists of approaching a moving ball with an omni-directional
/// wheeled robot.
///
/// The state space is the following (all data are provided in robot referential):
/// - state[0:1] -> position of the ball (x,y)
/// - state[2:3] -> position of the target (x,y)
/// - state[4:6] -> speed of the robot (x,y,theta)
/// - state[7:8] -> speed of the ball (x,y)  
/// - state[9]   -> Kicking tolerance (do not change at each step)
class SSLDynamicBallApproach : public BlackBoxProblem {
public:
  SSLDynamicBallApproach();

  bool isTerminal(const Eigen::VectorXd & state) const;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) const;

  Problem::Result getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  Eigen::VectorXd getStartingState(std::default_random_engine * engine) const override;

  /// Is the ball kickable in given state ?
  bool isKickable(const Eigen::VectorXd & state) const;
  /// Is the robot colliding with the ball
  bool isColliding(const Eigen::VectorXd & state) const;
  /// Is the ball outside of the given limits
  bool isOutOfSpace(const Eigen::VectorXd & state) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const override;

  /// Ensure that limits are consistent with the parameters
  void updateLimits();
  /// Update maximal distance at which the ball is accepted
  void setMaxDist(double dist);

protected:
  // STATE LIMITS
  /// The maximal distance to the ball [m]
  double ball_max_dist;
  /// The maximal distance to the target [m]
  double target_max_dist;
  /// The maximal cartesian speed [m/s]
  double max_robot_speed;
  /// The maximal rotation by step [rad/s]
  double max_robot_speed_theta;
  /// The maximal speed considered for the ball [m/s]
  double max_ball_speed;
  /// The minimal tolerance allowed for kick direction [rad]
  double min_kick_dir_tol;
  /// The minimal tolerance allowed for kick direction [rad]
  double max_kick_dir_tol;

  // ACTION_LIMITS
  /// The maximal cartesian acceleration
  double max_acc;
  /// The maximal angular acceleration
  double max_acc_theta;

  // KICK LIMITS
  /// The maximal distance allowed for entering in kick phase [m]
  double kick_x_max;
  /// The tolerance for y position for the robot [m]
  double kick_y_tol;
  /// Difference of speed along x-axis (robot referential) has to be in
  /// [0,kick_diff_speed_x_max] to allow kicks [m/s]
  double kick_diff_speed_x_max;
  /// Difference of speed along y-axis (robot referential) has to be in
  /// [-kick_diff_speed_y_max,kick_diff_speed_y_max] to allow kicks [m/s]
  double kick_diff_speed_y_max;
  /// Angular speed has to be in [-kick_speed_theta_max, kick_speed_theta_max]
  /// to allow kicks [rad/s]
  double kick_speed_theta_max;

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
  double ball_init_min_dist;

  /// Maximal distance at the beginning [m]
  /// (lower than max_dist to ensure the robot does not loose during the first steps)
  double ball_init_max_dist;

  /// Maximal distance to target at start [m]
  double target_init_max_dist;

  // Simple noise model (not even linear)
  // TODO: Ideally, noise model should be based on speed and acceleration (not supported by data)

  /// Noise on cartesian position for 1 second (independent on x and y) [cart]
  double cart_stddev;

  /// Noise on angular orientation  for 1 second [rad]
  double angular_stddev;

  /// The model used for ball speed evolution
  RollingBallModel rolling_ball_model;
};

}
