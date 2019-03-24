#pragma once

#include "rhoban_csa_mdp/core/black_box_problem.h"

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
///
/// There are two different setups for the problem: see Mode
/// - Wide: The ball starts at a random orientation with a random speed and
///         robot is stationary
/// - Finish: The robot start with the ball in front of him, roughly aligned
///           toward the goal target, robot is moving at a speed similar to
///           the ball speed
class SSLDynamicBallApproach : public BlackBoxProblem
{
public:
  enum Mode
  {
    Wide,
    Finish
  };

  SSLDynamicBallApproach();

  bool isTerminal(const Eigen::VectorXd& state) const;

  double getReward(const Eigen::VectorXd& state, const Eigen::VectorXd& action, const Eigen::VectorXd& dst) const;

  Problem::Result getSuccessor(const Eigen::VectorXd& state, const Eigen::VectorXd& action,
                               std::default_random_engine* engine) const override;

  Eigen::VectorXd getStartingState(std::default_random_engine* engine) const override;

  Eigen::VectorXd getWideStartingState(std::default_random_engine* engine) const;
  Eigen::VectorXd getFinishStartingState(std::default_random_engine* engine) const;

  /// Is current state inside
  bool isFinishState(const Eigen::VectorXd& state) const;
  /// Is the ball kickable in given state ?
  bool isKickable(const Eigen::VectorXd& state) const;
  /// Is the robot colliding with the ball
  bool isColliding(const Eigen::VectorXd& state) const;
  /// Is the ball outside of the given limits
  bool isOutOfSpace(const Eigen::VectorXd& state) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
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

  // FINISH LIMITS
  // Limits on x-axis for entering the 'finish' mode [m]
  Eigen::Vector2d finish_x_limits;
  // Limit on absolute value for y-axis for entering in 'finish' mode [m]
  double finish_y_tol;
  // Limit on difference of speed (between ball and robot) along x_axis to enter
  // 'finish' mode [m/s]
  Eigen::Vector2d finish_diff_speed_x_limits;
  /// Difference of speed (between ball and robot) along y-axis (robot
  /// referential) has to be in [-finish_diff_speed_y_max,finish_diff_speed_y_max]
  /// to enter 'finish' mode [m/s]
  double finish_diff_speed_y_max;
  /// Angular speed has to be in [-finish_speed_theta_max, finish_speed_theta_max]
  /// to enter 'finish' mode [rad/s]
  double finish_speed_theta_max;

  // KICK LIMITS
  // Limits on x-axis to perform a kick [m]
  Eigen::Vector2d kick_x_limits;
  // Limit on absolute value for y-axis to perform a kick [m]
  double kick_y_tol;
  // Limit on difference of speed (between ball and robot) along x_axis to
  // perform a kick [m/s]
  Eigen::Vector2d kick_diff_speed_x_limits;
  /// Difference of speed along y-axis (robot referential) has to be in
  /// [-kick_diff_speed_y_max,kick_diff_speed_y_max] to allow kicks [m/s]
  double kick_diff_speed_y_max;
  /// Angular speed has to be in [-kick_speed_theta_max, kick_speed_theta_max]
  /// to allow kicks [rad/s]
  double kick_speed_theta_max;

  // BALL COLLISION
  /// The distance at which the ball collide with the robot [m]
  double collision_radius;
  /// If 'ball_x' is higher than 'collision_forward', there is no collision [m]
  /// This is because the robot is a circle cut by a segment parallel to x-axis
  double collision_forward;
  /// The reward received when colliding with the ball
  double collision_reward;

  /// The reward received when getting out of space
  double out_of_space_reward;

  /// The time step for control [s]
  double dt;

  // STATE INITIALIZATION
  Mode mode;

  /// Minimal distance at the beginning [m] (Wide mode)
  double ball_init_min_dist;

  /// Maximal distance at the beginning [m] (Wide mode)
  /// (lower than max_dist to ensure the robot does not loose during the first steps)
  double ball_init_max_dist;

  /// Maximal distance to target at start [m] (Wide mode)
  double target_init_max_dist;

  /// Position of the ball along x-axis during initialization [m] (Finish mode)
  /// ball_x in [ball_x_init(0), ball_x_init(1)]
  Eigen::Vector2d ball_x_init;

  /// Position of the ball along y_axis during initialization [m] (Finish mode)
  double ball_y_init;

  // Simple noise model (not even linear)
  // TODO: Ideally, noise model should be based on speed and acceleration (not supported by data)

  /// Noise on cartesian position for 1 second (independent on x and y) [cart]
  double cart_stddev;

  /// Noise on angular orientation  for 1 second [rad]
  double angular_stddev;

  /// The model used for ball speed evolution
  RollingBallModel rolling_ball_model;
};

}  // namespace csa_mdp
