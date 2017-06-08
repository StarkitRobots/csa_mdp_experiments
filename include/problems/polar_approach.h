#pragma once

#include "rosban_csa_mdp/core/black_box_problem.h"

#include "Odometry/Odometry.hpp"

#include <Eigen/Core>

#include <functional>
#include <random>
#include <vector>

namespace csa_mdp
{

/// This problem consists of approaching a ball with a humanoid robot. The robot
/// in a 2 dimensional map and every dimension is expressed in the robot
/// referential. Since the robot needs to shoot toward a given target, it needs
/// to end in a position near the ball, while facing the target.
///
/// The robot needs to apply actions changing 'smoothly' in order to keep balance,
/// therefore, we define the action space as the difference toward previous action
///
/// The state space is the following:
/// - ball_dist
/// - ball_dir
/// - target_angle
/// - last_step_x
/// - last_step_y
/// - last_step_theta
class PolarApproach : public BlackBoxProblem {
public:
  PolarApproach();

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
  /// Can the robot see the ball?
  bool seeBall(const Eigen::VectorXd & state) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

  static double getBallX(const Eigen::VectorXd & state);
  static double getBallY(const Eigen::VectorXd & state);

  /// Ensure that limits are consistent with the parameters
  void updateLimits();
  /// Update maximal distance at which the ball is accepted
  void setMaxDist(double dist);

  /// Does the position of the ball allows the robot to kick with the left foot?
  bool canKickLeftFoot(const Eigen::VectorXd & state) const;
  /// Does the position of the ball allows the robot to kick with the right foot?
  bool canKickRightFoot(const Eigen::VectorXd & state) const;

protected:
  /// The displacement and noise model
  Leph::Odometry odometry;

  // STATE LIMITS
  /// The maximal distance to the ball along one of the axis
  double max_dist;
  /// The minimal order of step_x [m/step]
  double min_step_x;
  /// The maximal order for step_x [m/step]
  double max_step_x;
  /// The maximal lateral step [m/step]
  double max_step_y;
  /// The maximal rotation by step [rad/step]
  double max_step_theta;

  // ACTION_LIMITS
  double max_step_x_diff;
  double max_step_y_diff;
  double max_step_theta_diff;

  // TARGET PROPERTIES
  /// Minimal distance along x to kick
  double kick_x_min;
  /// Maximal distance along x to kick
  double kick_x_max;
  /// Ball tolerance along y axis for shooting
  double kick_y_tol;
  /// Ball ideal offset in y for each foot: (offset for left_foot, -offset for right_foot)
  /// Warning: For lateral kicks, offset should be a negative value (kick with opposite foot)
  double kick_y_offset;
  /// Direction offset when kicking the ball: symetrical
  /// right_kick: kick_dir = robot_dir + kick_theta_offset
  /// left_kick : kick_dir = robot_dir - kick_theta_offset
  /// For forward kicks, value is expected to be 0
  /// For lateral kicks, value is expected to be pi/2
  double kick_theta_offset;
  /// The maximal angle allowed for kicking
  double kick_theta_tol;
  /// Reward received when reaching kick position
  double kick_reward;

  // BALL VIEW
  /// Ball is seen in [-viewing_angle, viewing_angle]
  double viewing_angle;
  /// The reward received when not seeing the ball.
  double no_view_reward;

  // BALL COLLISION
  /// The distance at which the ball start colliding along x axis (front of the robot)
  double collision_x_front;
  /// The distance at which the ball start colliding along x axis (back of the robot)
  double collision_x_back;
  /// The distance at which the ball start colliding along y axis
  double collision_y;
  /// The reward received when colliding with the ball
  double collision_reward;

  // OUT OF SPACE
  /// The reward received when getting out of space
  double out_of_space_reward;

  // Step reward
  /// This reward is received at each non-terminal step
  double step_reward;

  /// Minimal distance at the beginning
  double init_min_dist;

  /// Maximal distance at the beginning
  /// (not max_dist to ensure the robot does not loose during the first steps)
  double init_max_dist;

  /// Are collisions with the ball considered as terminal?
  bool terminal_collisions;
};

}
