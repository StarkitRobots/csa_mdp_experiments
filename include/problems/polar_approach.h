#pragma once

#include "problems/blackbox_problem.h"

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

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) const override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  Eigen::VectorXd getStartingState() override;

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

  /// Return the predicted motion for the given walk orders (without noise)
  /// This method uses odometry coefficients if it has been initialized
  Eigen::VectorXd predictMotion(const Eigen::VectorXd & walk_orders) const;

  /// Ensure that limits are consistent with the parameters
  void updateLimits();
  /// Update maximal distance at which the ball is accepted
  void setMaxDist(double dist);

  /// Update motion odometry model
  void setOdometry(const Eigen::MatrixXd& model);

protected:
  // TODO: Use all parameters as members and implement from_xml + use dirty flag

  // PREDICTIVE ODOMETRY
  // structure is the following
  // offset_x, dx(dx), dx(dy), dx(dz)
  // offset_y, dy(dx), dy(dy), dy(dz)
  // offset_z, dz(dx), dz(dy), dz(dz)
  Eigen::MatrixXd odometry_coefficients;

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

  // NOISE
  double step_x_noise;
  double step_y_noise;
  double step_theta_noise;

  // TARGET PROPERTIES
  /// Minimal distance along x to kick
  double kick_x_min;
  /// Maximal distance along x to kick
  double kick_x_max;
  /// Ball tolerance along y axis for shooting
  double kick_y_tol;
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
};

}
