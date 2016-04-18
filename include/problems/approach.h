#pragma once

#include "problems/blackbox_problem.h"

#include <Eigen/Core>

#include <functional>
#include <random>
#include <vector>

/// This problem consists of approaching a ball with a humanoid robot. The robot
/// in a 2 dimensional map and every dimension is expressed in the robot
/// referential. Since the robot needs to shoot toward a given target, it needs
/// to end in a position near the ball, while facing the target.
///
/// The robot needs to apply actions changing 'smoothly' in order to keep balance,
/// therefore, we define the action space as the difference toward previous action
class Approach : public BlackBoxProblem {
public:
  Approach();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  Eigen::VectorXd getStartingState() override;

private:
  std::default_random_engine generator;

  // STATE LIMITS
  /// The maximal distance to the ball along one of the axis
  static double max_pos;
  /// The minimal order of step_x [m/step]
  static double min_step_x;
  /// The maximal order for step_x [m/step]
  static double max_step_x;
  /// The maximal lateral step [m/step]
  static double max_step_y;
  /// The maximal rotation by step [rad/step]
  static double max_step_theta;

  // ACTION_LIMITS
  static double max_step_x_diff;
  static double max_step_y_diff;
  static double max_step_theta_diff;

  // NOISE
  static double step_x_noise;
  static double step_y_noise;
  static double step_theta_noise;

  // TARGET PROPERTIES
  /// Minimal distance along x to kick
  static double kick_x_min;
  /// Maximal distance along x to kick
  static double kick_x_max;
  /// Ball tolerance along y axis for shooting
  static double kick_y_tol;
  /// The maximal angle allowed for kicking
  static double kick_theta_tol;
  /// Reward received when reaching kick position
  static double kick_reward;

  // BALL VIEW
  /// The maximal angle for viewing the ball
  static double viewing_angle;
  /// the reward received when not seeing the ball.
  static double no_view_reward;

  // BALL COLLISION
  /// The distance at which the ball start colliding along x axis
  static double collision_x;
  /// The distance at which the ball start colliding along y axis
  static double collision_y;
  /// The reward received when colliding with the ball
  static double collision_reward;

  // OUT OF SPACE
  /// The reward received when getting out of space
  static double out_of_space_reward;

  /// Minimal distance at the beginning
  static double init_min_dist;

  /// In reality, there is a huge difference between the order given to the walk
  /// system and its result
  static double walk_gain;
};
