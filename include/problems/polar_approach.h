#pragma once

#include "problems/approach.h"

#include <Eigen/Core>

#include <functional>
#include <random>
#include <vector>

/// This problem allows to solve the 'approach' problem with a slight difference,
/// in place of being expressed using cartesian coordinates, it is expressed using
/// polar coordinates
///
/// The state space is the following:
/// - ball_distance
/// - ball_direction
/// - target_angle
/// - last_step_x
/// - last_step_y
/// - last_step_theta
class PolarApproach : public Approach {
public:
  PolarApproach();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  Eigen::VectorXd getStartingState() override;

  std::string class_name() const override;

protected:
  static Eigen::VectorXd stateToCartesian(const Eigen::VectorXd & polar_state);
  static Eigen::VectorXd stateFromCartesian(const Eigen::VectorXd & cart_state);

};
