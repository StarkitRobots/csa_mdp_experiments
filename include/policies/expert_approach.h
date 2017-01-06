#pragma once

#include "rosban_csa_mdp/core/policy.h"

namespace csa_mdp
{

class ExpertApproach : public csa_mdp::Policy
{
public:
  enum class State
  { far, rotate, near };
  enum class Type
  { cartesian, polar };

  ExpertApproach();

  void init() override;

  Eigen::VectorXd getConfig() const;
  void setConfig(Type newType, const Eigen::VectorXd& params);

  /// Get raw action and update memory_state
  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state) override;
  /// Get raw action but do not update memory_state, state after action is
  /// written in 'final_state' (if it is not nullptr)
  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state,
                               State * final_state) const;
  /// Get raw action but do not update memory_state
  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state,
                               std::default_random_engine * external_engine) const override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

  // Approximation is not entirely appropriate, but it has the advantage of
  // having only three nodes
  // Significant changes are
  // - Hysteresis do not exist anymore
  // - Tolerance are not included anymore
  // - Use of ball_y in near is not available anymore
  // - Step is always step_max in far
  virtual std::unique_ptr<rosban_fa::FATree> extractFATree() const override;

  // Read type from the given string
  Type loadType(const std::string & type_str);

private:

  Type type;

  State memory_state;

  // Maximal step forward authorized
  double step_max;

  // Number of parameters which can modified (do not include step_max)
  static const int nb_parameters;

  // Radius
  /// Above this distance, go to far from rotate or near
  double max_rotate_radius;
  /// Below this distance, go to rotate from far
  double min_far_radius;
  /// Wished distance for rotate state
  double radius;
  // Global
  double step_p;
  // Far
  double far_theta_p;
  // Rotate
  double rotate_theta_p;
  double rotate_lateral_p;
  // Near
  double near_theta_p;
  double near_lateral_p;
  double stop_y_near;// Above this y, the robot has no forward / backward move
  double max_y_near;
  // Target
  double wished_x;
  double wished_y;
  /// Tolerance in rad to orientation error
  double target_theta_tol;
  /// Tolerance in rad to ball for 'good alignement'
  double ball_theta_tol;
};

std::string to_string(ExpertApproach::State state);
std::string to_string(ExpertApproach::Type state);

}
