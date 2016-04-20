#pragma once

#include "rosban_csa_mdp/core/policy.h"

class ExpertApproach : public csa_mdp::Policy
{
public:
  enum class State
  { far, rotate, near };

  ExpertApproach();

  void init() override;

  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state) override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:

  State current_state;

  double step_max;

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
