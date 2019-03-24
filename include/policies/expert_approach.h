#pragma once

#include "rhoban_csa_mdp/core/policy.h"

namespace csa_mdp
{
class ExpertApproach : public csa_mdp::Policy
{
public:
  enum class State
  {
    far,
    rotate,
    near
  };
  enum class Type
  {
    cartesian,
    polar
  };

  ExpertApproach();

  void init() override;

  Eigen::VectorXd getConfig() const;
  void setConfig(Type newType, const Eigen::VectorXd& params);

  /// Get raw action and update memory_state
  Eigen::VectorXd getRawAction(const Eigen::VectorXd& state) override;
  /// Get raw action but do not update memory_state, state after action is
  /// written in 'final_state' (if it is not nullptr)
  Eigen::VectorXd getRawAction(const Eigen::VectorXd& state, State* final_state) const;
  /// Get raw action but do not update memory_state
  Eigen::VectorXd getRawAction(const Eigen::VectorXd& state,
                               std::default_random_engine* external_engine) const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const override;

  // WARNING: Does not work in cartesian mode approach
  // Approximation is not entirely appropriate, but it has the advantage of
  // having only three nodes
  // Significant changes are
  // - Hysteresis do not exist anymore
  // - Tolerance are not included anymore
  // - Use of ball_y in near is not available anymore
  //   - Since we use ball_theta instead of ball_y, ball_theta_tol has to be pretty large
  // - Step is always step_max in far
  virtual std::unique_ptr<rhoban_fa::FATree> extractFATree() const override;

  std::unique_ptr<rhoban_fa::FATree> extractClassicFATree() const;
  std::unique_ptr<rhoban_fa::FATree> extractLateralFATree() const;
  std::unique_ptr<rhoban_fa::FATree> extractOpportunistFATree() const;

  // Read type from the given string
  Type loadType(const std::string& type_str);

private:
  /// Is input polar or cartesian?
  Type type;

  /// What is the current state of the approach
  State memory_state;

  /// Currently, there is 3  approach types: classic, lateral and opportunist
  std::string approach_type;

  /// When approach_type is opportunist, lateral approach is chosen only if angle
  /// is larger than lateral_threshold
  double lateral_threshold;

  /// What is the kick direction with the right foot in robot referential [rad]
  double right_foot_kick_dir;

  /// Ball offset for kicking with left foot (opposite for right foot)
  /// WARNING: always provide positive values, y is inverted automatically for
  /// lateral kicks
  /// @see PolarApproach:kick_y_offset
  double foot_y_offset;

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
  double stop_y_near;  // Above this y, the robot has no forward / backward move
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

}  // namespace csa_mdp
