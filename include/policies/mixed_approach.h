#pragma once

#include "rosban_csa_mdp/core/policy.h"

#include <memory>

class MixedApproach : public csa_mdp::Policy
{
public:

  MixedApproach();

  /// Load the function
  void loadNearbyApproach(const std::string & fa_path);

  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state) override;
  Eigen::VectorXd getRawAction(const Eigen::VectorXd & state,
                               std::default_random_engine * engine) const override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:

  /// Distance from which the robot start applying far policy [m]
  double far_dist_min;
  /// Gain on rotation from far
  double far_theta_p;
  /// Maximal speed when far from the ball [m/step]
  double far_max_speed;

  /// The policy used for nearby states
  std::unique_ptr<csa_mdp::Policy> nearby_policy;

  /// Random engine for when necessary
  std::default_random_engine engine;
};
