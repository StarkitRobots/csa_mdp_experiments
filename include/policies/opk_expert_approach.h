#pragma once

#include "rosban_csa_mdp/core/policy.h"

/// An expert approach for the problem one_player_kick
///
/// This approach always aim at the goal with a specified power
class OPKExpertApproach : public csa_mdp::Policy
{
public:

  OPKExpertApproach();

  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state) override;
  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state,
                               std::default_random_engine * external_engine) const override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:

  /// Which is the power at which kick are required
  double kick_power;
  /// Length of the field
  double field_length;
};
