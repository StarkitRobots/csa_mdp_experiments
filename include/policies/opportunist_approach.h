#pragma once

#include "rosban_csa_mdp/core/policy.h"

#include "problems/polar_approach.h"

namespace csa_mdp
{

class OpportunistApproach : public csa_mdp::Policy
{
public:
  // Each available option is made of a policy and its associated problem
  typedef std::pair<std::unique_ptr<csa_mdp::Policy>,
                    csa_mdp::PolarApproach> Option;
  
  OpportunistApproach();

  
  /// Get raw action but do not update memory_state
  Eigen::VectorXd getRawAction(const Eigen::VectorXd &state,
                               std::default_random_engine * external_engine) const override;
  
  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:
  /// Available approach options
  std::vector<Option> options;

  /// Number of rollouts performed by
  int nb_rollouts;

  /// Horizon used for the rollout
  int horizon;  
};

}
