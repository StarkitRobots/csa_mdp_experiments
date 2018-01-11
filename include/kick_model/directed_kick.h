#pragma once

#include "kick_model/kick_decision_model.h"

namespace csa_mdp
{

/// Directed kick chooses only the direction of the kick
class DirectedKick : public KickDecisionModel {
public:

  DirectedKick();

  /// informations content is not used (no need for additional informations)
  Eigen::VectorXd computeKickParameters(const Eigen::VectorXd & informations,
                                        const Eigen::VectorXd & actions) const override;

  /// informations content is not used (no need for additional informations)
  double computeKickDirection(const Eigen::VectorXd & informations,
                              const Eigen::VectorXd & actions) const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const override;
};

}
