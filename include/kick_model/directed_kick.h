#pragma once

#include "kick_model/kick_decision_model.h"

namespace csa_mdp
{

/// Directed kick chooses only the direction of the kick
class DirectedKick : public KickDecisionModel {
public:

  DirectedKick();

  Eigen::VectorXd computeKickParameters(const Eigen::Vector2d & ball_pos,
                                        const Eigen::VectorXd & actions) const override;
  double computeKickDirection(const Eigen::Vector2d & ball_pos,
                              const Eigen::VectorXd & actions) const override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;
};

}
