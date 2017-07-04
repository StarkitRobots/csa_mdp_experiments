#pragma once

#include "rosban_utils/serializable.h"

#include <Eigen/Core>

namespace csa_mdp
{

class KickDecisionModel : public rosban_utils::Serializable
{
public:

  KickDecisionModel();

  const Eigen::MatrixXd & getActionsLimits() const;
  const std::vector<std::string> & getActionsNames() const;

  /// Get the parameters of the kick
  /// ball_pos is in field referential [m]
  virtual Eigen::VectorXd computeKickParameters(const Eigen::Vector2d & ball_pos,
                                                const Eigen::VectorXd & actions) const = 0;

  /// Get the direction of the kick in the field basis [rad]
  /// ball_pos is in field referential [m]
  virtual double computeKickDirection(const Eigen::Vector2d & ball_pos,
                                      const Eigen::VectorXd & actions) const = 0;

protected:
  /// Limits for the actions
  Eigen::MatrixXd action_limits;
  /// Names of the actions
  std::vector<std::string> action_names;

};

}
