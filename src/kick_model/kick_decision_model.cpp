#include "kick_model/kick_decision_model.h"

namespace csa_mdp
{

KickDecisionModel::KickDecisionModel() {}

const Eigen::MatrixXd & KickDecisionModel::getActionsLimits() const
{
  return action_limits;
}

const std::vector<std::string> & KickDecisionModel::getActionsNames() const
{
  return action_names;
}

}
