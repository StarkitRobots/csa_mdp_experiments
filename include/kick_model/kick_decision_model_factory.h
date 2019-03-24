#pragma once

#include "kick_model/kick_decision_model.h"

#include "rhoban_utils/serialization/factory.h"

namespace csa_mdp
{
class KickDecisionModelFactory : public rhoban_utils::Factory<KickDecisionModel>
{
public:
  KickDecisionModelFactory();
};

}  // namespace csa_mdp
