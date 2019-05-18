#pragma once

#include "kick_model/kick_decision_model.h"

#include "starkit_utils/serialization/factory.h"

namespace csa_mdp
{
class KickDecisionModelFactory : public starkit_utils::Factory<KickDecisionModel>
{
public:
  KickDecisionModelFactory();
};

}  // namespace csa_mdp
