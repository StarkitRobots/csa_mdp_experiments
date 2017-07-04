#pragma once

#include "kick_model/kick_decision_model.h"

#include "rosban_utils/factory.h"

namespace csa_mdp
{

class KickDecisionModelFactory : public rosban_utils::Factory<KickDecisionModel> {
public:
  KickDecisionModelFactory();
};

}
