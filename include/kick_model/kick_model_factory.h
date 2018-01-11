#pragma once

#include "kick_model/kick_model.h"

#include "rhoban_utils/serialization/factory.h"

namespace csa_mdp
{

class KickModelFactory : public rhoban_utils::Factory<KickModel> {
public:
  KickModelFactory();
};

}
