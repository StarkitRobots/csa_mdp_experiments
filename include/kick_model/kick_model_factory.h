#pragma once

#include "kick_model/kick_model.h"

#include "rosban_utils/factory.h"

namespace csa_mdp
{

class KickModelFactory : public rosban_utils::Factory<KickModel> {
public:
  KickModelFactory();
};

}
