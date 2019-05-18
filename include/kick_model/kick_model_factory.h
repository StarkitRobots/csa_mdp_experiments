#pragma once

#include "kick_model/kick_model.h"

#include "starkit_utils/serialization/factory.h"

namespace csa_mdp
{
class KickModelFactory : public starkit_utils::Factory<KickModel>
{
public:
  KickModelFactory();
};

}  // namespace csa_mdp
