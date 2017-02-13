#include "kick_model/kick_model_factory.h"

#include "kick_model/custom_power_kick.h"
#include "kick_model/full_power_kick.h"

namespace csa_mdp
{

KickModelFactory::KickModelFactory()
{
  registerBuilder("CustomPowerKick",[](){return std::unique_ptr<KickModel>(new CustomPowerKick);});
  registerBuilder("FullPowerKick",[](){return std::unique_ptr<KickModel>(new FullPowerKick);});
}

}
