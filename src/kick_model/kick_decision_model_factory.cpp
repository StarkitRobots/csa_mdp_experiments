#include "kick_model/kick_decision_model_factory.h"

//#include "kick_model/custom_power_kick.h"
#include "kick_model/final_kick.h"
//#include "kick_model/full_power_kick.h"


namespace csa_mdp
{

KickDecisionModelFactory::KickDecisionModelFactory()
{
//  registerBuilder("CustomPowerKick",[](){return std::unique_ptr<KickDecisionModel>(new CustomPowerKick);});
  registerBuilder("FinalKick",
                  [](){return std::unique_ptr<KickDecisionModel>(new FinalKick);});
//  registerBuilder("FullPowerKick",[](){return std::unique_ptr<KickDecisionModel>(new FullPowerKick);});
}

}
