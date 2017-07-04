#include "kick_model/kick_model_factory.h"

#include "kick_model/classic_kick.h"


namespace csa_mdp
{

KickModelFactory::KickModelFactory()
{
  registerBuilder("ClassicKick",[](){return std::unique_ptr<KickModel>(new ClassicKick);});
}

}
