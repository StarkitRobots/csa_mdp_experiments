#include "learning_machine/learning_machine_factory.h"

#include "learning_machine/learning_machine_blackbox.h"

namespace csa_mdp
{
LearningMachineFactory::LearningMachineFactory()
{
  registerBuilder("LearningMachineBlackBox",
                  []() { return std::unique_ptr<LearningMachine>(new LearningMachineBlackBox); });
}

}  // namespace csa_mdp
