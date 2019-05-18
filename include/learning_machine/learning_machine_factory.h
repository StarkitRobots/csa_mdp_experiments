#pragma once

#include "learning_machine/learning_machine.h"

#include "starkit_utils/serialization/factory.h"

namespace csa_mdp
{
class LearningMachineFactory : public starkit_utils::Factory<LearningMachine>
{
public:
  LearningMachineFactory();
};

}  // namespace csa_mdp
