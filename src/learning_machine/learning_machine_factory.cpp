#include "learning_machine/learning_machine_factory.h"

#include "learning_machine/learning_machine_blackbox.h"
#include "learning_machine/learning_machine_controller.h"

LearningMachineFactory::LearningMachineFactory()
{
  registerBuilder("LearningMachineBlackBox",
                  [](){return std::unique_ptr<LearningMachine>(new LearningMachineBlackBox);});
  registerBuilder("LearningMachineController",
                  [](){return std::unique_ptr<LearningMachine>(new LearningMachineController);});
}
