#include "learning_machine/learning_machine_factory.h"

#include "learning_machine/learning_machine_blackbox.h"

#ifndef NO_ROSBAN_CONTROL
#include "learning_machine/learning_machine_controller.h"
#endif

LearningMachineFactory::LearningMachineFactory()
{
  registerBuilder("LearningMachineBlackBox",
                  [](){return std::unique_ptr<LearningMachine>(new LearningMachineBlackBox);});
#ifndef NO_ROSBAN_CONTROL
  registerBuilder("LearningMachineController",
                  [](){return std::unique_ptr<LearningMachine>(new LearningMachineController);});
#endif
}
