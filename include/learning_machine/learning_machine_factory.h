#pragma once

#include "learning_machine/learning_machine.h"

#include "rosban_utils/factory.h"

class LearningMachineFactory : public rosban_utils::Factory<LearningMachine>
{
public:
  LearningMachineFactory();
};
