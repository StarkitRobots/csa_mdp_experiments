#pragma once

#include "mre_machine/mre_machine.h"

#include "rosban_utils/factory.h"

class MREMachineFactory : public rosban_utils::Factory<MREMachine>
{
public:
  MREMachineFactory();
};
