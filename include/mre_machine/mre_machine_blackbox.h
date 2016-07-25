#pragma once

#include "mre_machine/mre_machine.h"

class MREMachineBlackBox : public MREMachine
{
public:
  MREMachineBlackBox(std::shared_ptr<Config> config);

  virtual void prepareRun() override;
  virtual void applyAction(const Eigen::VectorXd &action) override;
  
};
