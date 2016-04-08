#include "mre_machine/mre_machine_blackbox.h"

MREMachineBlackBox::MREMachineBlackBox(std::shared_ptr<Config> config)
  : MREMachine(config)
{
}

void MREMachineBlackBox::prepareRun()
{
  Eigen::VectorXd current_state = problem->getStartingState();
}

void MREMachineBlackBox::applyAction(const Eigen::VectorXd &action)
{
  csa_mdp::Sample new_sample = problem->getSample(current_state, cmd);
  current_reward = new_sample.reward;
  current_state = new_sample.next_state;
}
