#include "mre_machine/mre_machine_blackbox.h"

#include "problems/blackbox_problem.h"

MREMachineBlackBox::MREMachineBlackBox(std::shared_ptr<Config> config)
  : MREMachine(config)
{
}

void MREMachineBlackBox::prepareRun()
{
  MREMachine::prepareRun();
  BlackBoxProblem * bb_problem = dynamic_cast<BlackBoxProblem *>(problem.get());
  current_state = bb_problem->getStartingState();
}

void MREMachineBlackBox::applyAction(const Eigen::VectorXd &action)
{
  BlackBoxProblem * bb_problem = dynamic_cast<BlackBoxProblem *>(problem.get());
  csa_mdp::Sample new_sample = bb_problem->getSample(current_state, action);
  current_reward = new_sample.reward;
  current_state = new_sample.next_state;
}
