#include "learning_machine/learning_machine_blackbox.h"

LearningMachineBlackBox::LearningMachineBlackBox()
  : LearningMachine()
{
}

LearningMachineBlackBox::~LearningMachineBlackBox()
{}

void LearningMachineBlackBox::prepareRun()
{
  LearningMachine::prepareRun();
  BlackBoxProblem * casted = dynamic_cast<BlackBoxProblem *>(problem.get());
  if (casted == nullptr) {
    throw std::logic_error("Trying to run a LearningMachineBlackBox on a NOT blackbox problem");
  }
  current_state = casted->getStartingState();
}

void LearningMachineBlackBox::applyAction(const Eigen::VectorXd &action)
{
  csa_mdp::Sample new_sample = problem->getSample(current_state, action);
  current_reward = new_sample.reward;
  current_state = new_sample.next_state;
}

void LearningMachineBlackBox::setProblem(std::unique_ptr<csa_mdp::Problem> new_problem)
{
  // Apply everything from the parent class
  LearningMachine::setProblem(std::move(new_problem));
  // Custom task
  bb_problem = std::dynamic_pointer_cast<BlackBoxProblem>(problem);
  if (!bb_problem) {
    throw std::logic_error("Trying to run a LearningMachineBlackBox on a NOT blackbox problem");
  }
}
