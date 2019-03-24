#include "learning_machine/learning_machine_blackbox.h"

#include "rhoban_random/tools.h"

namespace csa_mdp
{
LearningMachineBlackBox::LearningMachineBlackBox() : LearningMachine()
{
  engine = rhoban_random::getRandomEngine();
}

LearningMachineBlackBox::~LearningMachineBlackBox()
{
}

void LearningMachineBlackBox::prepareRun()
{
  LearningMachine::prepareRun();
  BlackBoxProblem* casted = dynamic_cast<BlackBoxProblem*>(problem.get());
  if (casted == nullptr)
  {
    throw std::logic_error("Trying to run a LearningMachineBlackBox on a NOT blackbox problem");
  }
  status.successor = casted->getStartingState(&engine);
  status.reward = 0;
  status.terminal = false;
}

void LearningMachineBlackBox::applyAction(const Eigen::VectorXd& action)
{
  status = problem->getSuccessor(status.successor, action, &engine);
}

void LearningMachineBlackBox::setProblem(std::unique_ptr<csa_mdp::Problem> new_problem)
{
  // Apply everything from the parent class
  LearningMachine::setProblem(std::move(new_problem));
  // Custom task
  bb_problem = std::dynamic_pointer_cast<BlackBoxProblem>(problem);
  if (!bb_problem)
  {
    throw std::logic_error("Trying to run a LearningMachineBlackBox on a NOT blackbox problem");
  }
}

std::string LearningMachineBlackBox::getClassName() const
{
  return "LearningMachineBlackBox";
}

}  // namespace csa_mdp
