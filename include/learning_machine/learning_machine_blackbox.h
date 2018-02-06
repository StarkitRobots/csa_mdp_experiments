#pragma once
#include "learning_machine/learning_machine.h"

#include "rhoban_csa_mdp/core/black_box_problem.h"

#include <random>

namespace csa_mdp
{

class LearningMachineBlackBox : public LearningMachine
{
public:
  LearningMachineBlackBox();
  virtual ~LearningMachineBlackBox();

  virtual void prepareRun() override;
  virtual void applyAction(const Eigen::VectorXd &action) override;
  
  virtual void setProblem(std::unique_ptr<csa_mdp::Problem> problem) override;

  virtual std::string getClassName() const override;
protected:
  /// Access to another type of problem
  std::shared_ptr<BlackBoxProblem> bb_problem;
  /// Random generator
  std::default_random_engine engine;
};

}
