#pragma once

#include "problems/control_problem.h"
#include "problems/blackbox_problem.h"

#include "rosban_csa_mdp/core/problem_factory.h"

namespace csa_mdp
{

class ExtendedProblemFactory : public csa_mdp::ProblemFactory
{
public:
  ExtendedProblemFactory();

  /// Throws an exception if it fails to create a problem
  std::unique_ptr<ControlProblem> buildControl(const std::string &name);
  /// Throws an exception if it fails to create a problem
  std::unique_ptr<BlackBoxProblem> buildBlackBox(const std::string &name);

  /// Needs to be called to allow using additionnal Problems
  static void registerExtraProblems();
};

}
