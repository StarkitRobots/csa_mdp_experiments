#pragma once

#include "problems/control_problem.h"
#include "problems/blackbox_problem.h"

#include "rosban_utils/factory.h"

class ProblemFactory : public rosban_utils::Factory<csa_mdp::Problem>
{
public:

  ProblemFactory();

  /// Throws an exception if it fails to create a problem
  ControlProblem * buildControl(const std::string &name);
  /// Throws an exception if it fails to create a problem
  BlackBoxProblem * buildBlackBox(const std::string &name);
};
