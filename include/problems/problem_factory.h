#pragma once

#include "problems/control_problem.h"
#include "problems/blackbox_problem.h"

class ProblemFactory
{
public:
  /// Throws an exception if it fails to create a problem
  static csa_mdp::Problem * build(const std::string &name);
  /// Throws an exception if it fails to create a problem
  static ControlProblem * buildControl(const std::string &name);
  /// Throws an exception if it fails to create a problem
  static BlackBoxProblem * buildBlackBox(const std::string &name);
};
