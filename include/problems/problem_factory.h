#pragma once

#include "problems/control_problem.h"

class ProblemFactory
{
public:
  /// Throws an exception if it fails to create a problem
  static ControlProblem * build(const std::string &name);
};
