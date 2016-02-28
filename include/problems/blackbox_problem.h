#pragma once
#include "rosban_csa_mdp/core/problem.h"

class BlackBoxProblem : public csa_mdp::Problem
{
public:
  /// By which state should the episode start
  virtual Eigen::VectorXd getStartingState() = 0;
};
