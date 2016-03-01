#pragma once

#include "problems/blackbox_problem.h"

#include <random>

class DoubleIntegrator : public BlackBoxProblem
{
public:
  enum Version
  {
    // Also used in Ernst2005, it is the original version
    SantaMaria1998,
    // Bugged, need to request specifical informations to the authors
    Weinstein2012
  };

  Version version;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> noise_distribution;

  DoubleIntegrator(Version version = Version::SantaMaria1998);

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  Eigen::VectorXd getStartingState() override;
};
