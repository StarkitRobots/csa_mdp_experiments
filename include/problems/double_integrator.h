#include "rosban_csa_mdp/core/problem.h"

class DoubleIntegrator : public csa_mdp::Problem
{
public:
  DoubleIntegrator();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;
};
