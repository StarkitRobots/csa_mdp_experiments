#include "problems/double_integrator.h"

#include <iostream>

DoubleIntegrator::DoubleIntegrator()
{
  Eigen::MatrixXd state_limits(2,2), action_limits(1,2);
  state_limits << -1, 1, -1, 1;
  action_limits << -1, 1;
  setStateLimits(state_limits);
  setActionLimits(action_limits);
}

bool DoubleIntegrator::isTerminal(const Eigen::VectorXd & state) const
{
  for (int i = 0; i < 2; i++)
  {
    if (state(i) < getStateLimits()(i,0) || state(i) > getStateLimits()(i,1))
      return true;
  }
  return false;
}

double DoubleIntegrator::getReward(const Eigen::VectorXd & state,
                                   const Eigen::VectorXd & action,
                                   const Eigen::VectorXd & dst)
{
  (void)state;
  if (isTerminal(dst)) {
    return -50;
  }
  double posCost = dst(0) * dst(0);
  double accCost = action(0) * action(0);
  return -(posCost + accCost);
}

Eigen::VectorXd DoubleIntegrator::getSuccessor(const Eigen::VectorXd & state,
                                               const Eigen::VectorXd & action)
{
  double integrationStep = 0.05;
  double simulationStep = 0.5;
  double elapsed = 0;
  Eigen::VectorXd currentState = state;
  while (elapsed < simulationStep)
  {
    double dt = std::min(simulationStep - elapsed, integrationStep);
    Eigen::Vector2d nextState;
    double vel = currentState(1);
    double acc = action(0);
    nextState(0) = currentState(0) + dt * vel;
    nextState(1) = currentState(1) + dt * acc;
    elapsed += dt;
    currentState = nextState;
  }
  return currentState;
}
