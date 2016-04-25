#include "problems/double_integrator.h"

#include "rosban_regression_forests/tools/random.h"

#include <iostream>

DoubleIntegrator::DoubleIntegrator(Version version_)
  : version(version_)
{
  Eigen::MatrixXd state_limits(2,2), action_limits(1,2);
  switch(version)
  {
    case SantaMaria1998:
      state_limits << -1, 1, -1, 1;
      action_limits << -1, 1;
      break;
    case Weinstein2012:
      // No bound provided in the article
      state_limits << -5, 5, -5, 5;
      action_limits << -1.5, 1.5;
      // Preparing random
      generator = regression_forests::get_random_engine();
      noise_distribution = std::uniform_real_distribution<double>(-0.1, 0.1);
      break;
  }
  setStateLimits(state_limits);
  setActionLimits(action_limits);
}

bool DoubleIntegrator::isTerminal(const Eigen::VectorXd & state) const
{
  if (version == Weinstein2012) return false;// No terminal state
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
  if (isTerminal(dst)) {
    return -50;
  }
  double posCost = state(0) * state(0);
  double accCost = action(0) * action(0);
  return -(posCost + accCost);
}

Eigen::VectorXd DoubleIntegrator::getSuccessor(const Eigen::VectorXd & state,
                                               const Eigen::VectorXd & action)
{
  double integrationStep = 0.05;
  double simulationStep = 0.5;
  double elapsed = 0;
  double acc = action(0);
  if (version == Version::Weinstein2012)
  {
    acc += noise_distribution(generator);
  }
  Eigen::VectorXd currentState = state;
  while (elapsed < simulationStep)
  {
    double dt = std::min(simulationStep - elapsed, integrationStep);
    Eigen::Vector2d nextState;
    double vel = currentState(1);
    nextState(0) = currentState(0) + dt * vel;
    nextState(1) = currentState(1) + dt * acc;
    elapsed += dt;
    currentState = nextState;
  }
  return currentState;
}

Eigen::VectorXd DoubleIntegrator::getStartingState()
{
  Eigen::VectorXd state(2);
  state << 1, 0;
  return state;
}

void DoubleIntegrator::to_xml(std::ostream & out) const {(void)out;}

void DoubleIntegrator::from_xml(TiXmlNode * node) {(void)node;}

std::string DoubleIntegrator::class_name() const
{
  return "double_integrator";
}
