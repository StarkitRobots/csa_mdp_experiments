#include "problems/double_integrator.h"

#include "rosban_random/tools.h"

#include <iostream>

namespace csa_mdp
{

DoubleIntegrator::DoubleIntegrator(Version version_)
  : version(version_), random_start(false)
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
      break;
  }
  setStateLimits(state_limits);
  setActionLimits({action_limits});
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
                                   const Eigen::VectorXd & dst) const
{
  if (isTerminal(dst)) {
    return -50;
  }
  double posCost = state(0) * state(0);
  double accCost = action(0) * action(0);
  return -(posCost + accCost);
}

Problem::Result DoubleIntegrator::getSuccessor(const Eigen::VectorXd & state,
                                               const Eigen::VectorXd & action,
                                               std::default_random_engine * engine) const
{
  if (action.rows() != 2) {
    std::ostringstream oss;
    oss << "DoubleIntegrator::getSuccessor: "
        << "Invalid dimensions for action, expecting 2, got " << action.rows();
    throw std::runtime_error(oss.str());
  }

  std::uniform_real_distribution<double> noise_distribution;
  if (version == Weinstein2012)
  {
      noise_distribution = std::uniform_real_distribution<double>(-0.1, 0.1);
  }
  double integrationStep = 0.05;
  double simulationStep = 0.5;
  double elapsed = 0;
  double acc = action(1);
  if (version == Version::Weinstein2012)
  {
    acc += noise_distribution(*engine);
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
  Problem::Result result;
  result.successor = currentState;
  result.reward = getReward(state, action, currentState);
  result.terminal = isTerminal(currentState);
  return result;
}

Eigen::VectorXd DoubleIntegrator::getStartingState(std::default_random_engine * engine) const
{
  (void) engine;
  if (random_start) {
    return rosban_random::getUniformSamplesMatrix(getStateLimits(), 1, engine);
  }
  Eigen::VectorXd state(2);
  state << 1, 0;
  return state;
}

Json::Value DoubleIntegrator::toJson() const {
  Json::Value v;
  v["random_start"] =  random_start;
  return v;
}

void DoubleIntegrator::fromJson(const Json::Value & v, const std::string & dir_name) {
  (void)dir_name;
  rhoban_utils::tryRead(v, "random_start", &random_start);
}

std::string DoubleIntegrator::getClassName() const
{
  return "double_integrator";
}

}
