#include "problems/problem_factory.h"

#include "problems/cart_pole.h"
#include "problems/cart_pole_stabilization.h"
#include "problems/double_integrator.h"
#include "problems/inverted_pendulum.h"
#include "problems/double_inverted_pendulum.h"

using csa_mdp::Problem;

Problem * ProblemFactory::build(const std::string &name)
{
  if (name == "CartPole")
    return new CartPole();
  if (name == "CartPoleStabilization")
    return new CartPoleStabilization();
  if (name == "InvertedPendulum")
    return new InvertedPendulum();
  if (name == "DoubleInvertedPendulum")
    return new DoubleInvertedPendulum();
  if (name == "DoubleIntegrator")
    return new DoubleIntegrator();
  throw std::runtime_error("Problem '" + name + "' is not known by the ProblemFactory");
}

ControlProblem * ProblemFactory::buildControl(const std::string &name)
{
  Problem * p = build(name);
  ControlProblem * result = dynamic_cast<ControlProblem*>(p);
  if (result == nullptr)
  {
    delete(p);
    throw std::runtime_error("Problem '" + name + "' is not a ControlProblem");
  }
  return result;
}

BlackBoxProblem * ProblemFactory::buildBlackBox(const std::string &name)
{
  Problem * p = build(name);
  BlackBoxProblem * result = dynamic_cast<BlackBoxProblem*>(p);
  if (result == nullptr)
  {
    delete(p);
    throw std::runtime_error("Problem '" + name + "' is not a BlackBoxProblem");
  }
  return result;
}
