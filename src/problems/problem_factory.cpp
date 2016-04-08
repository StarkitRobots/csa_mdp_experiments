#include "problems/problem_factory.h"

#include "problems/cart_pole.h"
#include "problems/cart_pole_stabilization.h"
#include "problems/double_integrator.h"
#include "problems/inverted_pendulum.h"
#include "problems/double_inverted_pendulum.h"

using csa_mdp::Problem;

ProblemFactory::ProblemFactory()
{
  registerBuilder("CartPole",
                  [](TiXmlNode *node) {(void)node;return new CartPole();});
  registerBuilder("CartPoleStabilization",
                  [](TiXmlNode *node) {(void)node;return new CartPoleStabilization();});
  registerBuilder("InvertedPendulum",
                  [](TiXmlNode *node) {(void)node;return new InvertedPendulum();});
  registerBuilder("DoubleInvertedPendulum",
                  [](TiXmlNode *node) {(void)node;return new DoubleInvertedPendulum();});
  registerBuilder("DoubleIntegrator",
                  [](TiXmlNode *node) {(void)node;return new DoubleIntegrator();});
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
