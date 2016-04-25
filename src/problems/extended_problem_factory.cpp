#include "problems/extended_problem_factory.h"

#include "problems/approach.h"
#include "problems/cart_pole.h"
#include "problems/cart_pole_stabilization.h"
#include "problems/double_integrator.h"
#include "problems/inverted_pendulum.h"
#include "problems/double_inverted_pendulum.h"

using csa_mdp::Problem;

ExtendedProblemFactory::ExtendedProblemFactory()
{
}

void ExtendedProblemFactory::registerExtraProblems()
{
  // Allowing multiple calls without issues
  static bool performed = false;
  if (performed) return;
  performed = true;
  // Registering extra builders
  registerExtraBuilder("approach",
                       [](TiXmlNode *node) {(void)node;return new Approach();});
  registerExtraBuilder("cart_pole",
                       [](TiXmlNode *node)
                       {
                         Problem * p = new CartPole();
                         p->from_xml(node);
                         return p;
                       });
  registerExtraBuilder("cart_pole_stabilization",
                       [](TiXmlNode *node) {(void)node;return new CartPoleStabilization();});
  registerExtraBuilder("inverted_pendulum",
                       [](TiXmlNode *node) {(void)node;return new InvertedPendulum();});
  registerExtraBuilder("double_inverted_pendulum",
                       [](TiXmlNode *node) {(void)node;return new DoubleInvertedPendulum();});
  registerExtraBuilder("double_integrator",
                       [](TiXmlNode *node) {(void)node;return new DoubleIntegrator();});
}

ControlProblem * ExtendedProblemFactory::buildControl(const std::string &name)
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

BlackBoxProblem * ExtendedProblemFactory::buildBlackBox(const std::string &name)
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
