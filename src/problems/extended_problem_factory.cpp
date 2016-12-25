#include "problems/extended_problem_factory.h"

#include "problems/approach.h"
#include "problems/polar_approach.h"
#include "problems/cart_pole.h"
#include "problems/cart_pole_stabilization.h"
#include "problems/simulated_cart_pole.h"
#include "problems/double_integrator.h"
#include "problems/inverted_pendulum.h"
#include "problems/double_inverted_pendulum.h"
#include "problems/kick_optimizer.h"
#include "problems/one_player_kick.h"

namespace csa_mdp
{

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
                       [](){return std::unique_ptr<Problem>(new Approach);}, false);
  registerExtraBuilder("polar_approach",
                       [](){return std::unique_ptr<Problem>(new PolarApproach);});
  registerExtraBuilder("cart_pole", [](){return std::unique_ptr<Problem>(new CartPole);});
  registerExtraBuilder("simulated_cart_pole",
                       [](){return std::unique_ptr<Problem>(new SimulatedCartPole);});
  registerExtraBuilder("cart_pole_stabilization",
                       [](){return std::unique_ptr<Problem>(new CartPoleStabilization);});
  registerExtraBuilder("inverted_pendulum",
                       [](){return std::unique_ptr<Problem>(new InvertedPendulum);}, false);
  registerExtraBuilder("double_inverted_pendulum",
                       [](){return std::unique_ptr<Problem>(new DoubleInvertedPendulum);}, false);
  registerExtraBuilder("double_integrator",
                       [](){return std::unique_ptr<Problem>(new DoubleIntegrator);});
  registerExtraBuilder("kick_optimizer",
                       [](){return std::unique_ptr<Problem>(new KickOptimizer);}, false);
  registerExtraBuilder("one_player_kick",
                       [](){return std::unique_ptr<Problem>(new OnePlayerKick);});
}

std::unique_ptr<ControlProblem> ExtendedProblemFactory::buildControl(const std::string &name)
{
  Problem * p = build(name).release();
  ControlProblem * result = dynamic_cast<ControlProblem *>(p);
  if (result == nullptr)
  {
    delete(result);
    throw std::runtime_error("Problem '" + name + "' is not a ControlProblem");
  }
  return std::unique_ptr<ControlProblem>(result);
}

std::unique_ptr<BlackBoxProblem> ExtendedProblemFactory::buildBlackBox(const std::string &name)
{
  Problem * p = build(name).release();
  BlackBoxProblem * result = dynamic_cast<BlackBoxProblem *>(p);
  if (result == nullptr)
  {
    delete(result);
    throw std::runtime_error("Problem '" + name + "' is not a BlackBoxProblem");
  }
  return std::unique_ptr<BlackBoxProblem>(result);
}

}
