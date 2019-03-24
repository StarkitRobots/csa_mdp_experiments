#include "problems/extended_problem_factory.h"

#include "problems/ball_approach.h"
#include "problems/cart_pole_stabilization.h"
#include "problems/double_integrator.h"
#include "problems/kick_controler.h"
#include "problems/simulated_cart_pole.h"
#include "problems/ssl_ball_approach.h"
#include "problems/ssl_dynamic_ball_approach.h"

namespace csa_mdp
{
ExtendedProblemFactory::ExtendedProblemFactory()
{
}

void ExtendedProblemFactory::registerExtraProblems()
{
  // Allowing multiple calls without issues
  static bool performed = false;
  if (performed)
    return;
  performed = true;
  // Registering extra builders
  registerExtraBuilder("BallApproach", []() { return std::unique_ptr<Problem>(new BallApproach); });
  registerExtraBuilder("CartPoleStabilization", []() { return std::unique_ptr<Problem>(new CartPoleStabilization); });
  registerExtraBuilder("DoubleIntegrator", []() { return std::unique_ptr<Problem>(new DoubleIntegrator); });
  registerExtraBuilder("KickControler", []() { return std::unique_ptr<Problem>(new KickControler); });
  registerExtraBuilder("SimulatedCartPole", []() { return std::unique_ptr<Problem>(new SimulatedCartPole); });
  registerExtraBuilder("SSLBallApproach", []() { return std::unique_ptr<Problem>(new SSLBallApproach); });
  registerExtraBuilder("SSLDynamicBallApproach", []() { return std::unique_ptr<Problem>(new SSLDynamicBallApproach); });
}

std::unique_ptr<BlackBoxProblem> ExtendedProblemFactory::buildBlackBox(const std::string& name)
{
  Problem* p = build(name).release();
  BlackBoxProblem* result = dynamic_cast<BlackBoxProblem*>(p);
  if (result == nullptr)
  {
    delete (result);
    throw std::runtime_error("Problem '" + name + "' is not a BlackBoxProblem");
  }
  return std::unique_ptr<BlackBoxProblem>(result);
}

}  // namespace csa_mdp
