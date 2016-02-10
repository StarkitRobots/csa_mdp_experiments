#include "problems/problem_factory.h"

#include "problems/cart_pole.h"
#include "problems/inverted_pendulum.h"

ControlProblem * ProblemFactory::build(const std::string &name)
{
  if (name == "CartPole")
    return new CartPole();
  if (name == "InvertedPendulum")
    return new InvertedPendulum();
  throw std::runtime_error("Problem '" + name + "' is not known by the ProblemFactory");
}
