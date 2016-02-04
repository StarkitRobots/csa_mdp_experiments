#include "problems/cart_pole.h"

double CartPole::max_pos = 0.4;

CartPole::CartPole()
{
}

bool CartPole::isTerminal(const Eigen::VectorXd & state) const
{
  // Cart is not allowed to go too close to the borders
  if (std::fabs(state(0)) > max_pos)
    return true;
  return false;
}

double CartPole::getReward(const Eigen::VectorXd &state,
                           const Eigen::VectorXd &action,
                           const Eigen::VectorXd &dst)
{
  (void) action;
  if (isTerminal(dst) || isTerminal(state)) {
    return -50;
  }
  double cart_cost = std::fabs(dst(0) / max_pos);
  double poles_cost = 0;
  for (int i = 2; i < dst.rows(); i += 2)
  {
    poles_cost += std::fabs(dst(i) / M_PI);
  }
  return -(cart_cost + poles_cost);
}

Eigen::VectorXd CartPole::getSuccessor(const Eigen::VectorXd & state,
                                       const Eigen::VectorXd & action)
{
  (void) state;
  (void) action;
  throw std::runtime_error("Not implemented");
}
