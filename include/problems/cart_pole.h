#include "rosban_csa_mdp/core/problem.h"

#include <random>


/// States are:
/// cart_pos, cart_speed, theta1, omega1, theta2, omega2, ...
/// Action is:
/// torque: (applied on cart)
class CartPole : public csa_mdp::Problem
{
public:
  CartPole();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  // maximal distance from center to cart
  static double max_pos;
};
