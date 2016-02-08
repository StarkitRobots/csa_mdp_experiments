#include "problems/control_problem.h"

#include <random>


/// States are:
/// cart_pos, cart_speed, theta1, omega1, theta2, omega2, ...
/// Action is:
/// torque: (applied on cart)
class CartPole : public ControlProblem
{
public:
  CartPole();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  bool isValidStart(const Eigen::VectorXd &state) const override;

  Eigen::VectorXd getResetCmd(const Eigen::VectorXd &state) const override;

  // maximal distance from center to cart [m]
  static double max_pos;
  // maximal velocity of the cart [m/s]
  static double max_vel;
  // maximal torque applied by the cart [N]
  static double max_torque;
  // maximal distance from center to start trajectory [m]
  static double start_pos_tol;
  // maximal velocity at start trajectory [m/s]
  static double start_vel_tol;
};
