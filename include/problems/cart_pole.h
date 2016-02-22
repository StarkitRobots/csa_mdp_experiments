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
  // maximal velocity of the axis [rad/s]
  static double max_axis_vel;
  // maximal cart distance from center to start trajectory [m]
  static double start_cart_pos_tol;
  // maximal cart velocity at start trajectory [m/s]
  static double start_cart_vel_tol;
  // maximal pendulum angle to bottom to start trajectory [rad]
  static double start_axis_pos_tol;
  // maximal angular velocity at start trajectory [rad/s]
  static double start_axis_vel_tol;
};
