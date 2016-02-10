#include "problems/control_problem.h"

#include <random>


/// States are:
/// cart_pos, cart_speed, theta1, omega1, theta2, omega2, ...
/// Action is:
/// torque: (applied on cart)
class InvertedPendulum : public ControlProblem
{
public:
  InvertedPendulum();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  bool isValidStart(const Eigen::VectorXd &state) const override;

  Eigen::VectorXd getResetCmd(const Eigen::VectorXd &state) const override;

  // maximal torque applied on axis [N*m]
  static double max_torque;
  // maximal velocity of the axis [rad/s]
  static double max_axis_vel;
  // maximal distance from center to start trajectory [rad]
  static double start_pos_tol;
  // maximal velocity at start trajectory [rad/s]
  static double start_vel_tol;
};
