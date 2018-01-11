#include "problems/control_problem.h"

#include <random>

namespace csa_mdp
{

/// States are:
/// cart_pos, cart_speed, theta, omega
/// Action is:
/// torque: (applied on cart)
class CartPole : public ControlProblem
{
public:

  enum class RewardType
  { Binary, Continuous, Pilco };

  CartPole();

  void updateLimits();

  bool isTerminal(const Eigen::VectorXd & state) const;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & dst) const;

  Result getSuccessor(const Eigen::VectorXd & state,
                      const Eigen::VectorXd & action,
                      std::default_random_engine * engine) const override;

  bool isValidStart(const Eigen::VectorXd &state) const override;

  Eigen::VectorXd getResetCmd(const Eigen::VectorXd &state) const override;

  void toJson(std::ostream & out) const override;
  void fromJson(TiXmlNode * node) override;
  std::string getClassName() const override;

private:
  // maximal distance from center to cart [m]
  double max_pos;
  // maximal velocity of the cart [m/s]
  double max_vel;
  // maximal torque applied by the cart [N]
  double max_torque;
  // maximal velocity of the axis [rad/s]
  double max_axis_vel;
  // maximal cart distance from center to start trajectory [m]
  double start_cart_pos_tol;
  // maximal cart velocity at start trajectory [m/s]
  double start_cart_vel_tol;
  // maximal pendulum angle to bottom to start trajectory [rad]
  double start_axis_pos_tol;
  // maximal angular velocity at start trajectory [rad/s]
  double start_axis_vel_tol;

  /// Pole length [m] (Required for pilco reward)
  double pole_length;  

  /// Which type of reward is used
  RewardType reward_type;
};

std::string to_string(CartPole::RewardType type);
CartPole::RewardType loadRewardType(const std::string &type);

}
