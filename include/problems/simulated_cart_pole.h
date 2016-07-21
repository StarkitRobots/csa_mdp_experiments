#include "problems/blackbox_problem.h"

#include <random>


/// States are:
/// cart_pos, cart_speed, theta, omega
/// Action is:
/// torque: (applied on cart)
class SimulatedCartPole : public BlackBoxProblem
{
public:

  enum class RewardType
  { Binary, Continuous, Pilco };

  /// Default configuration is the pilco configuration
  SimulatedCartPole();

  void updateLimits();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  Eigen::VectorXd getStartingState() override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:
  /// maximal distance from center to cart [m]
  double max_pos;
  /// maximal velocity of the cart [m/s]
  double max_vel;
  /// maximal torque applied by the cart [N]
  double max_torque;
  /// maximal velocity of the axis [rad/s]
  double max_axis_vel;

  /// Pole length [m]
  double pole_length;
  /// Cart mass [kg]
  double cart_mass;
  /// Pendulum mass[kg]
  double pendulum_mass;
  /// Cart friction [N/(m/s)]
  double friction;
  /// Gravity acceleration [m/s^2]
  double gravity;

  /// Duration of an integration step [s]
  double integration_step;
  /// Duration of a simulation step [s] (1 / controlFrequency) 
  double simulation_step;

  /// Which type of reward is used
  RewardType reward_type;


  SimulatedCartPole::RewardType loadRewardType(const std::string &type);
};

std::string to_string(SimulatedCartPole::RewardType type);
