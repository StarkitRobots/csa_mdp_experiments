#include "problems/control_problem.h"

#include <random>

namespace csa_mdp
{

/// States are:
/// theta, omega
/// Action is:
/// torque: (applied on axis)
class InvertedPendulum : public ControlProblem
{
public:
  InvertedPendulum();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) const override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  bool isValidStart(const Eigen::VectorXd &state) const override;

  Eigen::VectorXd getResetCmd(const Eigen::VectorXd &state) const override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

  // maximal torque applied on axis [N*m]
  static double max_torque;
  // maximal velocity of the axis [rad/s]
  static double max_axis_vel;
  // maximal distance from center to start trajectory [rad]
  static double start_pos_tol;
  // maximal velocity at start trajectory [rad/s]
  static double start_vel_tol;
};

}
