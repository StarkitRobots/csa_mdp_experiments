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

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const override;

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
