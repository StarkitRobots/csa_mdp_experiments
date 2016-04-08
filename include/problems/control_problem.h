#pragma once
#include "rosban_csa_mdp/core/problem.h"

class ControlProblem : public csa_mdp::Problem
{
public:

  /// Is the current state valid for starting a run
  virtual bool isValidStart(const Eigen::VectorXd &state) const = 0;

  /// Which command needs to be applied in current state to go back to a valid start
  virtual Eigen::VectorXd getResetCmd(const Eigen::VectorXd &state) const = 0;

  /// Which command needs to be applied when the system is 'sleeping'
  virtual Eigen::VectorXd getNeutralCmd() const
    {
      return Eigen::VectorXd::Zero(getActionLimits().rows());
    }
};
