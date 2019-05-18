#pragma once

#include "starkit_csa_mdp/core/black_box_problem.h"

#include <random>

namespace csa_mdp
{
class DoubleIntegrator : public BlackBoxProblem
{
public:
  enum Version
  {
    // Also used in Ernst2005, it is the original version
    SantaMaria1998,
    // Bugged, need to request specifical informations to the authors
    Weinstein2012
  };

  Version version;

  DoubleIntegrator(Version version = Version::SantaMaria1998);

  bool isTerminal(const Eigen::VectorXd& state) const;

  double getReward(const Eigen::VectorXd& state, const Eigen::VectorXd& action, const Eigen::VectorXd& dst) const;

  Problem::Result getSuccessor(const Eigen::VectorXd& state, const Eigen::VectorXd& action,
                               std::default_random_engine* engine) const override;

  Eigen::VectorXd getStartingState(std::default_random_engine* engine) const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const override;

private:
  /// Is the state random or is it determinist?
  bool random_start;
};

}  // namespace csa_mdp
