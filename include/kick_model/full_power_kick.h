#pragma once

#include "kick_model/kick_model.h"

namespace csa_mdp
{
class FullPowerKick : public KickModel
{
public:
  virtual Eigen::MatrixXd getActionsLimits() const override;
  virtual std::vector<std::string> getActionsNames() const override;

  using KickModel::applyKick;

  virtual double getWishedDir(double ball_x, double ball_y, const Eigen::VectorXd& kick_parameters) const;

  /// Throw an error if kick_parameters size is not adapted
  virtual void applyKick(double ball_start_x, double ball_start_y, const Eigen::VectorXd& kick_parameters,
                         std::default_random_engine* engine, double* final_ball_x, double* final_ball_y,
                         double* kick_reward) const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const override;

private:
  /// The average distance of the shoot
  double kick_power;
};

}  // namespace csa_mdp
