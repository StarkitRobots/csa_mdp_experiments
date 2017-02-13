#pragma once

#include "kick_model/kick_model.h"

namespace csa_mdp
{

class CustomPowerKick : public KickModel {
public:
  virtual Eigen::MatrixXd getActionsLimits() const override;
  virtual std::vector<std::string> getActionsNames() const override;

  using KickModel::applyKick;

  /// Throw an error if kick_parameters size is not adapted
  virtual void applyKick(double ball_start_x, double ball_start_y,
                         const Eigen::VectorXd & kick_parameters,
                         std::default_random_engine * engine,
                         double * final_ball_x, double * final_ball_y,
                         double * kick_reward) const override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:
  /// The minimal kick power for the shoot
  double min_kick_power;
  /// The maximal kick power for the shoot
  double max_kick_power;
};

}
