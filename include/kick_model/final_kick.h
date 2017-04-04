#pragma once

#include "kick_model/kick_model.h"

namespace csa_mdp
{

class FinalKick : public KickModel {
public:

  FinalKick();

  virtual Eigen::MatrixXd getActionsLimits() const override;
  virtual std::vector<std::string> getActionsNames() const override;

  using KickModel::applyKick;

  virtual double getWishedDir(double ball_x, double ball_y,
                              const Eigen::VectorXd & kick_parameters) const;

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
  /// The average distance of the shoot
  double kick_power;
  
  /// The position of the opponent goal in x (expecting field_length / 2)
  double goal_x;

  /// The maximal value for target_y (expecting goal_width / 2)
  double max_y;
};

}
