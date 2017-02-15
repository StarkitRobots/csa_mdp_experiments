#pragma once

#include "rosban_utils/serializable.h"

#include <Eigen/Core>

namespace csa_mdp
{

class KickModel : public rosban_utils::Serializable {
public:
  KickModel();

  virtual Eigen::MatrixXd getActionsLimits() const = 0;
  virtual std::vector<std::string> getActionsNames() const = 0;

  /// Throw an error if kick_parameters size is not adapted
  virtual void applyKick(double ball_start_x, double ball_start_y,
                         const Eigen::VectorXd & kick_parameters,
                         std::default_random_engine * engine,
                         double * final_ball_x, double * final_ball_y,
                         double * reward) const = 0;

  /// Apply kick with given power and direction, using noise
  /// and reward specified by member variables
  void applyKick(double ball_start_x, double ball_start_y,
                 double kick_power, double kick_theta,
                 std::default_random_engine * engine,
                 double * final_ball_x, double * final_ball_y,
                 double * reward) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;

protected:
  double kick_pow_rel_noise;
  double kick_direction_noise;
  double kick_reward;
};

}