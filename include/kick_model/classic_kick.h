#pragma once

#include "kick_model/kick_model.h"

namespace csa_mdp
{

/// Default kick has the following properties:
/// - Average results
///   - Distance traveled
///   - Direction of the kick (symetrical for right/left foot)
/// - Noise
///   - Relative on dist (multiplier)
///   - Absolute on dir
///
/// Parameters:
/// - Kick theta tol [rad]
class ClassicKick : public KickModel {
public:

  ClassicKick();

  using KickModel::applyKick;

  /// @Inherited
  virtual Eigen::Vector2d applyKick(const Eigen::Vector2d & ball_pos,
                                    double kick_dir,
                                    const Eigen::VectorXd & kick_parameters,
                                    std::default_random_engine * engine = nullptr) const override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:
  /// The average distance of the shoot [m]
  double kick_power;

  /// Direction to which the ball is kicked by the right foot
  /// Robot referential [rad]
  double right_kick_dir;

  /// Standard deviation for the distance multiplier
  /// shoot real power is kick_power * U(1,rel_dist_stddev)
  double rel_dist_stddev;

  /// Standard deviation for the direction of the kick [rad]
  double dir_stddev;

  //TODO: parameters from KickZone
};

}