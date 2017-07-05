#pragma once

#include "rosban_utils/serializable.h"

#include <Eigen/Core>

#include <random>

namespace csa_mdp
{

/// Provided states follow this form (inside player_referential
/// 0: ball_x   [m]
/// 1: ball_y   [m]
/// 2: kick_dir [rad]
class KickZone : public rosban_utils::Serializable
{
public:

  KickZone();

  /// Can the robot shoot with any of the foot?
  bool isKickable(const Eigen::Vector3d & state) const;

  /// Can the robot shoot with any of the foot?
  /// ball_pos is in field referential [m]
  /// player_state is in field referential [m][m][rad]
  /// kick_dir is in field_referential [rad]
  bool isKickable(const Eigen::Vector2d & ball_pos,
                  const Eigen::Vector3d & player_state,
                  double kick_dir) const;

  /// Does the position of the ball allows the robot to kick with the left foot?
  bool canKickLeftFoot(const Eigen::Vector3d & state) const;
  /// Does the position of the ball allows the robot to kick with the right foot?
  bool canKickRightFoot(const Eigen::Vector3d & state) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

protected:
  /// Minimal distance along x to kick
  double kick_x_min;
  /// Maximal distance along x to kick
  double kick_x_max;
  /// Ball tolerance along y axis for shooting
  double kick_y_tol;
  /// Ball ideal offset in y for each foot: (offset for left_foot, -offset for right_foot)
  /// Warning: For lateral kicks, offset should be a negative value (kick with opposite foot)
  double kick_y_offset;
  /// Direction offset when kicking the ball: symetrical
  /// right_kick: kick_dir = robot_dir + kick_theta_offset
  /// left_kick : kick_dir = robot_dir - kick_theta_offset
  /// For forward kicks, value is expected to be 0
  /// For lateral kicks, value is expected to be pi/2
  double kick_theta_offset;
  /// The maximal angle allowed for kicking
  double kick_theta_tol;
};

}
