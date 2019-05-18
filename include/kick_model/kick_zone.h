#pragma once

#include "starkit_utils/serialization/json_serializable.h"

#include <Eigen/Core>

#include <random>

namespace csa_mdp
{
/// Provided states follow this form (inside player_referential)
/// 0: ball_x   [m]
/// 1: ball_y   [m]
/// 2: kick_wished_dir [rad]
class KickZone : public starkit_utils::JsonSerializable
{
public:
  KickZone();

  /// Return the desired state for kicks with the given foot
  /// (wishedX [m], wishedY[m], wishedOffset [rad])
  Eigen::Vector3d getWishedPos(bool right_foot) const;

  /// Return the desired position for the robot (in field referential) to kick a
  /// ball at ball_pos (fieldX[m],fieldY[m]) with the given foot toward provided
  /// direction
  Eigen::Vector3d getWishedPosInField(const Eigen::Vector2d& ball_pos, double kick_wished_dir, bool right_foot) const;

  /// Return the available margin along x-axis from the center [m]
  double getXRange() const;
  /// Return the available margin along y-axis from the center [m]
  double getYRange() const;
  /// Return the theta tolerance [rad]
  double getThetaTol() const;

  /// Can the robot shoot with any of the foot?
  bool isKickable(const Eigen::Vector3d& state) const;

  /// Can the robot shoot with any of the foot?
  /// ball_pos is in field referential [m]
  /// player_state is in field referential [m][m][rad]
  /// kick_dir is in field_referential [rad]
  bool isKickable(const Eigen::Vector2d& ball_pos, const Eigen::Vector3d& player_state, double kick_dir) const;

  /// Can the robot kick from given state with specified foot
  bool canKick(bool right_foot, const Eigen::Vector3d& state) const;

  /// Does the position of the ball allows the robot to kick with the left foot?
  bool canKickLeftFoot(const Eigen::Vector3d& state) const;
  /// Does the position of the ball allows the robot to kick with the right foot?
  bool canKickRightFoot(const Eigen::Vector3d& state) const;

  /// ball_pos is in field referential [m]
  /// player_state is in field referential [m][m][rad]
  /// kick_dir is in field_referential [rad]
  Eigen::Vector3d convertWorldStateToKickState(const Eigen::Vector2d& ball_pos, const Eigen::Vector3d& player_state,
                                               double kick_dir) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const override;

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
  /// For lateral kicks, value is expected to be around pi/2
  double kick_theta_offset;
  /// The maximal angle allowed for kicking
  double kick_theta_tol;
};

}  // namespace csa_mdp
