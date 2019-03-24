#pragma once

#include "kick_model/kick_decision_model.h"

namespace csa_mdp
{
/// Final kick chooses only the direction of the kick according to the ball
/// position.
/// The direction is chosen in order to reach theoretically the goal line at a
/// chosen y value (action)
class FinalKick : public KickDecisionModel
{
public:
  FinalKick();

  void updateActionLimits();

  /// ball_pos: position in the field referential [m]
  Eigen::VectorXd computeKickParameters(const Eigen::VectorXd& ball_pos, const Eigen::VectorXd& actions) const override;

  /// ball_pos: position in the field referential [m]
  double computeKickDirection(const Eigen::VectorXd& ball_pos, const Eigen::VectorXd& actions) const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const override;

private:
  /// The position of the opponent goal in x (expecting field_length / 2)
  double goal_x;

  /// The maximal value for target_y (expecting goal_width / 2)
  double max_y;
};

}  // namespace csa_mdp
