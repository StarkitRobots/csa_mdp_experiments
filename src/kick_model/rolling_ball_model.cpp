#include "kick_model/rolling_ball_model.h"

namespace csa_mdp
{
RollingBallModel::RollingBallModel() : decay_rate(0.9)
{
}

Eigen::Vector4d RollingBallModel::getNextState(const Eigen::Vector4d& ball_state, double dt) const
{
  // Warning: two successives updateBall(dt) is not the same as updateBall(2*dt)
  const Eigen::Vector2d& ball_pos = ball_state.segment(0, 2);
  const Eigen::Vector2d& ball_speed = ball_state.segment(2, 2);
  Eigen::Vector2d next_speed = ball_speed * (1 - (1 - decay_rate) * dt);
  Eigen::Vector2d avg_speed = (ball_speed + next_speed) / 2;
  Eigen::Vector4d result;
  result.segment(0, 2) = avg_speed * dt + ball_pos;
  result.segment(2, 2) = next_speed;
  return result;
}

Json::Value RollingBallModel::toJson() const
{
  Json::Value v;
  v["decay_rate"] = decay_rate;
  return v;
}

void RollingBallModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryRead(v, "decay_rate", &decay_rate);
}

std::string RollingBallModel::getClassName() const
{
  return "RollingBallModel";
}

}  // namespace csa_mdp
