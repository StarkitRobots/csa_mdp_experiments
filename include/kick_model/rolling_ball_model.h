#pragma once

#include "starkit_utils/serialization/json_serializable.h"

namespace csa_mdp
{
/// Simple model which ought to be updated properly
/// Ball state: [x,y,vx,vy]
class RollingBallModel : public starkit_utils::JsonSerializable
{
public:
  RollingBallModel();

  Eigen::Vector4d getNextState(const Eigen::Vector4d& ball_state, double dt) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  std::string getClassName() const override;

private:
  double decay_rate;
};

}  // namespace csa_mdp
