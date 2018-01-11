#pragma once

#include "rosban_csa_mdp/core/policy.h"

/// Custom approach build for Kick Controler problems with a single player
class OKSeed : public csa_mdp::Policy
{
public:
  OKSeed();

  Eigen::VectorXd getRawAction(const Eigen::VectorXd & state) override;
  Eigen::VectorXd getRawAction(const Eigen::VectorXd & state,
                               std::default_random_engine * external_engine) const override;

  /// This approach is built to be an fa_tree, therefore, there is no loss in conversion
  virtual std::unique_ptr<rosban_fa::FATree> extractFATree() const override;

  /// Powerful kick toward opponent goal
  Eigen::VectorXd backlaneKick() const;
  /// Side should be -1 (right) or 1 (left)
  Eigen::VectorXd centerKick(int side) const;
  /// Small kick forward
  Eigen::VectorXd placeKick() const;
  /// Side should be -1 (right) or 1 (left)
  Eigen::VectorXd finishKick(int side) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const override;

  /// If X < back_limit:
  double back_limit;
  /// If X > back_limit && abs(Y) > goal_width/2: then the robot center
  double goal_width;
  /// If abs(Y) < goal_width and X > finish_limit:
  /// - then the robot tries to score (try to kick in direction of goal_width/4 * side)
  double finish_limit;

  // id of the autoAimKick
  int autoAimId;
  // id of the smallKick
  int smallKickId;  
};
