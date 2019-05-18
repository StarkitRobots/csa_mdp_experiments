#include "policies/ok_seed.h"

#include "starkit_fa/constant_approximator.h"
#include "starkit_fa/orthogonal_split.h"

using namespace starkit_fa;

OKSeed::OKSeed() : back_limit(1), goal_width(2.6), finish_limit(2.5), autoAimId(0), smallKickId(1)
{
}

Eigen::VectorXd OKSeed::getRawAction(const Eigen::VectorXd& state)
{
  return getRawAction(state, NULL);
}

Eigen::VectorXd OKSeed::getRawAction(const Eigen::VectorXd& state, std::default_random_engine* external_engine) const
{
  (void)external_engine;
  // Extracting ball position
  double ball_x = state(0);
  double ball_y = state(1);
  // Backlane situation
  if (ball_x < back_limit)
  {
    return backlaneKick();
  }
  // Center situation
  if (std::fabs(ball_y) > goal_width / 2)
  {
    return centerKick(ball_y > 0 ? -1 : 1);
  }
  // Place situation (try to get the ball closer slowly)
  if (ball_x < finish_limit)
  {
    return placeKick();
  }
  // Finish goal!
  return finishKick(ball_y > 0 ? 1 : -1);
}

std::unique_ptr<starkit_fa::FATree> OKSeed::extractFATree() const
{
  // Build approximators
  std::unique_ptr<FunctionApproximator> backlane_node(new ConstantApproximator(backlaneKick()));
  std::unique_ptr<FunctionApproximator> center_left_node(new ConstantApproximator(centerKick(1)));
  std::unique_ptr<FunctionApproximator> center_right_node(new ConstantApproximator(centerKick(-1)));
  std::unique_ptr<FunctionApproximator> place_node(new ConstantApproximator(placeKick()));
  std::unique_ptr<FunctionApproximator> finish_left_node(new ConstantApproximator(finishKick(1)));
  std::unique_ptr<FunctionApproximator> finish_right_node(new ConstantApproximator(finishKick(-1)));
  // Build splits
  std::unique_ptr<Split> back_split(new OrthogonalSplit(0, back_limit));
  std::unique_ptr<Split> finish_split(new OrthogonalSplit(0, finish_limit));
  std::unique_ptr<Split> left_post_split(new OrthogonalSplit(1, goal_width / 2));
  std::unique_ptr<Split> right_post_split(new OrthogonalSplit(1, -goal_width / 2));
  std::unique_ptr<Split> central_split(new OrthogonalSplit(1, 0));
  // Build tree
  std::vector<std::unique_ptr<FunctionApproximator>> approximators;
  std::unique_ptr<FATree> tmp;
  // 1. In finish zone, avoid goalie
  approximators.push_back(std::move(finish_right_node));
  approximators.push_back(std::move(finish_left_node));
  tmp.reset(new FATree(std::move(central_split), approximators));
  // 2. Separate finish zone from place zone
  approximators.push_back(std::move(place_node));
  approximators.push_back(std::move(tmp));
  tmp.reset(new FATree(std::move(finish_split), approximators));
  // 3. Centering
  // 3a:
  approximators.push_back(std::move(center_left_node));
  approximators.push_back(std::move(tmp));
  tmp.reset(new FATree(std::move(right_post_split), approximators));
  // 3b:
  approximators.push_back(std::move(tmp));
  approximators.push_back(std::move(center_right_node));
  tmp.reset(new FATree(std::move(left_post_split), approximators));
  // 4. Backlane
  approximators.push_back(std::move(backlane_node));
  approximators.push_back(std::move(tmp));
  tmp.reset(new FATree(std::move(back_split), approximators));
  // Tree has been created
  return std::move(tmp);
}

Eigen::VectorXd OKSeed::backlaneKick() const
{
  return Eigen::Vector2d(autoAimId, 0);
}
Eigen::VectorXd OKSeed::centerKick(int side) const
{
  return Eigen::Vector2d(smallKickId, 120 * side * M_PI / 180);
}
Eigen::VectorXd OKSeed::placeKick() const
{
  return Eigen::Vector2d(smallKickId, 0);
}

Eigen::VectorXd OKSeed::finishKick(int side) const
{
  return Eigen::Vector2d(autoAimId, goal_width / 4 * side);
}

Json::Value OKSeed::toJson() const
{
  Json::Value v;
  // TODO: write Ids
  v["back_limit"] = back_limit;
  v["goal_width"] = goal_width;
  v["finish_limit"] = finish_limit;
  return v;
}

void OKSeed::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  // TODO: read Ids
  starkit_utils::tryRead(v, "back_limit", &back_limit);
  starkit_utils::tryRead(v, "goal_width", &goal_width);
  starkit_utils::tryRead(v, "finish_limit", &finish_limit);
}

std::string OKSeed::getClassName() const
{
  return "OKSeed";
}
