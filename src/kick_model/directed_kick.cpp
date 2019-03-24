#include "kick_model/directed_kick.h"

namespace csa_mdp
{
DirectedKick::DirectedKick()
{
  action_names.push_back("target_dir");
  action_limits = Eigen::MatrixXd(1, 2);
  action_limits << -M_PI, M_PI;
}

Eigen::VectorXd DirectedKick::computeKickParameters(const Eigen::VectorXd& unused, const Eigen::VectorXd& actions) const
{
  (void)unused;
  (void)actions;
  return Eigen::VectorXd();
}

double DirectedKick::computeKickDirection(const Eigen::VectorXd& unused, const Eigen::VectorXd& actions) const
{
  (void)unused;
  if (actions.rows() != 1)
  {
    std::ostringstream oss;
    oss << "DirectedKick::computeKickDirection: actions has invalid dimension: "
        << "(" << actions.rows() << " while expecting " << 1 << ")" << std::endl;
    throw std::logic_error(oss.str());
  }
  return actions(0);
}
Json::Value DirectedKick::toJson() const
{
  return Json::Value();
}

void DirectedKick::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)v;
  (void)dir_name;
}

std::string DirectedKick::getClassName() const
{
  return "DirectedKick";
}

}  // namespace csa_mdp
