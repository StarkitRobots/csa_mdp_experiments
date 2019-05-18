#include "kick_model/full_power_kick.h"

#include "starkit_utils/xml_tools.h"

namespace csa_mdp
{
Eigen::MatrixXd FullPowerKick::getActionsLimits() const
{
  Eigen::MatrixXd limits(1, 2);
  limits << -M_PI, M_PI;
  return limits;
}
std::vector<std::string> FullPowerKick::getActionsNames() const
{
  return { "kick_direction" };
}

double FullPowerKick::getWishedDir(double ball_x, double ball_y, const Eigen::VectorXd& kick_parameters) const
{
  // Direction of kick is not dependent on ball position
  (void)ball_x;
  (void)ball_y;

  return kick_parameters(0);
}

void FullPowerKick::applyKick(double ball_start_x, double ball_start_y, const Eigen::VectorXd& kick_parameters,
                              std::default_random_engine* engine, double* final_ball_x, double* final_ball_y,
                              double* reward) const
{
  if (kick_parameters.rows() != 1)
  {
    std::ostringstream oss;
    oss << "FullPowerKick::applyKick: invalid size for kick_parameters: " << kick_parameters.rows()
        << " rows, expecting 1";
    throw std::logic_error(oss.str());
  }
  applyKick(ball_start_x, ball_start_y, kick_power, kick_parameters(0), engine, final_ball_x, final_ball_y, reward);
}

Json::Value FullPowerKick::toJson() const
{
  KickModel::toJson(out);
  v["kick_power"] =  kick_power, out);
}

void FullPowerKick::fromJson(const Json::Value& v, const std::string& dir_name)
{
  KickModel::fromJson(node);
  kick_power = starkit_utils::read<double>(v, "kick_power");
}

std::string FullPowerKick::getClassName() const
{
  return "FullPowerKick";
}

}  // namespace csa_mdp
