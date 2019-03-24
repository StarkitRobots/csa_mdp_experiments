#include "kick_model/custom_power_kick.h"

#include "rhoban_utils/xml_tools.h"

namespace csa_mdp
{
Eigen::MatrixXd CustomPowerKick::getActionsLimits() const
{
  Eigen::MatrixXd limits(2, 2);
  limits << -M_PI, M_PI, min_kick_power, max_kick_power;
  return { limits };
}
std::vector<std::string> CustomPowerKick::getActionsNames() const
{
  return { "kick_direction", "kick_power" };
}

double CustomPowerKick::getWishedDir(double ball_x, double ball_y, const Eigen::VectorXd& kick_parameters) const
{
  // Direction of kick is not dependent on ball position
  (void)ball_x;
  (void)ball_y;

  return kick_parameters(0);
}

void CustomPowerKick::applyKick(double ball_start_x, double ball_start_y, const Eigen::VectorXd& kick_parameters,
                                std::default_random_engine* engine, double* final_ball_x, double* final_ball_y,
                                double* reward) const
{
  if (kick_parameters.rows() != 2)
  {
    std::ostringstream oss;
    oss << "CustomPowerKick::applyKick: invalid size for kick_parameters: " << kick_parameters.rows()
        << " rows, expecting 1";
    throw std::logic_error(oss.str());
  }
  double kick_power = kick_parameters(1);
  if (kick_power < min_kick_power || kick_power > max_kick_power)
  {
    std::ostringstream oss;
    oss << "CustomPowerKick::applyKick: kick_power is not in bounds: " << kick_power << " not in [" << min_kick_power
        << "," << max_kick_power << "]";
    throw std::logic_error(oss.str());
  }

  applyKick(ball_start_x, ball_start_y, kick_power, kick_parameters(0), engine, final_ball_x, final_ball_y, reward);
}

Json::Value CustomPowerKick::toJson() const
{
  KickModel::toJson(out);
  v["min_kick_power"] =  min_kick_power, out);
  v["max_kick_power"] =  max_kick_power, out);
}

void CustomPowerKick::fromJson(const Json::Value& v, const std::string& dir_name)
{
  KickModel::fromJson(node);
  min_kick_power = rhoban_utils::read<double>(v, "min_kick_power");
  max_kick_power = rhoban_utils::read<double>(v, "max_kick_power");
}

std::string CustomPowerKick::getClassName() const
{
  return "CustomPowerKick";
}

}  // namespace csa_mdp
