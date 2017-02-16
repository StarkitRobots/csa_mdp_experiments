#include "kick_model/custom_power_kick.h"

#include "rosban_utils/xml_tools.h"

namespace csa_mdp
{

Eigen::MatrixXd CustomPowerKick::getActionsLimits() const
{
  Eigen::MatrixXd limits(2,2);
  limits << -M_PI, M_PI, min_kick_power, max_kick_power;
  return {limits};
}
std::vector<std::string> CustomPowerKick::getActionsNames() const
{
  return {"kick_direction","kick_power"};
}

void CustomPowerKick::applyKick(double ball_start_x, double ball_start_y,
                                const Eigen::VectorXd & kick_parameters,
                                std::default_random_engine * engine,
                                double * final_ball_x, double * final_ball_y,
                                double * reward) const
{
  if (kick_parameters.rows() != 2) {
    std::ostringstream oss;
    oss << "CustomPowerKick::applyKick: invalid size for kick_parameters: "
        << kick_parameters.rows() << " rows, expecting 1";
    throw std::logic_error(oss.str());
  }
  double kick_power = kick_parameters(1);
  if (kick_power < min_kick_power || kick_power > max_kick_power) {
    std::ostringstream oss;
    oss << "CustomPowerKick::applyKick: kick_power is not in bounds: "
        << kick_power << " not in [" << min_kick_power << ","
        << max_kick_power << "]";
    throw std::logic_error(oss.str());
  }

  applyKick(ball_start_x, ball_start_y,
            kick_power, kick_parameters(0),
            engine, final_ball_x, final_ball_y, reward);
}

void CustomPowerKick::to_xml(std::ostream & out) const
{
  KickModel::to_xml(out);
  rosban_utils::xml_tools::write<double>("min_kick_power", min_kick_power, out);
  rosban_utils::xml_tools::write<double>("max_kick_power", max_kick_power, out);
}

void CustomPowerKick::from_xml(TiXmlNode * node)
{
  KickModel::from_xml(node);
  min_kick_power = rosban_utils::xml_tools::read<double>(node, "min_kick_power");
  max_kick_power = rosban_utils::xml_tools::read<double>(node, "max_kick_power");
}

std::string CustomPowerKick::class_name() const
{
  return "CustomPowerKick";
}

}
