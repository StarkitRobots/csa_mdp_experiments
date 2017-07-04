#include "kick_model/classic_kick.h"

using namespace rosban_utils;

namespace csa_mdp
{

ClassicKick::ClassicKick()
  : kick_power(1.0), right_kick_dir(0.0),
    rel_dist_stddev(0.05), dir_stddev(10 * M_PI /180)
{
  // Setting limits, names and default value for parameters
  parameters_limits = Eigen::MatrixXd(1,2);
  parameters_limits << -M_PI, M_PI;
  parameters_names.push_back("kick_theta_tol");
  default_parameters = Eigen::VectorXd(1);
  default_parameters(0) = 10 * M_PI / 180;
}

Eigen::Vector2d ClassicKick::applyKick(const Eigen::Vector2d & ball_pos,
                                       double kick_dir,
                                       const Eigen::VectorXd & kick_parameters,
                                       std::default_random_engine * engine) const
{
  double kick_real_dist = kick_power;
  double kick_real_dir = kick_dir;
  // If engine has been provided, apply noise
  if (engine != nullptr) {
    // Uniform noise related to kick tolerance
    std::uniform_real_distribution<double> player_dir_dist(-kick_parameters(0),
                                                           kick_parameters(0));
    // Kick related noise
    std::normal_distribution<double> kick_dir_dist(0.0, dir_stddev);
    // Distance relative noise
    std::normal_distribution<double> kick_factor_dist(1.0, rel_dist_stddev);
    // Apply noise
    kick_real_dist *= kick_factor_dist(*engine);
    kick_real_dir  += kick_dir_dist(*engine);
    kick_real_dir  += player_dir_dist(*engine);
  }
  Eigen::Vector2d final_pos = ball_pos;
  final_pos(0) += kick_real_dist * cos(kick_real_dir);
  final_pos(1) += kick_real_dist * sin(kick_real_dir);
  return final_pos;
}

void ClassicKick::to_xml(std::ostream & out) const
{
  KickModel::to_xml(out);
  xml_tools::write<double>("kick_power"     , kick_power     , out);
  xml_tools::write<double>("right_kick_dir" , right_kick_dir , out);
  xml_tools::write<double>("rel_dist_stddev", rel_dist_stddev, out);
  xml_tools::write<double>("dir_stddev"     , dir_stddev     , out);
}

void ClassicKick::from_xml(TiXmlNode * node)
{
  KickModel::from_xml(node);
  xml_tools::try_read<double>(node, "kick_power"     , kick_power     );
  xml_tools::try_read<double>(node, "right_kick_dir" , right_kick_dir );
  xml_tools::try_read<double>(node, "rel_dist_stddev", rel_dist_stddev);
  xml_tools::try_read<double>(node, "dir_stddev"     , dir_stddev     );
}

std::string ClassicKick::class_name() const
{
  return "ClassicKick";
}


}
