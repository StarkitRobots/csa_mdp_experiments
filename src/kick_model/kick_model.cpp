#include "kick_model/kick_model.h"

#include "rosban_utils/xml_tools.h"

namespace csa_mdp
{

void KickModel::applyKick(double ball_start_x, double ball_start_y,
                          double kick_power, double kick_theta,
                          std::default_random_engine * engine,
                          double * final_ball_x, double * final_ball_y,
                          double * reward) const
{
  // Distribution initialization
  std::uniform_real_distribution<double> dist_noise(1.0 - kick_pow_rel_noise,
                                                    1.0 + kick_pow_rel_noise);
  std::uniform_real_distribution<double> theta_noise(-kick_direction_noise,
                                                     kick_direction_noise);
  // Sampling real parameters
  double ball_real_dist  = kick_power * dist_noise(*engine);
  double ball_real_angle = kick_theta + theta_noise(*engine);

  // Estimating ball final position
  double cos_kick = cos(ball_real_angle);
  double sin_kick = sin(ball_real_angle);
  *final_ball_x = ball_start_x + cos_kick * ball_real_dist;
  *final_ball_y = ball_start_y + sin_kick * ball_real_dist;
  *reward = kick_reward;
}

void KickModel::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<double>("kick_pow_rel_noise"  , kick_pow_rel_noise  , out);
  rosban_utils::xml_tools::write<double>("kick_direction_noise", kick_direction_noise, out);
  rosban_utils::xml_tools::write<double>("kick_reward"         , kick_reward         , out);
}

void KickModel::from_xml(TiXmlNode * node)
{
  kick_pow_rel_noise   = rosban_utils::xml_tools::read<double>(node, "kick_pow_rel_noise"  );
  kick_direction_noise = rosban_utils::xml_tools::read<double>(node, "kick_direction_noise");
  kick_reward          = rosban_utils::xml_tools::read<double>(node, "kick_reward"         );  
}

}
