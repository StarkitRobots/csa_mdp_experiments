#include "kick_model/kick_model.h"

#include "rosban_utils/xml_tools.h"

namespace csa_mdp
{

KickModel::KickModel()
  : kick_pow_rel_noise(-1),
    kick_direction_noise(-1),
    kick_pow_rel_stddev(-1),
    kick_direction_stddev(-1),
    kick_reward(-10)
{
}

void KickModel::applyKick(double ball_start_x, double ball_start_y,
                          double kick_power, double kick_theta,
                          std::default_random_engine * engine,
                          double * final_ball_x, double * final_ball_y,
                          double * reward) const
{
  double kick_rel_factor = 1.0;
  double kick_dir_noise_sampled = 0.0;
  // Sampling direction noise
  if (kick_direction_noise > 0) {
    std::uniform_real_distribution<double> theta_noise(-kick_direction_noise,
                                                       kick_direction_noise);
    kick_dir_noise_sampled = theta_noise(*engine);
  }
  if (kick_direction_stddev > 0) {
    std::normal_distribution<double> theta_noise(0, kick_direction_stddev);
    kick_dir_noise_sampled = theta_noise(*engine);
  }
  // Sampling power noise
  if (kick_pow_rel_noise > 0) {
    std::uniform_real_distribution<double> dist_noise(-kick_pow_rel_noise,
                                                      kick_pow_rel_noise);
    kick_rel_factor += dist_noise(*engine);
  }
  if (kick_pow_rel_stddev > 0) {
    std::normal_distribution<double> dist_noise(0, kick_pow_rel_stddev);
    kick_rel_factor += dist_noise(*engine);
  }  

  // Sampling real parameters
  double ball_real_dist  = kick_power * kick_rel_factor;
  double ball_real_angle = kick_theta + kick_dir_noise_sampled;

  // Estimating ball final position
  double cos_kick = cos(ball_real_angle);
  double sin_kick = sin(ball_real_angle);
  *final_ball_x = ball_start_x + cos_kick * ball_real_dist;
  *final_ball_y = ball_start_y + sin_kick * ball_real_dist;
  *reward = kick_reward;
}

void KickModel::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<double>("kick_pow_rel_noise"   , kick_pow_rel_noise   , out);
  rosban_utils::xml_tools::write<double>("kick_direction_noise" , kick_direction_noise , out);
  rosban_utils::xml_tools::write<double>("kick_pow_rel_stddev"  , kick_pow_rel_stddev  , out);
  rosban_utils::xml_tools::write<double>("kick_direction_stddev", kick_direction_stddev, out);
  rosban_utils::xml_tools::write<double>("kick_reward"          , kick_reward          , out);
}

void KickModel::from_xml(TiXmlNode * node)
{
  rosban_utils::xml_tools::try_read<double>(node, "kick_pow_rel_noise"   , kick_pow_rel_noise   );
  rosban_utils::xml_tools::try_read<double>(node, "kick_direction_noise" , kick_direction_noise );
  rosban_utils::xml_tools::try_read<double>(node, "kick_pow_rel_stddev"  , kick_pow_rel_stddev  );
  rosban_utils::xml_tools::try_read<double>(node, "kick_direction_stddev", kick_direction_stddev);
  rosban_utils::xml_tools::try_read<double>(node, "kick_reward"          , kick_reward          );  
}

}
