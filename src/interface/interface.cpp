#include "interface/interface.h"

// TODO: externalize
static double normalizeAngle(double value)
{
  while (value > M_PI)
  {
    value -= 2 * M_PI;
  }
  while (value < -M_PI)
  {
    value += 2 * M_PI;
  }
  return value;
}

Eigen::VectorXd joints2State(const JointListener::StateMap &states,
                             const rosban_control::ControlConfig &config)
{
  std::vector<std::string> missing_sensors;
  int nb_sensors = config.linear_sensors.size() + config.angular_sensors.size();
  Eigen::VectorXd result(2 * nb_sensors);
  int sensor_idx = 0;
  for (const std::string & sensor : config.linear_sensors)
  {
    try
    {
      JointListener::JointState state = states.at(sensor);
      result(sensor_idx++) = state.pos;
      result(sensor_idx++) = state.vel;
    }
    catch (const std::out_of_range & exc)
    {
      missing_sensors.push_back(sensor);
    }
  }
  for (const std::string & sensor : config.angular_sensors)
  {
    try
    {
      JointListener::JointState state = states.at(sensor);
      state.pos = normalizeAngle(state.pos);
      result(sensor_idx++) = state.pos;
      result(sensor_idx++) = state.vel;
    }
    catch (const std::out_of_range &exc)
    {
      missing_sensors.push_back(sensor);
    }
  }
  if (missing_sensors.size() != 0)
  {
    std::ostringstream oss;
    oss << "Missing sensors: {";
    for (size_t i = 0; i < missing_sensors.size(); i++)
    {
      oss << missing_sensors[i];
      if (i < missing_sensors.size() - 1) oss << ",";
    }
    oss << "}";
    throw std::out_of_range(oss.str());
  }
  return result;
}

