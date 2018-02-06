#include "joint_listener.hpp"
#include "control_config.h"

#include <Eigen/Core>

#include <stdexcept>

/// Throw an out_of_range exception if states are missing
Eigen::VectorXd joints2State(const JointListener::StateMap &states,
                             const rhoban_control::ControlConfig &config);
