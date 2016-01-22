#include "rosban_csa_mdp/solvers/mre.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <ros/ros.h>

#include <fstream>

using csa_mdp::FPF;
using csa_mdp::MRE;

// Control properties
int frequency = 10;//Hz
double max_pos = M_PI;//rad
double max_vel = 3 * M_PI;//rad /s
double max_torque = 2.5;//N*m
std::string dof = "axis";

bool isTerminal(const Eigen::VectorXd & state)
{
  bool pos_bad = state(0) < -max_pos || state(0) > max_pos;
  bool vel_bad = state(1) < -max_vel || state(1) > max_vel;
  return pos_bad || vel_bad;
}

double getReward(const Eigen::VectorXd & state,
                 const Eigen::VectorXd & action,
                 const Eigen::VectorXd & next_state)
{
  (void)state;// Unused
  if (isTerminal(next_state))
  {
    return -50;
  }
  // Normalizing position if necessary
  double position = next_state(0);
  if (position > max_pos || position < -max_pos)
  {
    throw std::runtime_error("Invalid position found, expecting value in [-pi,pi]");
  }
  double pos_cost   = std::pow(position  / max_pos   , 2);
  double force_cost = std::pow(action(0) / max_torque, 2);
  return - (pos_cost + force_cost);
  
}

int main(int argc, char ** argv)
{
  std::string robot = ros::getROSArg(argc, argv, "robot");
  std::string log_path = ros::getROSArg(argc, argv, "log_path");

  if (robot == "" || log_path == "")
  {
    std::cerr << "Usage: rosrun .... robot:=<file> log_path:=<path>" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Init ros access
  ros::init(argc, argv, "ip_mre");
  ros::NodeHandle nh;

  // Auto complete path:
  if (log_path[log_path.size() -1] != '/')
  {
    log_path = log_path + "/";
  }

  // Open trajectory.csv
  std::ofstream trajectories_output;
  trajectories_output.open(log_path + "trajectories.csv");
  if (!trajectories_output.is_open())
  {
    std::cerr << "Failed to open file: '" << log_path << "trajectories.csv'" << std::endl;
  }
  std::ofstream samples_output;
  samples_output.open(log_path + "samples.csv");
  if (!samples_output.is_open())
  {
    std::cerr << "Failed to open file: '" << log_path << "samples.csv'" << std::endl;
  }
  
  // INITIALIZATION
  ros::Rate rate(frequency);
  // Communications with controllers
  ControlBridge bridge(nh, {dof}, "/" + robot + "/");        // Write command
  JointListener listener(nh, "/" + robot + "/joint_states"); // Read status

  // build limits
  Eigen::MatrixXd state_limits(2,2), action_limits(1,2);
  state_limits << -max_pos, max_pos, -max_vel, max_vel;
  action_limits << -max_torque, max_torque;
  

  // EXPLORATION AND MRE PROPERTIES
  // Exploration size
  int nb_trajectories = 500;
  int trajectory_max_length = 100;

  // MRE properties
  int max_points = 10;
  double reward_max = 0;
  int plan_period = -1;
  int nb_knownness_trees = 25;
  MRE::KnownnessTree::Type knownness_tree_type = MRE::KnownnessTree::Type::Random;

  // FPF properties
  FPF::Config fpf_conf;
  fpf_conf.setStateLimits(state_limits);
  fpf_conf.setActionLimits(action_limits);
  fpf_conf.horizon = 40;
  fpf_conf.discount = 0.98;
  fpf_conf.max_action_tiles = 50;
  fpf_conf.q_value_conf.k = 3;
  fpf_conf.q_value_conf.n_min = 5;
  fpf_conf.q_value_conf.nb_trees = 25;
  fpf_conf.q_value_conf.min_var = std::pow(10, -6);
  fpf_conf.q_value_conf.appr_type = regression_forests::ApproximationType::PWC;
  fpf_conf.policy_samples = 1000;
  fpf_conf.policy_conf.k = 2;
  fpf_conf.policy_conf.n_min = 10;
  fpf_conf.policy_conf.nb_trees = 25;
  fpf_conf.policy_conf.min_var = std::pow(10, -6);
  fpf_conf.policy_conf.appr_type = regression_forests::ApproximationType::PWL;

  MRE mre(state_limits,
          action_limits,
          max_points,
          reward_max,
          plan_period,
          nb_knownness_trees,
          knownness_tree_type,
          fpf_conf,
          [](const Eigen::VectorXd &state) {return isTerminal(state);});

  // Wait until connection has been established properly
  while(ros::ok())
  {
    auto joints = listener.getStatus();
    // Once dof is found, keep on
    if (joints.count(dof) != 0) break;
    std::cerr << "'" << dof << "' not found in joints, check robot name" << std::endl;
    std::cerr << "nb joints: " << joints.size() << std::endl;
    ros::spinOnce();
    rate.sleep();
  }

  // Printing csv header
  trajectories_output << "time,run,step,pos,vel,cmd,raw_pos" << std::endl;
  samples_output << "src_pos,src_vel,torque,dst_pos,dest_vel,reward" << std::endl;

  // Until user stops process or total number of trajectories has been reached
  int trajectory_id = 1;
  while (ros::ok() && trajectory_id <= nb_trajectories)
  {
    // Start a new trajectory
    Eigen::VectorXd last_state(2);
    Eigen::VectorXd last_action(1);
    // Until enough step have been taken
    bool failure = false;
    rate.reset();// Updating policies can be quite long
    double trajectory_reward = 0;
    for (int step = 0; step <= trajectory_max_length; step++)
    {
      // let ros treat message
      ros::spinOnce();
      // Start by reading values
      auto joints = listener.getStatus();
      JointListener::JointState joint_state;
      double raw_pos;// For plotting purpose
      try
      {
        joint_state = joints.at(dof);
        raw_pos = joint_state.pos;
        while (joint_state.pos > M_PI)  joint_state.pos -= 2 * M_PI;
        while (joint_state.pos < -M_PI) joint_state.pos += 2 * M_PI;
      }
      catch (const std::out_of_range &exc)
      {
        failure = true;
        break;
      }
      Eigen::VectorXd state(2);
      state(0) = joint_state.pos;
      state(1) = joint_state.vel;
      // compute, bound and send command
      Eigen::VectorXd action = mre.getAction(state);
      if (action(0) >  max_torque) action(0) =  max_torque;
      if (action(0) < -max_torque) action(0) = -max_torque;
      bridge.send({{dof, action(0)}});
      // if there is a previous state, build and add sample
      if (step != 0)
      {
        double reward = getReward(last_state, last_action, state);
        csa_mdp::Sample new_sample(last_state,
                                   last_action,
                                   state,
                                   reward);
        try{
          mre.feed(new_sample);
        }
        catch(const std::runtime_error &exc)
        {
          bool pos_bad = last_state(0) < -max_pos || last_state(0) > max_pos;
          bool vel_bad = last_state(1) < -max_vel || last_state(1) > max_vel;
          std::cerr << "failed to add sample to MRE:" << std::endl
                    << "\tlast_state: " << last_state.transpose() << std::endl
                    << "\tpos_bad: " << pos_bad << std::endl
                    << "\tvel_bad: " << vel_bad << std::endl
                    << "\tisTerminal(last_state): " << isTerminal(last_state) << std::endl;
          throw exc;
        }
        samples_output << last_state(0)  << ","
                       << last_state(1)  << ","
                       << last_action(0) << ","
                       << state(0)       << ","
                       << state(1)       << ","
                       << reward         << std::endl;
        trajectory_reward += reward;
      }
      // update memory
      last_state = state;
      last_action = action;
      // write entry
      trajectories_output << ros::Time::now().toSec() << ","
                          << trajectory_id << ","
                          << step << ","
                          << joint_state.pos << ","
                          << joint_state.vel << ","
                          << action(0) << ","
                          << raw_pos << std::endl;
      // If state is terminal, end trajectory
      if (isTerminal(state)) break;
      // sleep for a given time
      rate.sleep();
    }
    // Handle case where connection has been broken
    if (failure)
    {
      std::cerr << "Missing information in listener, aborting learning experiment" << std::endl;
      break;
    }
    // Apply a 0 torque during policy computation
    bridge.send({{dof, 0}});
    // Update and save policy
    std::cout << "Reward for trajectory " << trajectory_id << ": " << trajectory_reward << std::endl;
    std::cout << "Updating policy " << trajectory_id << std::endl;
    mre.updatePolicy();
    std::string prefix = log_path + "T" + std::to_string(trajectory_id) + "_";
    std::cout << "Saving all with prefix " << prefix << std::endl;
    mre.saveStatus(prefix);
    trajectory_id++;
  }
  trajectories_output.close();
  samples_output.close();
  return EXIT_SUCCESS;
}
