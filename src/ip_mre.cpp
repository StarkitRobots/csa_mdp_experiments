#include "rosban_csa_mdp/solvers/mre.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <ros/ros.h>

#include <fstream>

#include <sys/stat.h>

using csa_mdp::FPF;
using csa_mdp::MRE;

// Control properties
int frequency = 10;//Hz
double max_pos = M_PI;//rad
//// Simple pendulum values
//double max_vel = 3 * M_PI;//rad /s
//double max_torque = 2.5;//N*m
// Double pendulum values
std::vector<double> max_vel = { 3 * M_PI, 6 * M_PI};
std::vector<double> max_torque = { 3.0, 2.5};

// State and action spaces
Eigen::MatrixXd state_limits;
Eigen::MatrixXd action_limits;

bool isTerminal(const Eigen::VectorXd & state)
{
  for (int i = 0; i < state.rows(); i++)
  {
    if (state(i) < state_limits(i,0) || state(i) > state_limits(i,1))
    {
      return true;
    }
  }
  return false;
}

double getReward(const Eigen::VectorXd & state,
                 const Eigen::VectorXd & action,
                 const Eigen::VectorXd & next_state)
{
  (void)state;// Unused
  if (isTerminal(next_state))
  {
    return -100;
  }
  double pos_cost = 0;
  double speed_cost = 0;
  double torque_cost = 0;
  // state costs
  for (int i = 0; i < state.rows(); i++)
  {
    if (i % 2 == 0)
    {
      // Error on the first dof has much more impact to make convergence easier
      double cost = std::fabs(state(i) / max_pos);
      if (i > 0) cost /= 10;
      pos_cost += cost;
    }
    else
    {
      speed_cost += 0;// nothing for now
    }
  }
  for (int i = 0; i < action.rows(); i++)
  {
    //torque_cost += std::pow(action(i) / max_torque[i], 2);
  }
  return - (pos_cost + speed_cost + torque_cost);
}

int main(int argc, char ** argv)
{
  std::string robot = ros::getROSArg(argc, argv, "robot");
  std::string log_path = ros::getROSArg(argc, argv, "log_path");
  std::string dof_count_string = ros::getROSArg(argc, argv, "_dof_count");

  if (robot == "" || log_path == "" || dof_count_string == "")
  {
    std::cerr << "Usage: rosrun .... robot:=<file> log_path:=<path> _dof_count:=<nb_dofs>" << std::endl;
    exit(EXIT_FAILURE);
  }

  int dof_count = std::stoi(dof_count_string);

  // Init ros access
  ros::init(argc, argv, "ip_mre");
  ros::NodeHandle nh;

  // Auto complete path:
  if (log_path[log_path.size() -1] != '/')
  {
    log_path = log_path + "/";
  }

  // Create a directory for details
  std::string details_path = log_path + "details";

  
  struct stat folder_stat;
  // If there is no 'details' folder, create it
  if (stat(details_path.c_str(), &folder_stat) != 0 || !S_ISDIR(folder_stat.st_mode))
  {
    std::string mkdir_cmd = "mkdir " + details_path;
    // If it fails, exit
    if (system(mkdir_cmd.c_str()))
    {
      std::cerr << "Failed to create '" << details_path << "' folder" << std::endl;
      exit(EXIT_FAILURE);
    }
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
 

  // Listing axis
  std::vector<std::string> dofs;
  for (int i = 1; i <= dof_count; i++)
  {
    dofs.push_back("axis" + std::to_string(i));
  }
 
  // INITIALIZATION
  ros::Rate rate(frequency);
  // Communications with controllers
  ControlBridge bridge(nh, dofs, "/" + robot + "/");         // Write command
  JointListener listener(nh, "/" + robot + "/joint_states"); // Read status

  // build limits
  state_limits  = Eigen::MatrixXd(2 * dof_count, 2);
  action_limits = Eigen::MatrixXd(    dof_count, 2);
  for (int i = 0; i < dof_count; i++)
  {
    state_limits(2*i  ,1) = max_pos;
    state_limits(2*i+1,1) = max_vel[i];
    action_limits(i,1) = max_torque[i];
  }
  // symetrical limits
  state_limits.block(0,0,2*dof_count,1) = -state_limits.block(0,1,2*dof_count,1);
  action_limits.block(0,0,dof_count,1) = -action_limits.block(0,1,dof_count,1);

  // EXPLORATION AND MRE PROPERTIES
  // Exploration size
  int nb_trajectories = 200;
  int trajectory_max_length = 200;

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
  fpf_conf.horizon = 30;
  fpf_conf.discount = 0.98;
  fpf_conf.max_action_tiles = 50;
  fpf_conf.q_value_conf.k = 6;
  fpf_conf.q_value_conf.n_min = 2;
  fpf_conf.q_value_conf.nb_trees = 25;
  fpf_conf.q_value_conf.min_var = std::pow(10, -8);
  fpf_conf.q_value_conf.appr_type = regression_forests::ApproximationType::PWC;
  fpf_conf.policy_samples = 1;// This parameter just needs to be higher than 0
  fpf_conf.policy_conf.k = 4;
  fpf_conf.policy_conf.n_min = 10;
  fpf_conf.policy_conf.nb_trees = 25;
  fpf_conf.policy_conf.min_var = std::pow(10, -4);
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
    // Check if all joints are available
    std::vector<std::string> missing_joints;
    for (const std::string & dof : dofs)
    {
      if (joints.count(dof) == 0)
      {
        missing_joints.push_back(dof);
      }
    }
    // If there was no missing joints, break the loop
    if (missing_joints.size() == 0) break;
    // Otherwise print missing joints
    std::cerr << missing_joints.size() << " joints missing, check robot name" << std::endl;
    for (const std::string & dof : missing_joints)
    {
      std::cerr << "\t" << dof << " is missing" << std::endl;
    }
    ros::spinOnce();
    rate.sleep();
  }

  // Printing csv header
  trajectories_output << "time,run,step,";
  // Source state
  for (int dof_id = 1; dof_id <= dof_count; dof_id++)
  {
    trajectories_output << "theta" << dof_id << ","
                        << "omega" << dof_id << ",";
    samples_output << "src_theta" << dof_id << ","
                   << "src_omega" << dof_id << ",";
  }
  // Action
  for (int dof_id = 1; dof_id <= dof_count; dof_id++)
  {
    trajectories_output << "command" << dof_id << ",";
    samples_output << "command" << dof_id << ",";
  }
  // Next state / raw_value
  for (int dof_id = 1; dof_id <= dof_count; dof_id++)
  {
    trajectories_output << "raw_theta" << dof_id;
    if (dof_id != dof_count) trajectories_output << ",";
    samples_output << "dst_theta" << dof_id << ","
                   << "dst_omega" << dof_id << ",";
  }
  samples_output << "reward" << std::endl;
  trajectories_output << std::endl;

  // Until user stops process or total number of trajectories has been reached
  int trajectory_id = 1;
  while (ros::ok() && trajectory_id <= nb_trajectories)
  {
    // Start a new trajectory
    Eigen::VectorXd last_state(2 * dof_count);
    Eigen::VectorXd last_action(dof_count);
    // Updating policies can be quite long, we need to reset rate to have a 'real sleep'
    rate.reset();
    // Until enough step have been taken
    bool failure = false;
    double trajectory_reward = 0;
    // Until enough step have been taken
    for (int step = 0; step <= trajectory_max_length; step++)
    {
      // let ros treat message
      ros::spinOnce();
      // Start by reading current state:
      Eigen::VectorXd state(2 * dof_count);
      Eigen::VectorXd raw_pos(dof_count);
      auto joints = listener.getStatus();
      std::string dof_name;
      try {
        for (int dof_id = 1; dof_id <= dof_count; dof_id++)
        {
          dof_name = "axis" + std::to_string(dof_id);
          JointListener::JointState joint_state = joints.at(dof_name);
          // Keeping raw_pos (for plotting purpose)
          raw_pos(dof_id - 1) = joint_state.pos;
          // Normalizing pos in -pi, pi
          while (joint_state.pos > M_PI)
          {
            joint_state.pos -= 2 * M_PI;
          }
          while (joint_state.pos < -M_PI)
          {
            joint_state.pos += 2 * M_PI;
          }
          // Writing values to state:
          state(2 * (dof_id - 1)     ) = joint_state.pos;
          state(2 * (dof_id - 1) + 1 ) = joint_state.vel;
        }
      }
      catch (const std::out_of_range &exc) {
        std::cerr << "Failed to find " << dof_name << " in the published joints" << std::endl;
        failure = true;
      }
      if (failure) break;
      // Compute, bound and send command
      Eigen::VectorXd action = mre.getAction(state);
      for (int i = 0; i < action.rows(); i++)
      {
        if (action(i) >  max_torque[i]) action(i) =  max_torque[i];
        if (action(i) < -max_torque[i]) action(i) = -max_torque[i];
      }
      // Prepare command
      std::map<std::string, double> targets;
      for (int dof_id = 1; dof_id <= dof_count; dof_id++)
      {
        std::string dof_name = "axis" + std::to_string(dof_id);
        targets[dof_name] = action(dof_id - 1);
      }
      bridge.send(targets);
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
          std::cerr << "failed to add sample to MRE:" << std::endl
                    << "\tlast_state: " << last_state.transpose() << std::endl
                    << "\tisTerminal(last_state): " << isTerminal(last_state) << std::endl;
          throw exc;
        }
        // Writing samples line
        for (int i = 0; i < last_state.rows(); i++)
          samples_output << last_state(i) << ",";
        for (int i = 0; i < action.rows(); i++)
          samples_output << action(i) << ",";
        for (int i = 0; i < state.rows(); i++)
          samples_output << state(i) << ",";
        samples_output << reward << std::endl;
        trajectory_reward += reward;
      }
      // update memory
      last_state = state;
      last_action = action;
      // write trajectory entry
      trajectories_output << ros::Time::now().toSec() << ","
                          << trajectory_id << ","
                          << step << ",";
      for (int i = 0; i < state.rows(); i++)
        trajectories_output << state(i) << ",";
      for (int i = 0; i < action.rows(); i++)
        trajectories_output << action(i) << ",";
      for (int i = 0; i < raw_pos.rows(); i++)
      {
        trajectories_output << raw_pos(i);
        if (i < raw_pos.rows() - 1) trajectories_output << ",";
      }
      trajectories_output << std::endl;
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
    std::map<std::string, double> targets;
    for (int dof_id = 1; dof_id <= dof_count; dof_id++)
    {
      std::string dof_name = "axis" + std::to_string(dof_id);
      targets[dof_name] = 0;
    }
    bridge.send(targets);
    // Update and save policy
    std::cout << "Reward for trajectory " << trajectory_id << ": " << trajectory_reward << std::endl;
    std::cout << "Updating policy " << trajectory_id << std::endl;
    mre.updatePolicy();
    std::cout << "\tTime spent to compute q_value   : " << mre.getQValueTime() << "[s]" << std::endl;
    std::cout << "\tTime spent to compute the policy: " << mre.getPolicyTime() << "[s]" << std::endl;
    std::string prefix = details_path + "/T" + std::to_string(trajectory_id) + "_";
    std::cout << "Saving all with prefix " << prefix << std::endl;
    mre.saveStatus(prefix);
    trajectory_id++;
  }
  trajectories_output.close();
  samples_output.close();
  return EXIT_SUCCESS;
}
