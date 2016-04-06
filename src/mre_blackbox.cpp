#include "problems/problem_factory.h"

#include "interface/interface.h"

#include <rosban_regression_forests/tools/random.h>
#include <rosban_csa_mdp/solvers/mre.h>

#include <ros/ros.h>

#include <chrono>
#include <fstream>

#include <sys/stat.h>

using csa_mdp::MRE;

void write_log(std::ostream &out,
               int run, int step,
               const Eigen::VectorXd &state,
               const Eigen::VectorXd &action)
{
  out << run << "," << step << ",";
  for (int i = 0; i < state.rows(); i++)
  {
    out << state(i) << ",";
  }
  for (int i = 0; i < action.rows(); i++)
  {
    out << action(i);
    if (i < action.rows() - 1) out << ",";
  }
  out << std::endl;
}

class Config : public rosban_utils::Serializable
{
public:
  Config()
    {
    }

  std::string class_name() const override
    {
      return "Config";
    }

  void to_xml(std::ostream &out) const override
    {
      rosban_utils::xml_tools::write<int>("nb_runs", nb_runs, out);
      rosban_utils::xml_tools::write<int>("nb_steps", nb_steps, out);
      rosban_utils::xml_tools::write<std::string>("problem", problem, out);
      mre_config.write("mre", out);
    }

  void from_xml(TiXmlNode *node) override
    {
      nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
      nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
      problem  = rosban_utils::xml_tools::read<std::string>(node, "problem");
      mre_config.read(node, "mre");
    }

  int nb_runs;
  int nb_steps;
  std::string problem;
  csa_mdp::MRE::Config mre_config;
};


int main(int argc, char ** argv)
{
  // Parsing ros arguments
  std::string config_path = ros::getROSArg(argc, argv, "config_path");

  if (config_path == "")
  {
    std::cerr << "Usage: rosrun .... config_path:=<path>" << std::endl;
    std::cerr << "\tNote: <path> folder should contain a file named Config.xml" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Going to the specified path
  if (chdir(config_path.c_str()))
  {
    std::cerr << "Failed to set '" << config_path << "' as working directory." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create a directory for details if required
  std::string details_path("details");  
  struct stat folder_stat;
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

  // Loading config
  Config config;
  config.load_file();

  // Building problem
  BlackBoxProblem * problem = ProblemFactory().buildBlackBox(config.problem);

  // Applying problem limits
  config.mre_config.mrefpf_conf.setStateLimits(problem->getStateLimits());
  config.mre_config.mrefpf_conf.setActionLimits(problem->getActionLimits());

  // Save an exhaustive version of the configuration used
  config.save_file("UsedConfig.xml");

  // Creating mre instance
  MRE mre(config.mre_config,
          [problem](const Eigen::VectorXd &state)
          {
            return problem->isTerminal(state);
          });

  // Logging trajectories
  std::ofstream logs;
  logs.open("logs.csv");
  // csv header
  logs << "run,step,";
  // State information
  int x_dim = config.mre_config.mrefpf_conf.getStateLimits().rows();
  for (int i = 0; i < x_dim; i++)
  {
    logs << "state_" << i << ",";
  }
  // Commands
  int u_dim = config.mre_config.mrefpf_conf.getActionLimits().rows();
  for (int i = 0; i < u_dim; i++)
  {
    logs << "action_" << i;
    if (i < u_dim - 1) logs << ",";
  }
  logs << std::endl;

  // Logging time consumption
  std::ofstream time_logs;
  time_logs.open("time_logs.csv");
  time_logs << "run,type,time" << std::endl;

  for (int run = 1; run <= config.nb_runs; run++)
  {
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(u_dim);
    Eigen::VectorXd current_state = problem->getStartingState();

    // Initiate everything needed for the run
    int step = 0;
    bool finish_run = false;
    double trajectory_reward = 0;

    while (!finish_run)
    {
      cmd = mre.getAction(current_state);
      csa_mdp::Sample new_sample = problem->getSample(current_state, cmd);
      write_log(logs, run, step, current_state, cmd);
      // Feed mre
      mre.feed(new_sample);
      trajectory_reward += new_sample.reward;
      current_state = new_sample.next_state;
      // If a terminal state is reached, break the run
      if (problem->isTerminal(current_state) || step >= config.nb_steps)
      {
        cmd = cmd * 0;//No more active command
        finish_run = true;
      }
      step++;
    }//End of while OK
    write_log(logs, run, step, current_state, cmd);
    std::cout << "Reward for run " << run << ": " << trajectory_reward << std::endl;
    //std::cout << "Updating policy " << run << std::endl;
    mre.updatePolicy();
    //std::cout << "\tTime spent to compute q_value   : " << mre.getQValueTime() << "[s]"
    //          << std::endl;
    //std::cout << "\tTime spent to compute the policy: " << mre.getPolicyTime() << "[s]"
    //          << std::endl;
    //std::string prefix = details_path + "/T" + std::to_string(run) + "_";
    //std::cout << "Saving all with prefix " << prefix << std::endl;
    //mre.saveStatus(prefix);
    // Log time
    time_logs << run << ",qValue," << mre.getQValueTime() << std::endl;
    time_logs << run << ",policy," << mre.getPolicyTime() << std::endl;
  }
  logs.close();
  time_logs.close();
}
