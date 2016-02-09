#include "problems/cart_pole.h"

#include "rosban_csa_mdp/core/history.h"
#include "rosban_csa_mdp/solvers/fpf.h"

#include <ros/ros.h>

#include <unistd.h>

using csa_mdp::History;
using csa_mdp::Problem;
using csa_mdp::FPF;

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
      rosban_utils::xml_tools::write<std::string>("log_path"      , log_path      , out);
      rosban_utils::xml_tools::write<std::string>("problem"       , problem       , out);
      rosban_utils::xml_tools::write<int>        ("run_column"    , run_column    , out);
      rosban_utils::xml_tools::write<int>        ("step_column"   , step_column   , out);
      rosban_utils::xml_tools::write_vector<int> ("state_columns" , state_columns , out);
      rosban_utils::xml_tools::write_vector<int> ("action_columns", action_columns, out);
      fpf_conf.write("fpf_conf", out);
    }

  void from_xml(TiXmlNode *node) override
    {
      log_path       = rosban_utils::xml_tools::read<std::string>(node, "log_path"      );
      problem        = rosban_utils::xml_tools::read<std::string>(node, "problem"       );
      run_column     = rosban_utils::xml_tools::read<int>        (node, "run_column"    );
      step_column    = rosban_utils::xml_tools::read<int>        (node, "step_column"   );
      state_columns  = rosban_utils::xml_tools::read_vector<int> (node, "state_columns" );
      action_columns = rosban_utils::xml_tools::read_vector<int> (node, "action_columns");
      fpf_conf.read(node, "fpf_conf");
    }

  std::string log_path;
  std::string problem;
  int run_column;
  int step_column;
  std::vector<int> state_columns;
  std::vector<int> action_columns;
  FPF::Config fpf_conf;
};


int main(int argc, char ** argv)
{
  std::string config_path = ros::getROSArg(argc, argv, "config_path");
  if (config_path == "")
  {
    std::cerr << "Usage: rosrun .... config_path:=<path>" << std::endl;
    std::cerr << "\tNote: <path> folder should contain a file named Config.xml" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (chdir(config_path.c_str()))
  {
    std::cerr << "Failed to set '" << config_path << "' as working directory." << std::endl;
    exit(EXIT_FAILURE);
  }

  /// CONFIG
  Config config;
  config.load_file();// Loading file Config.xml

  // Choosing problem
  ControlProblem * problem = NULL;
  if (config.problem == "CartPole")
  {
    problem = new CartPole();
  }
  else
  {
    std::cerr << "Unknown problem: '" << config.problem << "'" << std::endl;
    exit(EXIT_FAILURE);
  }


  // Specifying reward function
  Problem::RewardFunction reward_func = [problem](const Eigen::VectorXd &src,
                                                  const Eigen::VectorXd &action,
                                                  const Eigen::VectorXd &result)
    {
      return problem->getReward(src, action, result);
    };

  // Reading csv file
  std::vector<History> histories = History::readCSV(config.log_path,
                                                    config.run_column,
                                                    config.step_column,
                                                    config.state_columns,
                                                    config.action_columns,
                                                    reward_func,
                                                    true);

  // Producing Samples
  std::vector<csa_mdp::Sample> samples = History::getBatch(histories);

  FPF solver;
  solver.conf = config.fpf_conf;
  auto is_terminal = [config](const Eigen::VectorXd &state)
    {
      const Eigen::MatrixXd &state_limits = config.fpf_conf.getStateLimits();
      for (int dim = 0; dim < state.rows(); dim++)
      {
        if (state(dim) > state_limits(dim,1) || state(dim) < state_limits(dim,0))
        {
          return true;
        }
      }
      return false;
    };
  solver.solve(samples, is_terminal);
  solver.getValueForest().save("q_values.data");
  for (int dim = 0; dim < config.fpf_conf.getActionLimits().rows(); dim++)
  {
    solver.getPolicyForest(dim).save("policy_" + std::to_string(dim) + ".data");
  }


}
