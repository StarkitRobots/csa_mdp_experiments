#include "problems/extended_problem_factory.h"

#include "rhoban_csa_mdp/core/history.h"
#include "rhoban_csa_mdp/solvers/fpf.h"

#include <fstream>

#include <unistd.h>

using namespace csa_mdp;

class Config : public rhoban_utils::JsonSerializable
{
public:
  Config()
    {
    }

  std::string getClassName() const override
    {
      return "policy_learner";
    }

  Json::Value toJson() const override
    {
      Json::Value v;
      v["history_conf"] = history_conf.toJson();
      v["fpf_conf"    ] = fpf_conf.toJson();
      return v;
    }

  void fromJson(const Json::Value & v, const std::string & dir_name) override
    {
      history_conf.read(v, "history_conf", dir_name);
      fpf_conf.read(v, "fpf_conf", dir_name);
    }

  History::Config history_conf;
  FPF::Config fpf_conf;
};

void usage()
{
  std::cerr << "Usage: ... <config_path>" << std::endl;
  exit(EXIT_FAILURE);
}

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    usage();
  }
  std::string config_path = (argv[1]);

  // Registering extra_types
//TODO: Fix problems
//  ExtendedProblemFactory::registerExtraProblems();

  if (chdir(config_path.c_str()))
  {
    std::cerr << "Failed to set '" << config_path << "' as working directory." << std::endl;
    exit(EXIT_FAILURE);
  }

  /// CONFIG
  Config config;
  config.loadFile();// Loading file Config.xml

  std::shared_ptr<Problem> problem = config.history_conf.problem;

  if (problem->getNbActions() != 1) {
    throw std::runtime_error("main: learn_from_logs not implemented for multi actions problems");
  }

  // Setting limits
  config.fpf_conf.setStateLimits(problem->getStateLimits());
  config.fpf_conf.setActionLimits(problem->getActionLimits(0));

  // Reading csv file
  std::vector<History> histories = History::readCSV(config.history_conf);

  // Producing Samples
  std::vector<csa_mdp::Sample> samples = History::getBatch(histories);

  std::cout << "Computing policies from " << samples.size() << " samples" << std::endl;

  FPF solver;
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
  solver.solve(samples, is_terminal, config.fpf_conf);
  solver.getValueForest().save("q_values.data");
  for (int dim = 0; dim < config.fpf_conf.getActionLimits().rows(); dim++)
  {
    solver.getPolicyForest(dim).save("policy_" + std::to_string(dim) + ".data");
  }

  std::ofstream time_file;
  time_file.open("time_consumption");
  time_file << "QValue TS: " << config.fpf_conf.q_training_set_time << " [s]" << std::endl;
  time_file << "QValue ET: " << config.fpf_conf.q_extra_trees_time  << " [s]" << std::endl;
  time_file << "Policy TS: " << config.fpf_conf.p_training_set_time << " [s]" << std::endl;
  time_file << "Policy ET: " << config.fpf_conf.p_extra_trees_time  << " [s]" << std::endl;
}
