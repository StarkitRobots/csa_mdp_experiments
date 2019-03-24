#include "learning_machine/learning_machine.h"

#include "rhoban_csa_mdp/core/history.h"
#include "rhoban_csa_mdp/core/problem_factory.h"
#include "rhoban_csa_mdp/solvers/learner_factory.h"

#include "rhoban_utils/timing/benchmark.h"

#include <sys/stat.h>

using csa_mdp::History;
using csa_mdp::Learner;
using csa_mdp::LearnerFactory;
using csa_mdp::Problem;
using csa_mdp::ProblemFactory;

using rhoban_utils::Benchmark;

using csa_mdp::Policy;

namespace csa_mdp
{
std::string LearningMachine::details_path("details");

LearningMachine::LearningMachine()
  : run(1)
  , step(0)
  , discount(0.98)
  , nb_threads(1)
  , policy_id(1)
  , policy_runs_required(1)
  , policy_runs_performed(0)
  , policy_total_reward(0)
  , best_policy_score(std::numeric_limits<double>::lowest())
  , update_rule(UpdateRule::square)
  , time_budget(std::numeric_limits<double>::max())
  , save_details(false)
  , save_run_logs(true)
  , save_best_policy(true)
{
}

LearningMachine::~LearningMachine()
{
  closeActiveStreams();
}

void LearningMachine::setProblem(std::unique_ptr<csa_mdp::Problem> new_problem)
{
  problem = std::move(new_problem);
  learning_dimensions = problem->getLearningDimensions();
  propagate();
}

void LearningMachine::setLearner(std::unique_ptr<csa_mdp::Learner> new_learner)
{
  learner = std::move(new_learner);
  propagate();
}

void LearningMachine::setLearningDimensions(const std::vector<int>& new_learning_dimensions)
{
  learning_dimensions = new_learning_dimensions;
  propagate();
}

void LearningMachine::setDiscount(double new_discount)
{
  discount = new_discount;
  propagate();
}

void LearningMachine::propagate()
{
  // Only propagate if both problem and learner have been set
  if (problem && learner)
  {
    learner->setStateLimits(getLearningSpace(problem->getStateLimits()));
    learner->setActionLimits(problem->getActionsLimits());
    learner->setDiscount(discount);
    learner->setNbThreads(nb_threads);
  }
}

void LearningMachine::execute()
{
  init();
  while (alive() && run <= nb_runs)
  {
    doRun();
    run++;
  }
}

void LearningMachine::doRun()
{
  Benchmark::open("preparation");
  prepareRun();
  writeTimeLog("preparation", Benchmark::close());
  Benchmark::open("simulation");
  while (alive() && step < nb_steps && !status.terminal)
  {
    doStep();
    step++;
  }
  writeTimeLog("simulation", Benchmark::close());
  endRun();
}

void LearningMachine::doStep()
{
  Eigen::VectorXd cmd = learner->getAction(getLearningState(status.successor));
  Eigen::VectorXd last_state = status.successor;
  applyAction(cmd);
  if (save_run_logs)
  {
    writeRunLog(run_logs, run, step, last_state, cmd, status.reward);
  }
  csa_mdp::Sample new_sample(getLearningState(last_state), cmd, getLearningState(status.successor), status.reward);
  // Add new sample
  learner->feed(new_sample);
  trajectory_reward += status.reward;
  double disc_reward = status.reward * std::pow(discount, step);
  trajectory_disc_reward += disc_reward;
}

void LearningMachine::init()
{
  learner->setStart();
  // First of all open/reset streams if necessary
  closeActiveStreams();
  openStreams();
  // Write Headers
  if (save_run_logs)
  {
    writeRunLogHeader(run_logs);
  }
  time_logs << "policy,run,type,time" << std::endl;
  reward_logs << "run,policy,reward,disc_reward,elapsed_time" << std::endl;
  // Preload some experiment
  if (seed_path != "")
  {
    if (problem->getNbActions() != 1)
    {
      throw std::logic_error("LearningMachine::init: seeds not handled for multi-actions problems");
    }

    std::cout << "Loading experiments from the seed at '" << seed_path << "'" << std::endl;
    std::vector<History> histories =
        History::readCSV(seed_path, problem->getStateLimits().rows(), problem->getActionLimits(0).rows());
    std::vector<csa_mdp::Sample> samples = History::getBatch(histories);
    for (const csa_mdp::Sample& s : samples)
    {
      const csa_mdp::Sample learning_sample(getLearningState(s.state), s.action, getLearningState(s.next_state),
                                            s.reward);
      learner->feed(learning_sample);
    }
    std::cout << "\tExperiments loaded" << std::endl;
    learner->internalUpdate();
    std::cout << "\tPreliminary update done" << std::endl;
  }
  // If saving details, then create folder
  if (save_details)
  {
    createDetailFolder();
  }
}

bool LearningMachine::alive()
{
  return learner->getLearningTime() < time_budget;
}

void LearningMachine::prepareRun()
{
  step = 0;
  trajectory_reward = 0;
  trajectory_disc_reward = 0;
  status.reward = 0;
  status.terminal = false;
}

void LearningMachine::endRun()
{
  policy_runs_performed++;
  policy_total_reward += trajectory_disc_reward;
  // If the maximal step has not been reached, it mean we reached a final state
  if (save_run_logs)
  {
    Eigen::VectorXd fake_action;
    writeRunLog(run_logs, run, step, status.successor, fake_action, 0);
  }
  reward_logs << run << "," << policy_id << "," << trajectory_reward << "," << trajectory_disc_reward << ","
              << learner->getLearningTime() << std::endl;
  // If it is the last run of the policy, perform some operations
  if (policy_runs_performed >= policy_runs_required)
  {
    // If current policy is better than the other, then save it
    double policy_score = policy_total_reward / policy_runs_performed;
    if (save_best_policy && learner->hasAvailablePolicy() && policy_score > best_policy_score)
    {
      createDetailFolder();
      std::ostringstream oss;
      oss << details_path << "/best_";
      std::string prefix = oss.str();
      learner->saveStatus(prefix);
      best_policy_score = policy_score;
      std::cout << "Found a new 'best policy' at policy_id: " << policy_id << " with a score of: " << policy_score
                << std::endl;
    }
    // Save the current status
    if (save_details)
    {
      std::ostringstream oss;
      oss << details_path << "/update_" << policy_id << "_";
      std::string prefix = oss.str();
      learner->saveStatus(prefix);
    }
    // Update internal structure only if there is still some runs to go
    if (run < nb_runs)
    {
      learner->internalUpdate();
      // Write time entries
      for (const auto& entry : learner->getTimeRepartition())
      {
        writeTimeLog(entry.first, entry.second);
      }
      // Set properties for the next policy
      policy_id++;
      policy_total_reward = 0;
      policy_runs_performed = 0;
      switch (update_rule)
      {
        case UpdateRule::each:
          policy_runs_required = 1;
          break;
        case UpdateRule::square:
          policy_runs_required = policy_id;
          break;
      }
    }
  }
  learner->endRun();
}

void LearningMachine::closeActiveStreams()
{
  if (run_logs.is_open())
  {
    run_logs.close();
  }
  if (time_logs.is_open())
  {
    time_logs.close();
  }
  if (reward_logs.is_open())
  {
    reward_logs.close();
  }
}

void LearningMachine::openStreams()
{
  if (save_run_logs)
  {
    run_logs.open("run_logs.csv");
  }
  time_logs.open("time_logs.csv");
  reward_logs.open("reward_logs.csv");
}

void LearningMachine::writeRunLogHeader(std::ostream& out)
{
  out << "run,step,";
  // State information
  for (const std::string& name : problem->getStateNames())
  {
    out << name << ",";
  }
  // Commands
  out << "action_id,";
  for (int action_id = 0; action_id < problem->getNbActions(); action_id++)
  {
    for (const std::string& name : problem->getActionNames(action_id))
    {
      out << "a" << action_id << "_" << name << ",";
    }
  }
  out << "reward" << std::endl;
}

void LearningMachine::writeTimeLog(const std::string& type, double time)
{
  time_logs << policy_id << "," << run << "," << type << "," << time << std::endl;
}

void LearningMachine::writeRunLog(std::ostream& out, int run, int step, const Eigen::VectorXd& state,
                                  const Eigen::VectorXd& action, double reward)
{
  out << run << "," << step << ",";

  for (int i = 0; i < state.rows(); i++)
  {
    out << state(i) << ",";
  }
  int curr_action_id = -1;
  if (action.rows() > 0)
    curr_action_id = action(0);
  out << curr_action_id << ",";
  // Currently jumping first element of action (only used for multiple action spaces problems)
  for (int action_id = 0; action_id < problem->getNbActions(); action_id++)
  {
    if (curr_action_id == action_id)
    {
      for (int i = 1; i < action.rows(); i++)
      {
        out << action(i) << ",";
      }
    }
    else
    {
      for (int i = 0; i < problem->actionDims(action_id); i++)
      {
        out << "NA,";
      }
    }
  }
  out << reward << std::endl;
}

void LearningMachine::createDetailFolder() const
{
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
}

Eigen::MatrixXd LearningMachine::getLearningSpace(const Eigen::MatrixXd& space)
{
  Eigen::MatrixXd learning_space(learning_dimensions.size(), 2);
  for (size_t dim = 0; dim < learning_dimensions.size(); dim++)
  {
    learning_space.row(dim) = space.row(learning_dimensions[dim]);
  }
  return learning_space;
}

Eigen::VectorXd LearningMachine::getLearningState(const Eigen::VectorXd& state)
{
  Eigen::VectorXd learning_state(learning_dimensions.size());
  for (size_t dim = 0; dim < learning_dimensions.size(); dim++)
  {
    learning_state(dim) = state(learning_dimensions[dim]);
  }
  return learning_state;
}

std::string LearningMachine::getClassName() const
{
  return "LearningMachine";
}

Json::Value LearningMachine::toJson() const
{
  Json::Value v;
  if (learner)
  {
    v["learner"] = learner->toFactoryJson();
  }
  if (problem)
  {
    v["problem"] = problem->toFactoryJson();
  }
  v["update_rule"] = to_string(update_rule);
  v["nb_runs"] = nb_runs;
  v["nb_steps"] = nb_steps;
  v["nb_threads"] = nb_threads;
  v["discount"] = discount;
  v["time_budget"] = time_budget;
  v["save_details"] = save_details;
  v["save_run_logs"] = save_run_logs;
  v["save_best_policy"] = save_best_policy;
  v["learning_dimensions"] = rhoban_utils::vector2Json(learning_dimensions);
  return v;
}

void LearningMachine::fromJson(const Json::Value& v, const std::string& dir_name)
{
  // First: read problem
  std::unique_ptr<Problem> tmp_problem;
  std::string problem_path;
  rhoban_utils::tryRead(v, "problem_path", &problem_path);
  if (problem_path != "")
  {
    tmp_problem = ProblemFactory().buildFromJsonFile(dir_name + problem_path);
  }
  else
  {
    tmp_problem = ProblemFactory().read(v, "problem", dir_name);
  }
  if (!tmp_problem)
  {
    throw rhoban_utils::JsonParsingError("LearningMachine::fromJson: No problem found");
  }
  setProblem(std::move(tmp_problem));
  // Override learning dimensions if custom config is provided
  std::vector<int> new_learning_dims;
  rhoban_utils::tryReadVector(v, "learning_dimensions", &new_learning_dims);
  if (new_learning_dims.size() > 0)
    setLearningDimensions(new_learning_dims);
  // Then: read learner
  setLearner(LearnerFactory().read(v, "learner", dir_name));
  // Then... read everything else
  std::string update_rule_str;
  rhoban_utils::tryRead(v, "update_rule", &update_rule_str);
  if (update_rule_str != "")
  {
    update_rule = loadUpdateRule(update_rule_str);
  }
  nb_runs = rhoban_utils::read<int>(v, "nb_runs");
  nb_steps = rhoban_utils::read<int>(v, "nb_steps");
  rhoban_utils::tryRead(v, "nb_threads", &nb_threads);
  rhoban_utils::tryRead(v, "discount", &discount);
  rhoban_utils::tryRead(v, "time_budget", &time_budget);
  rhoban_utils::tryRead(v, "save_details", &save_details);
  rhoban_utils::tryRead(v, "save_run_logs", &save_run_logs);
  rhoban_utils::tryRead(v, "save_best_policy", &save_best_policy);
  rhoban_utils::tryRead(v, "seed_path", &seed_path);
  setDiscount(discount);
}

std::string to_string(LearningMachine::UpdateRule rule)
{
  switch (rule)
  {
    case LearningMachine::UpdateRule::each:
      return "each";
    case LearningMachine::UpdateRule::square:
      return "square";
  }
  throw std::runtime_error("Unknown LearningMachine::UpdateRule type in to_string(Type)");
}

LearningMachine::UpdateRule LearningMachine::loadUpdateRule(const std::string& rule)
{
  if (rule == "each")
  {
    return LearningMachine::UpdateRule::each;
  }
  if (rule == "square")
  {
    return LearningMachine::UpdateRule::square;
  }
  throw std::runtime_error("Unknown LearningMachine::UpdateRule: '" + rule + "'");
}

}  // namespace csa_mdp
