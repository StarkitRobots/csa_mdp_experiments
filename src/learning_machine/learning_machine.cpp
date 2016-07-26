#include "learning_machine/learning_machine.h"

#include "rosban_csa_mdp/core/history.h"
#include "rosban_csa_mdp/core/problem_factory.h"
#include "rosban_csa_mdp/solvers/learner_factory.h"

#include "rosban_utils/benchmark.h"

#include <sys/stat.h>

using csa_mdp::History;
using csa_mdp::Learner;
using csa_mdp::LearnerFactory;
using csa_mdp::Problem;
using csa_mdp::ProblemFactory;

using rosban_utils::Benchmark;

using csa_mdp::Policy;

std::string LearningMachine::details_path("details");

LearningMachine::LearningMachine()
  : run(1), step(0), discount(0.98), nb_threads(1),
    policy_id(1), policy_runs_required(1), policy_runs_performed(0), policy_total_reward(0),
    best_policy_score(std::numeric_limits<double>::lowest()),
    time_budget(std::numeric_limits<double>::max()),
    update_rule(UpdateRule::square),
    save_details(false), save_run_logs(true), save_best_policy(true)
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

void LearningMachine::setLearningDimensions(const std::vector<int> & new_learning_dimensions)
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
  // Only propagate Limits is 
  if (problem && learner)
  {
    learner->setStateLimits(getLearningSpace(problem->getStateLimits()));
    learner->setActionLimits(problem->getActionLimits());
    learner->setDiscount(discount);
    learner->setNbThreads(nb_threads);
  }
}

void LearningMachine::execute()
{
  init();
  while(alive() && run <= nb_runs)
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
  while (alive() && step < nb_steps && !problem->isTerminal(current_state))
  {
    doStep();
    step++;
  }
  writeTimeLog("simulation", Benchmark::close());
  endRun();
}

void LearningMachine::doStep()
{
  Eigen::VectorXd cmd = learner->getAction(getLearningState(current_state));
  Eigen::VectorXd last_state = current_state;
  applyAction(cmd);
  if (save_run_logs) {
    writeRunLog(run_logs, run, step, last_state, cmd, current_reward);
  }
  csa_mdp::Sample new_sample(getLearningState(last_state),
                             cmd,
                             getLearningState(current_state),
                             current_reward);
  // Add new sample
  learner->feed(new_sample);
  trajectory_reward += current_reward;
  double disc_reward = current_reward * std::pow(discount, step);
  trajectory_disc_reward += disc_reward;
}

void LearningMachine::init()
{
  learner->setStart();
  // First of all open/reset streams if necessary
  closeActiveStreams();
  openStreams();
  // Write Headers
  if (save_run_logs) { writeRunLogHeader(run_logs); }
  time_logs << "policy,run,type,time" << std::endl;
  reward_logs << "run,policy,reward,disc_reward,elapsed_time" << std::endl;
  // Preload some experiment
  if (seed_path != "")
  {
    std::vector<History> histories = History::readCSV(seed_path,
                                                      problem->getStateLimits().rows(),
                                                      problem->getActionLimits().rows());
    std::vector<csa_mdp::Sample> samples = History::getBatch(histories);
    for (const csa_mdp::Sample & s : samples)
    {
      const csa_mdp::Sample learning_sample(getLearningState(s.state),
                                            s.action,
                                            getLearningState(s.next_state),
                                            s.reward);
      learner->feed(learning_sample);
    }
    learner->internalUpdate();
  }
  // If saving details, then create folder
  if (save_details) { createDetailFolder(); }
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
  current_reward = 0;
}

void LearningMachine::endRun()
{
  policy_runs_performed++;
  policy_total_reward += trajectory_disc_reward;
  // If the maximal step has not been reached, it mean we reached a final state
  int u_dim = problem->getActionLimits().rows();
  if (save_run_logs) {
    writeRunLog(run_logs, run, step, current_state, Eigen::VectorXd::Zero(u_dim), 0);
  }
  reward_logs << run << ","
              << policy_id << ","
              << trajectory_reward << ","
              << trajectory_disc_reward << ","
              << learner->getLearningTime() << std::endl;
  // If it is the last run of the policy, perform some operations
  if (policy_runs_performed >= policy_runs_required)
  {
    // If current policy is better than the other, then save it
    double policy_score = policy_total_reward / policy_runs_performed;
    if (save_best_policy  &&
        learner->hasAvailablePolicy() &&
        policy_score > best_policy_score) {
      createDetailFolder();
      std::ostringstream oss;
      oss << details_path << "/best_";
      std::string prefix = oss.str();
      learner->savePolicy(prefix);
      best_policy_score = policy_score;
      std::cout << "Found a new 'best policy' at policy_id: " << policy_id
                << " with a score of: " << policy_score << std::endl;
    }
    // Update internal structure only if there is still some runs to go
    if (run < nb_runs) {
      learner->internalUpdate();
      // Save q_value and nb_steps
      if (save_details)
      {
        std::ostringstream oss;
        oss << details_path << "/update_" << policy_id << "_";
        std::string prefix = oss.str();
        learner->saveStatus(prefix);
      }
      // Write time entries
      for (const auto & entry : learner->getTimeRepartition())
      {
        writeTimeLog(entry.first, entry.second);
      }
      // Set properties for the next policy
      policy_id++;
      policy_total_reward = 0;
      policy_runs_performed = 0;
      switch(update_rule)
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
}

void LearningMachine::closeActiveStreams()
{
  if (run_logs.is_open()   ) { run_logs.close();    }
  if (time_logs.is_open()  ) { time_logs.close();   }
  if (reward_logs.is_open()) { reward_logs.close(); }
}

void LearningMachine::openStreams()
{
  if (save_run_logs) { run_logs.open("run_logs.csv"); }
  time_logs.open("time_logs.csv");
  reward_logs.open("reward_logs.csv");
}

void LearningMachine::writeRunLogHeader(std::ostream &out)
{
  out << "run,step,";
  // State information
  for (const std::string & name : problem->getStateNames())
  {
    out << name << ",";
  }
  // Commands
  for (const std::string & name : problem->getActionNames())
  {
    out << name << ",";
  }
  out << "reward" << std::endl;
}

void LearningMachine::writeTimeLog(const std::string &type, double time)
{
  time_logs << policy_id << "," << run << "," << type << "," << time << std::endl;
}

void LearningMachine::writeRunLog(std::ostream &out, int run, int step,
                             const Eigen::VectorXd &state,
                             const Eigen::VectorXd &action,
                             double reward)
{
  out << run << "," << step << ",";

  for (int i = 0; i < state.rows(); i++)
  {
    out << state(i) << ",";
  }
  for (int i = 0; i < action.rows(); i++)
  {
    out << action(i) << ",";
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

Eigen::MatrixXd LearningMachine::getLearningSpace(const Eigen::MatrixXd & space)
{
  Eigen::MatrixXd learning_space(learning_dimensions.size(), 2);
  for (size_t dim = 0; dim < learning_dimensions.size(); dim++)
  {
    learning_space.row(dim) = space.row(learning_dimensions[dim]);
  }
  return learning_space;
}

Eigen::VectorXd LearningMachine::getLearningState(const Eigen::VectorXd & state)
{
  Eigen::VectorXd learning_state(learning_dimensions.size());
  for (size_t dim = 0; dim < learning_dimensions.size(); dim++)
  {
    learning_state(dim) = state(learning_dimensions[dim]);
  }
  return learning_state;
}

std::string LearningMachine::class_name() const
{
  return "LearningMachine";
}

void LearningMachine::to_xml(std::ostream &out) const
{
  if (learner)
  {
    out << "<learner>";
    learner->write(problem->class_name(), out);
    out << "</learner>";
  }
  if (problem)
  {
    out << "<problem>";
    problem->write(problem->class_name(), out);
    out << "</problem>";
  }
  rosban_utils::xml_tools::write<std::string>("update_rule", to_string(update_rule), out);
  rosban_utils::xml_tools::write<int>("nb_runs", nb_runs, out);
  rosban_utils::xml_tools::write<int>("nb_steps", nb_steps, out);
  rosban_utils::xml_tools::write<int>("nb_threads", nb_threads, out);
  rosban_utils::xml_tools::write<double>("discount", discount, out);
  rosban_utils::xml_tools::write<double>("time_budget", time_budget, out);
  rosban_utils::xml_tools::write<bool>("save_details", save_details, out);
  rosban_utils::xml_tools::write<bool>("save_run_logs", save_run_logs, out);
  rosban_utils::xml_tools::write<bool>("save_best_policy", save_best_policy, out);
  rosban_utils::xml_tools::write_vector<int>("learning_dimensions", learning_dimensions, out);
}

void LearningMachine::from_xml(TiXmlNode *node)
{
  // First: read problem
  TiXmlNode * problem_node = node->FirstChild("problem");
  if(!problem_node) {
    throw std::runtime_error("Failed to find node 'problem' in '" + node->ValueStr() + "'");
  }
  setProblem(std::unique_ptr<Problem>(ProblemFactory().build(problem_node)));
  // Override learning dimensions if custom config is provided
  std::vector<int> new_learning_dims;
  rosban_utils::xml_tools::try_read_vector<int>(node, "learning_dimensions", new_learning_dims);
  if (new_learning_dims.size() > 0) setLearningDimensions(new_learning_dims);
  // Then: read learner
  TiXmlNode * learner_node = node->FirstChild("learner");
  if(!learner_node) {
    throw std::runtime_error("Failed to find node 'learner' in '" + node->ValueStr() + "'");
  }
  setLearner(std::unique_ptr<Learner>(LearnerFactory().build(learner_node)));
  // Then... read everything else
  std::string update_rule_str;
  rosban_utils::xml_tools::try_read<std::string>(node, "update_rule", update_rule_str);
  if (update_rule_str != "")
  {
    update_rule = loadUpdateRule(update_rule_str);
  }
  nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
  nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
  rosban_utils::xml_tools::try_read<int>   (node, "nb_threads", nb_threads);
  rosban_utils::xml_tools::try_read<double>(node, "discount", discount);
  setDiscount(discount);
  rosban_utils::xml_tools::try_read<double>(node, "time_budget", time_budget);
  rosban_utils::xml_tools::try_read<bool>(node, "save_details", save_details);
  rosban_utils::xml_tools::try_read<bool>(node, "save_run_logs", save_run_logs);
  rosban_utils::xml_tools::try_read<bool>(node, "save_best_policy", save_best_policy);
  rosban_utils::xml_tools::try_read<std::string>(node, "seed_path", seed_path);
}

std::string to_string(LearningMachine::UpdateRule rule)
{
  switch (rule)
  {
    case LearningMachine::UpdateRule::each:   return "each";
    case LearningMachine::UpdateRule::square: return "square";
  }
  throw std::runtime_error("Unknown LearningMachine::UpdateRule type in to_string(Type)");
}

LearningMachine::UpdateRule LearningMachine::loadUpdateRule(const std::string &rule)
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
