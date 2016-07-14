#include "mre_machine/mre_machine.h"

#include "rosban_csa_mdp/core/history.h"
#include "rosban_csa_mdp/core/problem_factory.h"
#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_utils/benchmark.h"

#include <sys/stat.h>

using csa_mdp::History;
using csa_mdp::PolicyFactory;
using csa_mdp::Problem;
using csa_mdp::ProblemFactory;
using csa_mdp::MRE;

using rosban_utils::Benchmark;

using csa_mdp::Policy;

std::string MREMachine::details_path("details");

MREMachine::Config::Config()
  : mode(MREMachine::Mode::exploration), save_details(false), save_run_logs(true)
{
}

std::string MREMachine::Config::class_name() const
{
  return "Config";
}

void MREMachine::Config::to_xml(std::ostream &out) const
{
  rosban_utils::xml_tools::write<std::string>("mode", to_string(mode), out);
  rosban_utils::xml_tools::write<std::string>("update_rule", to_string(update_rule), out);
  rosban_utils::xml_tools::write<int>("nb_runs", nb_runs, out);
  rosban_utils::xml_tools::write<int>("nb_steps", nb_steps, out);
  rosban_utils::xml_tools::write<bool>("save_details", save_details, out);
  rosban_utils::xml_tools::write<bool>("save_run_logs", save_run_logs, out);
  out << "<problem>";
  problem->write(problem->class_name(), out);
  out << "</problem>";
  switch(mode)
  {
    case MREMachine::Mode::evaluation:
      policy->write("policy", out);
      break;
    case MREMachine::Mode::exploration:
      mre_config.write("mre", out);
      break;
    case MREMachine::Mode::full:
      throw std::runtime_error("MREMachine does not implement mode 'full' yet");
  }
}

void MREMachine::Config::from_xml(TiXmlNode *node)
{
  std::string mode_str = rosban_utils::xml_tools::read<std::string>(node, "mode");
  mode = loadMode(mode_str);
  std::string update_rule_str;
  rosban_utils::xml_tools::try_read<std::string>(node, "update_rule", update_rule_str);
  if (update_rule_str != "")
  {
    update_rule = loadUpdateRule(update_rule_str);
  }
  nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
  nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
  rosban_utils::xml_tools::try_read<bool>(node, "save_details", save_details);
  rosban_utils::xml_tools::try_read<bool>(node, "save_run_logs", save_run_logs);
  TiXmlNode * problem_node = node->FirstChild("problem");
  if(!problem_node) throw std::runtime_error("Failed to find node 'problem'");
  problem = std::unique_ptr<Problem>(ProblemFactory().build(problem_node));
  switch(mode)
  {
    case MREMachine::Mode::evaluation:
    {
      TiXmlNode * policy_node = node->FirstChild("policy");
      if(!policy_node) throw std::runtime_error("Failed to find node 'policy'");
      policy = std::unique_ptr<Policy>(PolicyFactory().build(policy_node));
      break;
    }
    case MREMachine::Mode::exploration:
      mre_config.read(node, "mre");
      rosban_utils::xml_tools::try_read<std::string>(node, "seed_path", seed_path);
      break;
    case MREMachine::Mode::full:
      throw std::runtime_error("MREMachine does not implement mode 'full' yet");      
  }
}

MREMachine::MREMachine(std::shared_ptr<Config> config_)
  : config(config_), run(1), step(0),
    policy_id(1), policy_runs_required(1), policy_runs_performed(0), policy_total_reward(0),
    best_policy_score(std::numeric_limits<double>::lowest())
{
  // Generating problem
  problem = config->problem;
  // Applying problem Limits
  config->mre_config.mrefpf_conf.setStateLimits(problem->getStateLimits());
  config->mre_config.mrefpf_conf.setActionLimits(problem->getActionLimits());
  // Initalizing mre
  if ( config->mode != MREMachine::Mode::evaluation )
  {
    mre = std::unique_ptr<MRE>(new MRE(config->mre_config,
                                       [this](const Eigen::VectorXd &state)
    {
      return this->problem->isTerminal(state);
    }
                                 ));
  }
  // Setting action space for policy
  if ( config->mode == MREMachine::Mode::evaluation )
  {
    config->policy->setActionLimits(problem->getActionLimits());
  }
  // Opening log files
  if (config->save_run_logs) { run_logs.open("run_logs.csv"); }
  time_logs.open("time_logs.csv");
  reward_logs.open("reward_logs.csv");
}

MREMachine::~MREMachine()
{
  if (config->save_run_logs) { run_logs.close(); }
  time_logs.close();
  reward_logs.close();
}

void MREMachine::execute()
{
  init();
  while(alive() && run <= config->nb_runs)
  {
    doRun();
    run++;
  }
}

void MREMachine::doRun()
{
  Benchmark::open("preparation");
  prepareRun();
  writeTimeLog("preparation", Benchmark::close());
  Benchmark::open("simulation");
  while (alive() && step < config->nb_steps && !problem->isTerminal(current_state))
  {
    doStep();
    step++;
  }
  writeTimeLog("simulation", Benchmark::close());
  endRun();
}

void MREMachine::doStep()
{
  Eigen::VectorXd cmd;
  switch(config->mode)
  {
    case MREMachine::Mode::exploration:
      cmd = mre->getAction(current_state);
      break;
    case MREMachine::Mode::evaluation:
      cmd = config->policy->getAction(current_state);
      break;
    case MREMachine::Mode::full:
      throw std::runtime_error("MREMachine doest not implement mode 'full' yet");      
  }
  Eigen::VectorXd last_state = current_state;
  applyAction(cmd);
  if (config->save_run_logs) {
    writeRunLog(run_logs, run, step, last_state, cmd, current_reward);
  }
  if (config->mode == MREMachine::Mode::exploration)
  {
    csa_mdp::Sample new_sample(last_state,
                               cmd,
                               current_state,
                               current_reward);
    // Add new sample
    mre->feed(new_sample);
  }
  trajectory_reward += current_reward;
  double disc = config->mre_config.mrefpf_conf.discount;
  double disc_reward = current_reward * std::pow(disc, step);
  trajectory_disc_reward += disc_reward;
}

void MREMachine::init()
{
  if (config->save_run_logs) { writeRunLogHeader(run_logs); }
  time_logs << "run,type,time" << std::endl;
  reward_logs << "run,policy,reward,disc_reward" << std::endl;
  // Preload some experiment
  if (config->mode == MREMachine::Mode::exploration && config->seed_path != "")
  {
    std::vector<History> histories = History::readCSV(config->seed_path,
                                                      problem->getStateLimits().rows(),
                                                      problem->getActionLimits().rows());
    std::vector<csa_mdp::Sample> samples = History::getBatch(histories);
    for (const csa_mdp::Sample & s : samples)
    {
      mre->feed(s);
    }
    mre->updatePolicy();
  }
  // If saving details, then create folder
  if (config->save_details) { createDetailFolder(); }
}

bool MREMachine::alive()
{
  return true;
}

void MREMachine::prepareRun()
{
  step = 0;
  trajectory_reward = 0;
  trajectory_disc_reward = 0;
  current_reward = 0;
}

void MREMachine::endRun()
{
  policy_runs_performed++;
  policy_total_reward += trajectory_reward;
  // If the maximal step has not been reached, it mean we reached a final state
  int u_dim = problem->getActionLimits().rows();
  if (config->save_run_logs) {
    writeRunLog(run_logs, run, step, current_state, Eigen::VectorXd::Zero(u_dim), 0);
  }
  reward_logs << run << ","
              << policy_id << ","
              << trajectory_reward << ","
              << trajectory_disc_reward << std::endl;
  // If it is the last run of the policy, perform some operations
  if (config->mode == MREMachine::Mode::exploration &&
      policy_runs_performed >= policy_runs_required)
  {
    // If current policy is better than the other, then save it
    double policy_score = policy_total_reward / policy_runs_performed;
    if (mre->hasAvailablePolicy() && policy_score > best_policy_score) {
      createDetailFolder();
      std::ostringstream oss;
      oss << details_path << "/best_";
      std::string prefix = oss.str();
      mre->savePolicies(prefix);
      best_policy_score = policy_score;
      std::cout << "Found a new 'best policy' at policy_id: " << policy_id
                << " with a score of: " << policy_score << std::endl;
    }
    // Do not update policy if its the last trial (it won't be tested)
    if (run < config->nb_runs) {
      mre->updatePolicy();
      // Save q_value and nb_steps
      if (config->save_details)
      {
        std::ostringstream oss;
        oss << details_path << "/update_" << policy_id << "_";
        std::string prefix = oss.str();
        mre->saveStatus(prefix);
      }
      writeTimeLog("qValueTS", mre->getQValueTrainingSetTime());
      writeTimeLog("qValueET", mre->getQValueExtraTreesTime());
      writeTimeLog("policyTS", mre->getPolicyTrainingSetTime());
      writeTimeLog("policyET", mre->getPolicyExtraTreesTime());
      // Set properties for the next policy
      policy_id++;
      policy_total_reward = 0;
      policy_runs_performed = 0;
      switch(config->update_rule)
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
  if (config->mode == MREMachine::Mode::evaluation)
  {
    config->policy->init();
  }
}

void MREMachine::writeRunLogHeader(std::ostream &out)
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

void MREMachine::writeTimeLog(const std::string &type, double time)
{
  time_logs << run << "," << type << "," << time << std::endl;
}

void MREMachine::writeRunLog(std::ostream &out, int run, int step,
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

void MREMachine::createDetailFolder() const
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

std::string to_string(MREMachine::Mode mode)
{
  switch (mode)
  {
    case MREMachine::Mode::exploration: return "exploration";
    case MREMachine::Mode::evaluation:  return "evaluation";
    case MREMachine::Mode::full:        return "full";
  }
  throw std::runtime_error("Unknown MREMachine::Mode type in to_string(Type)");
}

MREMachine::Mode loadMode(const std::string &mode)
{
  if (mode == "exploration")
  {
    return MREMachine::Mode::exploration;
  }
  if (mode == "evaluation")
  {
    return MREMachine::Mode::evaluation;
  }
  if (mode == "full")
  {
    return MREMachine::Mode::full;
  }
  throw std::runtime_error("Unknown MREMachine::Mode: '" + mode + "'");
}

std::string to_string(MREMachine::UpdateRule rule)
{
  switch (rule)
  {
    case MREMachine::UpdateRule::each:   return "each";
    case MREMachine::UpdateRule::square: return "square";
  }
  throw std::runtime_error("Unknown MREMachine::UpdateRule type in to_string(Type)");
}

MREMachine::UpdateRule loadUpdateRule(const std::string &rule)
{
  if (rule == "each")
  {
    return MREMachine::UpdateRule::each;
  }
  if (rule == "square")
  {
    return MREMachine::UpdateRule::square;
  }
  throw std::runtime_error("Unknown MREMachine::UpdateRule: '" + rule + "'");
}
