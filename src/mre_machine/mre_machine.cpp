#include "mre_machine/mre_machine.h"

#include "problems/problem_factory.h"

#include "rosban_utils/benchmark.h"

using csa_mdp::Problem;
using csa_mdp::MRE;

using rosban_utils::Benchmark;

MREMachine::Config::Config()
{
}

std::string MREMachine::Config::class_name() const
{
  return "Config";
}

void MREMachine::Config::to_xml(std::ostream &out) const
{
  rosban_utils::xml_tools::write<int>("nb_runs", nb_runs, out);
  rosban_utils::xml_tools::write<int>("nb_steps", nb_steps, out);
  rosban_utils::xml_tools::write<std::string>("problem", problem, out);
  mre_config.write("mre", out);
}

void MREMachine::Config::from_xml(TiXmlNode *node)
{
  nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
  nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
  problem  = rosban_utils::xml_tools::read<std::string>(node, "problem");
  mre_config.read(node, "mre");
}

MREMachine::MREMachine(std::shared_ptr<Config> config_)
  : config(config_), run(1), step(0)
{
  // Generating problem
  problem = std::shared_ptr<Problem>(ProblemFactory().build(config->problem));
  // Applying problem Limits
  config->mre_config.mrefpf_conf.setStateLimits(problem->getStateLimits());
  config->mre_config.mrefpf_conf.setActionLimits(problem->getActionLimits());
  // Initalizing mre
  mre = std::unique_ptr<MRE>(new MRE(config->mre_config,
                                     [this](const Eigen::VectorXd &state)
  {
    return this->problem->isTerminal(state);
  }
                               ));
  // Opening log files
  run_logs.open("run_logs.csv");
  time_logs.open("time_logs.csv");
  reward_logs.open("reward_logs.csv");
}

MREMachine::~MREMachine()
{
  run_logs.close();
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
  while (alive() && step <= config->nb_steps && !problem->isTerminal(current_state))
  {
    doStep();
    step++;
  }
  writeTimeLog("simulation", Benchmark::close());
  endRun();
}

void MREMachine::doStep()
{
  Eigen::VectorXd cmd = mre->getAction(current_state);
  Eigen::VectorXd last_state = current_state;
  applyAction(cmd);
  writeRunLog(run_logs, run, step, last_state, cmd, current_reward);
  csa_mdp::Sample new_sample(last_state,
                             cmd,
                             current_state,
                             current_reward);
  // Apply modifications
  mre->feed(new_sample);
  trajectory_reward += current_reward;
}

void MREMachine::init()
{
  writeRunLogHeader(run_logs);
  time_logs << "run,type,time" << std::endl;
  reward_logs << "run,reward" << std::endl;
}

bool MREMachine::alive()
{
  return true;
}

void MREMachine::prepareRun()
{
  step = 0;
  trajectory_reward = 0;
  current_reward = 0;
}

void MREMachine::endRun()
{
  // If the maximal step has not been reached, it mean we reached a final state
  if (step <= config->nb_steps)
  {
    int u_dim = problem->getActionLimits().rows();
    writeRunLog(run_logs, run, step, current_state, Eigen::VectorXd::Zero(u_dim), current_reward);
  }
  reward_logs << run << "," << trajectory_reward << std::endl;
  mre->updatePolicy();
  writeTimeLog("qValue", mre->getQValueTime());
  writeTimeLog("policy", mre->getPolicyTime());
}

void MREMachine::writeRunLogHeader(std::ostream &out)
{
  out << "run,step,";
  // State information
  int x_dim = config->mre_config.mrefpf_conf.getStateLimits().rows();
  for (int i = 0; i < x_dim; i++)
  {
    out << "state_" << i << ",";
  }
  // Commands
  int u_dim = config->mre_config.mrefpf_conf.getActionLimits().rows();
  for (int i = 0; i < u_dim; i++)
  {
    out << "action_" << i << ",";
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
