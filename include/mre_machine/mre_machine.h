#pragma once

#include "rosban_csa_mdp/solvers/mre.h"

#include "rosban_csa_mdp/core/policy.h"
#include "rosban_csa_mdp/core/problem.h"

#include <fstream>

/// Base class for running experiments
class MREMachine
{
public:

  /// There is three types of mode for a mre_machine
  /// 1. exploration (gather samples and update policy frequently)
  /// 2. evaluation (uses a provided policy for runs)
  /// 3. full (gather samples, then learn policies and finally perform an evaluation)
  enum class Mode
  { exploration, evaluation, full };

  enum class UpdateRule
  { each, square };

  class Config : rosban_utils::Serializable
  {
  public:
    Config();

    virtual std::string class_name() const override;
    void to_xml(std::ostream &out) const override;
    void from_xml(TiXmlNode *node) override;

    Mode mode;
    UpdateRule update_rule;
    int nb_runs;
    int nb_steps;

    /// Are the policy / q_value saved after each policy update?
    bool save_details;

    std::shared_ptr<csa_mdp::Problem> problem;
    csa_mdp::MRE::Config mre_config;
    /// When using exploration mode, a 'seed' can be provided which is a file containing
    /// one or several runs which can be used to learn a first policy 
    std::string seed_path;
    std::unique_ptr<csa_mdp::Policy> policy;
    //TODO add a boolean for choosing if details should be written or not
  };

  MREMachine(std::shared_ptr<Config> config);
  virtual ~MREMachine();

  /// If these function return false, the process ends as quickly as possible 
  virtual bool alive();

  /// Run the whole process
  void execute();

  /// Perform a single run
  void doRun();
  
  /// Perform a single step
  void doStep();

  /// Init the experiment
  virtual void init();

  /// Prepare next run
  virtual void prepareRun();

  /// Finish a run
  virtual void endRun();

  /// Apply the provided action and update current state and last reward
  /// This method should include a sleep if required
  virtual void applyAction(const Eigen::VectorXd &action) = 0;

  
  void writeRunLogHeader(std::ostream &out);

  void writeTimeLog(const std::string &type, double time);

  static void writeRunLog(std::ostream &out,
                          int run, int step,
                          const Eigen::VectorXd &state,
                          const Eigen::VectorXd &action,
                          double reward);

protected:
  std::shared_ptr<Config> config;
  std::shared_ptr<csa_mdp::Problem> problem;

  /// The online explorator
  std::unique_ptr<csa_mdp::MRE> mre;

  /// Output files
  std::ofstream run_logs;
  std::ofstream time_logs;
  std::ofstream reward_logs;

  // Current status
  int run;
  int step;
  double trajectory_reward;
  double trajectory_disc_reward;
  Eigen::VectorXd current_state;
  double current_reward;

  int policy_id;
  // Nb runs required for the given policy
  int policy_runs_required;
  // Nb runs performed by the current policy
  int policy_runs_performed;
  // Total reward gathered by the current policy
  double policy_total_reward;
  // Policy score: average reward per trial
  double best_policy_score;
};

std::string to_string(MREMachine::Mode mode);
MREMachine::Mode loadMode(const std::string &mode);

std::string to_string(MREMachine::UpdateRule rule);
MREMachine::UpdateRule loadUpdateRule(const std::string &rule);
