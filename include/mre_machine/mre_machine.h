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

  class Config : rosban_utils::Serializable
  {
  public:
    Config();

    virtual std::string class_name() const override;
    void to_xml(std::ostream &out) const override;
    void from_xml(TiXmlNode *node) override;

    Mode mode;
    int nb_runs;
    int nb_steps;
    std::string problem;
    csa_mdp::MRE::Config mre_config;
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
  Eigen::VectorXd current_state;
  double current_reward;

  // Spacing updates
  int nb_updates;
  int next_update;
};

std::string to_string(MREMachine::Mode mode);
MREMachine::Mode loadMode(const std::string &mode);
