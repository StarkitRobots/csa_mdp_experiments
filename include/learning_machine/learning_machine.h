#pragma once

#include "starkit_csa_mdp/solvers/learner.h"

#include "starkit_csa_mdp/core/policy.h"
#include "starkit_csa_mdp/core/problem.h"

#include <fstream>
#include <memory>

namespace csa_mdp
{
/// Base class for running experiments
class LearningMachine : public starkit_utils::JsonSerializable
{
public:
  // TODO replace update rule by two int runs_by_policy and runs_by_policy_growth
  /// What is the frequency of update?
  /// - each  : Update after every run
  /// - square: The number of run before each update is the number of update
  enum class UpdateRule
  {
    each,
    square
  };

  LearningMachine();
  virtual ~LearningMachine();

  /// Also reset the learning dimensions to problem default and propagate
  virtual void setProblem(std::unique_ptr<csa_mdp::Problem> problem);
  /// Also propagate
  void setLearner(std::unique_ptr<csa_mdp::Learner> learner);
  /// Also propagate
  void setLearningDimensions(const std::vector<int>& learning_dimensions);
  // Also propagate
  void setDiscount(double new_discount);
  /// Inform the learner of current limits and discount
  void propagate();

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
  virtual void applyAction(const Eigen::VectorXd& action) = 0;

  /// Open the streams with respect to the configuration
  virtual void openStreams();

  /// Close all the opened streams
  virtual void closeActiveStreams();

  void writeRunLogHeader(std::ostream& out);

  void writeTimeLog(const std::string& type, double time);

  void writeRunLog(std::ostream& out, int run, int step, const Eigen::VectorXd& state, const Eigen::VectorXd& action,
                   double reward);

  /// Check if detail folder exists and if not, then create it
  void createDetailFolder() const;

  /// Get the learning space from the given full space
  Eigen::MatrixXd getLearningSpace(const Eigen::MatrixXd& space);

  /// Get the learning state from the given full state
  Eigen::VectorXd getLearningState(const Eigen::VectorXd& state);

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

protected:
  /// The online explorator
  std::unique_ptr<csa_mdp::Learner> learner;

  /// The problem being solved
  std::shared_ptr<csa_mdp::Problem> problem;

  // Current status
  int run;
  int step;
  double trajectory_reward;
  double trajectory_disc_reward;
  Problem::Result status;

  /// The discount gain used
  double discount;
  /// The number of threads allowed
  int nb_threads;

  /// The current policy number
  int policy_id;
  /// Nb runs required for the given policy
  int policy_runs_required;
  /// Nb runs performed by the current policy
  int policy_runs_performed;
  /// Total reward gathered by the current policy
  double policy_total_reward;
  /// Policy score: average reward per trial
  double best_policy_score;

  /// Frequency of update for the internal structure
  UpdateRule update_rule;
  /// Maximal number of runs
  int nb_runs;
  /// Maximal number of steps per run
  int nb_steps;
  /// How many time can the learning machine use? [s]
  /// (default is unlimited)
  double time_budget;

  /// Are the policy / q_value saved after each policy update?
  bool save_details;
  /// Are the run content saved?
  bool save_run_logs;
  /// Is the best policy saved?
  bool save_best_policy;

  // Output files
  std::ofstream run_logs;
  std::ofstream time_logs;
  std::ofstream reward_logs;

  /// Which dimensions of the state space are used as input for learning
  std::vector<int> learning_dimensions;

  /// When using exploration mode, a 'seed' can be provided which is a file containing
  /// one or several runs which can be used to learn a first policy
  std::string seed_path;

  /// Path at which details are saved
  static std::string details_path;

  LearningMachine::UpdateRule loadUpdateRule(const std::string& rule);
};

std::string to_string(LearningMachine::UpdateRule rule);

}  // namespace csa_mdp
