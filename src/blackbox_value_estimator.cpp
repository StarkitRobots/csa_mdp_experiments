#include "starkit_csa_mdp/solvers/black_box_learner_factory.h"

#include "policies/expert_approach.h"
#include "policies/mixed_approach.h"
#include "policies/ok_seed.h"
#include "problems/extended_problem_factory.h"

#include "starkit_csa_mdp/core/policy_factory.h"
#include "starkit_fa/trainer_factory.h"
#include "starkit_random/tools.h"
#include "starkit_utils/threading/multi_core.h"

#include <fenv.h>

namespace csa_mdp
{
class BlackboxValueEstimator : public starkit_utils::JsonSerializable
{
public:
  /// Dummy constructor
  BlackboxValueEstimator() : nb_samples(1000), evals_per_sample(1), nb_threads(1), horizon(100), discount(1.0)
  {
  }

  /// Generate inputs and observations according to internal parameters
  void generateSamples(Eigen::MatrixXd& inputs, Eigen::MatrixXd& observations, std::default_random_engine* engine) const
  {
    int input_dims = problem->stateDims();
    /// Initializing local variables
    inputs = Eigen::MatrixXd::Zero(input_dims, nb_samples);
    observations = Eigen::MatrixXd::Zero(nb_samples, 1);
    std::vector<std::default_random_engine> engines =
        starkit_random::getRandomEngines(std::min(nb_threads, nb_samples), engine);
    // The task which has to be performed :
    starkit_utils::MultiCore::StochasticTask task = [this, &inputs, &observations,
                                                    input_dims](int start_idx, int end_idx,
                                                                std::default_random_engine* thread_engine) {
      // Access to some variables
      int thread_samples = end_idx - start_idx;
      const Eigen::MatrixXd& limits = problem->getStateLimits();
      // Sampling initial states
      inputs.block(0, start_idx, input_dims, thread_samples) =
          starkit_random::getUniformSamplesMatrix(limits, thread_samples, thread_engine);
      for (int idx = start_idx; idx < end_idx; idx++)
      {
        const Eigen::VectorXd& state = inputs.col(idx);
        double total_reward = 0;
        for (int eval = 0; eval < evals_per_sample; eval++)
        {
          double eval_reward = problem->sampleRolloutReward(state, *policy, horizon, discount, thread_engine);
          total_reward += eval_reward;
        }
        observations(idx, 0) = total_reward / evals_per_sample;
      }
    };
    // Running computation
    starkit_utils::MultiCore::runParallelStochasticTask(task, nb_samples, &engines);
  }

  std::unique_ptr<starkit_fa::FunctionApproximator> trainApproximator(std::default_random_engine* engine) const
  {
    Eigen::MatrixXd inputs, observations;
    generateSamples(inputs, observations, engine);
    return approximator->train(inputs, observations, problem->getStateLimits());
  }

  Json::Value toJson() const override
  {
    throw std::logic_error("BlackboxValueEstimator::toJson: not implemented");
  }

  void fromJson(const Json::Value& v, const std::string& dir_name) override
  {
    // Reading basic properties
    starkit_utils::tryRead(v, "nb_samples", &nb_samples);
    starkit_utils::tryRead(v, "evals_per_sample", &evals_per_sample);
    starkit_utils::tryRead(v, "nb_threads", &nb_threads);
    starkit_utils::tryRead(v, "horizon", &horizon);
    starkit_utils::tryRead(v, "discount", &discount);
    // Getting problem (mandatory)
    std::shared_ptr<const Problem> tmp_problem;
    std::string problem_path;
    starkit_utils::tryRead(v, "problem_path", &problem_path);
    if (problem_path != "")
    {
      tmp_problem = ProblemFactory().buildFromJsonFile(dir_name + problem_path);
    }
    else
    {
      tmp_problem = ProblemFactory().read(v, "problem", dir_name);
    }
    problem = std::dynamic_pointer_cast<const BlackBoxProblem>(tmp_problem);
    if (!problem)
    {
      throw std::runtime_error("BlackBoxLearner::fromJson: problem is not a BlackBoxProblem");
    }
    // Reading approximator (mandatory)
    approximator = starkit_fa::TrainerFactory().read(v, "approximator", dir_name);
    // Reading policy (mandatory)
    policy = PolicyFactory().read(v, "policy", dir_name);
    // Updating the number of threads to use to build the approximator
    approximator->setNbThreads(nb_threads);
    // updating action limits for policy
    policy->setActionLimits(problem->getActionsLimits());
  }

  std::string getClassName() const override
  {
    return "BlackboxValueEstimator";
  }

private:
  /// Blackbox problem
  std::shared_ptr<const BlackBoxProblem> problem;

  /// Approximator used to train function approximator
  std::unique_ptr<starkit_fa::Trainer> approximator;

  /// The policy used to navigate
  std::unique_ptr<Policy> policy;

  /// Number of samples used for approximation
  int nb_samples;

  /// Number of evaluations used to approximate the average reward for each
  /// sample. Allow to control the signal to noise ratio.
  int evals_per_sample;

  /// Number of threads used for both:
  /// - Evaluating samples
  /// - Training approximator
  int nb_threads;

  /// Until which maximal horizon rollouts are performed?
  int horizon;

  /// Discount used for rollouts
  double discount;
};

}  // namespace csa_mdp

using namespace csa_mdp;

int main()
{
  // Abort if error are found
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  PolicyFactory::registerExtraBuilder("expert_approach", []() { return std::unique_ptr<Policy>(new ExpertApproach); });
  PolicyFactory::registerExtraBuilder("OKSeed", []() { return std::unique_ptr<Policy>(new OKSeed); });
  PolicyFactory::registerExtraBuilder("mixed_approach", []() { return std::unique_ptr<Policy>(new MixedApproach); });

  ExtendedProblemFactory::registerExtraProblems();

  BlackboxValueEstimator estimator;

  estimator.loadFile();

  std::default_random_engine engine = starkit_random::getRandomEngine();

  std::unique_ptr<starkit_fa::FunctionApproximator> approximator;
  approximator = estimator.trainApproximator(&engine);

  approximator->save("approximated_value.bin");

  exit(EXIT_SUCCESS);
}
