#include "rosban_csa_mdp/solvers/black_box_learner_factory.h"

#include "policies/expert_approach.h"
#include "policies/mixed_approach.h"
#include "policies/ok_seed.h"
#include "problems/extended_problem_factory.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_random/tools.h"

#include <fenv.h>

using namespace csa_mdp;


int main(int argc, char ** argv) {
  std::string learner_path("black_box_learner.json");
  if (argc >= 2) {
    learner_path = argv[1];
  }

  // Abort if error are found
  feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);


  PolicyFactory::registerExtraBuilder("ExpertApproach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  PolicyFactory::registerExtraBuilder("OKSeed",
                                      []() {return std::unique_ptr<Policy>(new OKSeed);});
  PolicyFactory::registerExtraBuilder("MixedApproach",
                                      []() {return std::unique_ptr<Policy>(new MixedApproach);});

  ExtendedProblemFactory::registerExtraProblems();

  std::shared_ptr<BlackBoxLearner> bbl;
  bbl = BlackBoxLearnerFactory().buildFromJsonFile(learner_path);

  std::default_random_engine engine = rosban_random::getRandomEngine();

  bbl->run(&engine);
}
