#include "rosban_csa_mdp/solvers/black_box_learner_factory.h"

#include "policies/expert_approach.h"
#include "policies/mixed_approach.h"
#include "policies/opk_expert_approach.h"
#include "policies/ok_seed.h"
#include "problems/extended_problem_factory.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_random/tools.h"

#include <fenv.h>

using namespace csa_mdp;


int main() {
  // Abort if error are found
  feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);


  PolicyFactory::registerExtraBuilder("expert_approach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  PolicyFactory::registerExtraBuilder("OKSeed",
                                      []() {return std::unique_ptr<Policy>(new OKSeed);});
// TODO: fix custom policies
//  PolicyFactory::registerExtraBuilder("mixed_approach",
//                                      []() {return std::unique_ptr<Policy>(new MixedApproach);});
//  PolicyFactory::registerExtraBuilder("opk_expert_approach",
//                                      []() {return std::unique_ptr<Policy>(new OPKExpertApproach);});

  ExtendedProblemFactory::registerExtraProblems();

  std::shared_ptr<BlackBoxLearner> bbl;
  bbl = BlackBoxLearnerFactory().buildFromXmlFile("BlackBoxLearner.xml", "BlackBoxLearner");

  std::default_random_engine engine = rosban_random::getRandomEngine();

  bbl->run(&engine);
}
