#include "learning_machine/learning_machine_factory.h"
#include "problems/extended_problem_factory.h"
#include "policies/expert_approach.h"
#include "policies/mixed_approach.h"
#include "policies/ok_seed.h"

#include "rhoban_csa_mdp/core/policy_factory.h"

using namespace csa_mdp;

int main(int argc, char ** argv)
{
  std::string learner_path("learning_machine.json");
  if (argc >= 2) {
    learner_path = argv[1];
  }

  // Registering extra features from csa_mdp
  PolicyFactory::registerExtraBuilder("ExpertApproach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  PolicyFactory::registerExtraBuilder("MixedApproach",
                                      []() {return std::unique_ptr<Policy>(new MixedApproach);});
  PolicyFactory::registerExtraBuilder("OKSeed",
                                      []() {return std::unique_ptr<Policy>(new OKSeed);});

  ExtendedProblemFactory::registerExtraProblems();

  // Loading the learning Machine
  LearningMachineFactory lmf;
  std::shared_ptr<LearningMachine> lm;
  lm = lmf.buildFromJsonFile(learner_path);

  // Runnig the process
  lm->execute();
}
