#include "learning_machine/learning_machine_factory.h"
#include "problems/extended_problem_factory.h"
#include "policies/expert_approach.h"
#include "policies/mixed_approach.h"
#include "policies/ok_seed.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#ifndef NO_ROSBAN_CONTROL
#include "learning_machine/learning_machine_controller.h"
#include <ros/ros.h>
#endif

using namespace csa_mdp;

int main()
{
  // Registering extra features from csa_mdp
  PolicyFactory::registerExtraBuilder("expert_approach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  PolicyFactory::registerExtraBuilder("mixed_approach",
                                      []() {return std::unique_ptr<Policy>(new MixedApproach);});
  PolicyFactory::registerExtraBuilder("OKSeed",
                                      []() {return std::unique_ptr<Policy>(new OKSeed);});

  ExtendedProblemFactory::registerExtraProblems();

  // Loading the learning Machine
  LearningMachineFactory lmf;
  std::shared_ptr<LearningMachine> lm;
  lm = lmf.buildFromJsonFile("LearningMachine.xml");

#ifndef NO_ROSBAN_CONTROL
  // If we need a controller, init ros link
  if (std::dynamic_pointer_cast<LearningMachineController>(lm))
  {
    ros::init(argc, argv, "mre_controller");
    ros::NodeHandle nh;
  }
#endif


  // Runnig the process
  lm->execute();
}
