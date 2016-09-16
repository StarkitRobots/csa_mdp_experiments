#include "learning_machine/learning_machine_factory.h"
#include "problems/extended_problem_factory.h"
#include "policies/expert_approach.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#ifndef NO_ROSBAN_CONTROL
#include "learning_machine/learning_machine_controller.h"
#include <ros/ros.h>
#endif

using csa_mdp::Policy;
using csa_mdp::PolicyFactory;

int main(int argc, char ** argv)
{
  // Registering extra features from csa_mdp
  PolicyFactory::registerExtraBuilder("expert_approach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  ExtendedProblemFactory::registerExtraProblems();

  // Loading the learning Machine
  LearningMachineFactory lmf;
  std::shared_ptr<LearningMachine> lm;
  lm = lmf.buildFromXmlFile("LearningMachine.xml", "LearningMachine");

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
