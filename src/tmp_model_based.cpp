#include "learning_machine/learning_machine_blackbox.h"
#include "problems/extended_problem_factory.h"
#include "policies/expert_approach.h"

#include "rosban_csa_mdp/core/policy_factory.h"

using csa_mdp::PolicyFactory;

int main()
{
  // Registering extra features from csa_mdp
  PolicyFactory::registerExtraBuilder("expert_approach",[](TiXmlNode * node)
                                      { (void)node; return new ExpertApproach();});
  ExtendedProblemFactory::registerExtraProblems();

  // Loading the learning Machine
  LearningMachineBlackBox lmb;
  lmb.load_file();

  // Runnig the process
  lmb.execute();
}
