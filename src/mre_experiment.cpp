#include "mre_machine/mre_machine_controller.h"
#include "mre_machine/mre_machine_factory.h"

#include "policies/expert_approach.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#include <ros/ros.h>

#include <iostream>

void usage()
{
  std::cerr << "Usage: ... <config_path>" << std::endl;
  exit(EXIT_FAILURE);
}

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    usage();
  }

  PolicyFactory::registerExtraBuilder("expert_approach",[](TiXmlNode * node)
                                      { (void)node; return new ExpertApproach();});

  std::string config_path(argv[1]);

  // Going to the specified path
  if (chdir(config_path.c_str()))
  {
    std::cerr << "Failed to set '" << config_path << "' as working directory." << std::endl;
    exit(EXIT_FAILURE);
  }

  MREMachineFactory f;
  MREMachine * mre_machine = f.buildFromXmlFile("mre_experiment.xml", "mre_experiment");

  if (dynamic_cast<MREMachineController *>(mre_machine) != NULL)
  {
    ros::init(argc, argv, "mre_controller");
    ros::NodeHandle nh;
  }

  mre_machine->execute();

  delete(mre_machine);
}
