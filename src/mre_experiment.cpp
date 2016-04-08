#include "solvers/mre_machine_factory.h"

#include <iostream>

void usage(int argc, char ** argv)
{
  std::cerr << "Usage: ... <config_path" << std::endl;
  exit(EXIT_FAILURE);
}

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    usage();
  }

  ros::init(argc, argv, "mre_controller");

  std::string config_path(argv[1]);

  // Going to the specified path
  if (chdir(config_path.c_str()))
  {
    std::cerr << "Failed to set '" << config_path << "' as working directory." << std::endl;
    exit(EXIT_FAILURE);
  }

  MREMachineFactory f;
  MREMachine * mre_machine = f.build_from_xml_file(config_path);

  mre_machine->init();

}
