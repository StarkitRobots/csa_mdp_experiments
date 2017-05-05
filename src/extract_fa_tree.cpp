#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_fa/fa_tree.h"

#include "policies/expert_approach.h"
#include "policies/ok_seed.h"

#include <iostream>

using namespace csa_mdp;

using rosban_fa::FATree;

int main(int argc, char ** argv)
{
  PolicyFactory::registerExtraBuilder("expert_approach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  PolicyFactory::registerExtraBuilder("OKSeed",
                                      []() {return std::unique_ptr<Policy>(new OKSeed);});

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input_xml> <output_binary>" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string input_path(argv[1]);
  std::string output_path(argv[2]);

  std::unique_ptr<Policy> p = PolicyFactory().buildFromXmlFile(input_path, "policy");

  std::unique_ptr<FATree> tree = p->extractFATree();

  tree->save(output_path);


}
