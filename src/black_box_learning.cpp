#include "rosban_csa_mdp/solvers/black_box_learner_factory.h"

#include "problems/extended_problem_factory.h"

#include "rosban_random/tools.h"

using namespace csa_mdp;

int main()
{
  ExtendedProblemFactory::registerExtraProblems();

  std::shared_ptr<BlackBoxLearner> bbl;
  bbl = BlackBoxLearnerFactory().buildFromXmlFile("BlackBoxLearner.xml", "BlackBoxLearner");

  std::default_random_engine engine = rosban_random::getRandomEngine();

  bbl->run(&engine);
}
