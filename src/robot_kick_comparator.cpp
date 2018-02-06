#include "policies/expert_approach.h"
#include "policies/ok_seed.h"

#include "problems/kick_controler.h"

#include "rhoban_csa_mdp/core/policy_factory.h"
#include "rhoban_random/tools.h"

#include <fenv.h>

using namespace csa_mdp;

int xSteps = 10;
int ySteps = 10;
int thetaSteps = 10;

int nb_evaluations = 10000;
int horizon = 100;

int main()
{
  // Abort if error are found
  feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);

  PolicyFactory::registerExtraBuilder("expert_approach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  PolicyFactory::registerExtraBuilder("OKSeed",
                                      []() {return std::unique_ptr<Policy>(new OKSeed);});

  KickControler problem_1p;
  KickControler problem_2p;

  problem_1p.loadFile("Problem1P.xml");
  problem_2p.loadFile("Problem2P.xml");

  std::unique_ptr<Policy> policy_1p = PolicyFactory().buildFromJsonFile("Policy1P.xml");
  std::unique_ptr<Policy> policy_2p = PolicyFactory().buildFromJsonFile("Policy2P.xml");

  policy_1p->setActionLimits(problem_1p.getActionsLimits());
  policy_2p->setActionLimits(problem_2p.getActionsLimits());

  Eigen::Vector2d ball_pos(1.8,-2.5);
  Eigen::Vector3d p1_pos(1.3,-2.5, 0);

  Eigen::MatrixXd state_space = problem_2p.getStateLimits();

  Eigen::Matrix<double,3,2> space_p2 = state_space.block(5,0,3,2);

  Eigen::Vector3d nbSteps(xSteps,ySteps,thetaSteps);

  Eigen::Vector3d steps = (space_p2.col(1) - space_p2.col(0)).cwiseQuotient(nbSteps);

  Eigen::Vector3d start = space_p2.col(0) + steps / 2;

  Eigen::Vector3d current = start;

  Eigen::VectorXd init_state_1p(5);
  Eigen::VectorXd init_state_2p(8);
  init_state_1p.segment(0,2) = ball_pos;
  init_state_1p.segment(0,3) = p1_pos;
  init_state_2p.segment(0,2) = ball_pos;
  init_state_2p.segment(5,3) = p1_pos;


  std::default_random_engine engine = rhoban_random::getRandomEngine();

  double reward_1p = 0;
  for (int i = 0; i < nb_evaluations; i++) {
    reward_1p += problem_1p.sampleRolloutReward(init_state_1p, *policy_1p,
                                                horizon, 1, &engine);
  }

  reward_1p /= nb_evaluations;

  std::cout << "x,y,dir,reward,gain" << std::endl;

  while (current(0) < space_p2(0,1)) {
    current(1) = start(1);
    while (current(1) < space_p2(1,1)) {
      current(2) = start(2);
      while (current(2) < space_p2(2,1)) {
        init_state_2p.segment(2,3) = current;
        double reward_2p = 0;
        for (int i = 0; i < nb_evaluations; i++) {
          reward_2p += problem_2p.sampleRolloutReward(init_state_2p, *policy_2p,
                                                      horizon, 1, &engine);
        }
        reward_2p /= nb_evaluations;

        double gain = reward_2p - reward_1p;

        std::cout << current(0) << "," << current(1) << "," << current(2) << "," << reward_2p << "," << gain << std::endl;

        current(2) += steps(2);
      }
      current(1) += steps(1);
    }
    current(0) += steps(0);
  }

}
