#include "problems/multi_kick_single_player.h"

#include "kick_model/kick_model_factory.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_random/tools.h"
#include "rosban_utils/xml_tools.h"

using namespace rosban_utils;

static bool debug_failures = false;

static double normalizeAngle(double angle)
{
  return angle - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

namespace csa_mdp
{

void MultiKickSinglePlayer::KickOption::to_xml(std::ostream & out) const {
  (void) out;
  //TODO
  throw std::logic_error("MultiKickSinglePlayer::KickOption::to_xml: not implemented");
}

void MultiKickSinglePlayer::KickOption::from_xml(TiXmlNode * node)
{
  kick_model = KickModelFactory().read(node, "kick_model");
  approach_model.from_xml(node);
  approach_policy = PolicyFactory().read(node, "policy");
}

std::string MultiKickSinglePlayer::KickOption::class_name() const
{
  return "multi_kick_single_player";
}

MultiKickSinglePlayer::MultiKickSinglePlayer()
  : step_initial_noise(0.25),
    goal_reward(0),
    approach_step_reward(-1),
    failure_reward(-500),
    field_width(6),
    field_length(9),
    goal_width(2.6),
    goal_area_size_x(1),
    goal_area_size_y(5),
    goalkeeper_success_rate(0.05)
{
  /// Updating limits
  Eigen::MatrixXd state_limits(5,2), action_limits(2,2);
  state_limits <<
    -field_length / 2, field_length / 2,
    -field_width / 2, field_width / 2,
    -field_length / 2, field_length / 2,
    -field_width / 2, field_width / 2,
    - M_PI, M_PI;
  setStateLimits(state_limits);
  setStateNames({"ball_x", "ball_y", "robot_x", "robot_y", "robot_theta"});
}

Problem::Result MultiKickSinglePlayer::getSuccessor(const Eigen::VectorXd & state,
                                                    const Eigen::VectorXd & action,
                                                    std::default_random_engine * engine) const
{
  // Checking state dimension
  if (state.rows() != 5) {
    std::ostringstream oss;
    oss << "MultiKickSinglePlayer::getSuccessor: invalid dimension for state: "
        << state.rows() << " (expected 5)";
    throw std::logic_error(oss.str());
  }
  // Choosing Kick Option:
  int action_id = (int)action(0);
  if (action_id < 0 || action_id >= (int)kick_options.size()) {
    std::ostringstream oss;
    oss << "MultiKickSinglePlayer::getSuccessor: invalid value for action_id: "
        << action(0) << " (expecting value in [" << 0 << "," << kick_options.size() << "[)";
    throw std::logic_error(oss.str());
  }
  const KickOption & kick_option = *(kick_options[action_id]);
  // Checking action dimension (depending on kick_option
  int expected_action_dimension = 1 + kick_option.kick_model->getActionsLimits().rows();
  if (action.rows() != expected_action_dimension) {
    std::ostringstream oss;
    oss << "MultiKickSinglePlayer::getSuccessor: invalid dimension for action: "
        << action.rows() << " (expected " << expected_action_dimension << ")";
    throw std::logic_error(oss.str());
  }
  // Initializing result properties
  Problem::Result result;
  result.successor = state;
  result.reward = 0;
  result.terminal = false;
  // Importing the parameters of the state
  double ball_x = state(0);
  double ball_y = state(1);
  // Importing the parameters of the action
  double kick_theta = action(1);
  double kick_power = action(2);
  // T1: Adding noise to get ball_real
  double ball_real_x, ball_real_y;
  initialBallNoise(ball_x, ball_y, &ball_real_x, &ball_real_y, engine);
  // T2: Handle cases where ball was outside of the field
  if (std::fabs(ball_real_x) > field_length / 2 ||
      std::fabs(ball_real_y) > field_width / 2)
  {
    // Update ball position and force it to be terminal
    result.successor(0) = ball_real_x;
    result.successor(1) = ball_real_y;
    result.terminal = true;
    // Has a goal been scored?
    if (isGoal(ball_x, ball_y, ball_real_x, ball_real_y))
      result.reward += goal_reward;
    else
      result.reward += failure_reward;
    // No need to perform additional computations
    return result;
  }
  // T3: Simulate player approach (end approach if the robot commited illegal attack)
  performApproach(kick_option, &result, action(1), engine);
  if (result.terminal) {
    return result;
  }
  // T4: Apply kick with noise
  double ball_final_x, ball_final_y;
  double kick_reward;
  kick_option.kick_model->applyKick(ball_real_x, ball_real_y,
                                    action.segment(1, action.rows()-1), engine,
                                    &ball_final_x, &ball_final_y, &kick_reward);
  result.reward += kick_reward;
  // T5: Testing if ball left the field after kick
  if (std::fabs(ball_final_x) > field_length / 2 ||
      std::fabs(ball_final_y) > field_width / 2)
  {
    // State is terminal
    result.terminal = true;
    // Has a goal been scored?
    if (isGoal(ball_real_x, ball_real_y, ball_final_x, ball_final_y)) {
      result.reward += goal_reward;
    }
    else {
      result.reward += failure_reward;
      if (debug_failures)
      {
        std::cout << "---------------------------" << std::endl;
        std::cout << "Failed to score a goal:" << std::endl;
        std::cout << "\tball_x: " << ball_x << std::endl;
        std::cout << "\tball_y: " << ball_y << std::endl;
        std::cout << "\tball_real_x: " << ball_real_x << std::endl;
        std::cout << "\tball_real_y: " << ball_real_y << std::endl;
        std::cout << "\tball_final_x: " << ball_final_x << std::endl;
        std::cout << "\tball_final_y: " << ball_final_y << std::endl;
        std::cout << "\tkick_power: " << kick_power << std::endl;
        std::cout << "\tkick_theta: " << kick_theta << std::endl;
      }
    }
  }  
  return result;
}

Eigen::VectorXd MultiKickSinglePlayer::getStartingState(std::default_random_engine * engine) const
{
  Eigen::MatrixXd limits(3,2);
  limits <<
    -field_length/2 + step_initial_noise, field_length/2 - step_initial_noise,
    -field_width/2 + step_initial_noise, field_width/2 - step_initial_noise,
    -M_PI, M_PI; 
  std::vector<Eigen::VectorXd> random_positions;
  random_positions = rosban_random::getUniformSamples(limits, 2, engine);
  Eigen::VectorXd state(5);
  state.segment(0,2) = random_positions[0].segment(0,2);// Ball pos
  state.segment(2,3) = random_positions[1];// Player pos
  return state;
}

void MultiKickSinglePlayer::performApproach(const KickOption & kick_option,
                                            Problem::Result * status,
                                            double kick_theta_field,
                                            std::default_random_engine * engine) const
{
  // Explicit names for ball position
  double ball_x = status->successor(0);
  double ball_y = status->successor(1);
  // Computing the status in the polar_approach problem
  Problem::Result pa_status;
  pa_status.successor = toPolarApproachState(status->successor, kick_theta_field);
  pa_status.reward = 0;
  pa_status.terminal = false;
  // Simulating steps until destination has been reached
  int nb_steps = 0;
  while (!kick_option.approach_model.isKickable(pa_status.successor)
         && !status->terminal)
  {
    Eigen::VectorXd action = kick_option.approach_policy->getAction(pa_status.successor);
    pa_status = kick_option.approach_model.getSuccessor(pa_status.successor, action, engine);
    // If robot is inside of the dead zone, each step has a failure risk
    Eigen::VectorXd curr_opk_state = toMultiKickSinglePlayerState(pa_status.successor,
                                                          ball_x, ball_y,
                                                          kick_theta_field);
    if (isGoalArea(curr_opk_state(2), curr_opk_state(3))) {
      std::uniform_real_distribution<double> failure_distrib(0,1);
      if (failure_distrib(*engine) < goalkeeper_success_rate)
      {
        status->reward += failure_reward;
        status->terminal = true;
      }
    }
    nb_steps++;
    int max_steps = 1000;
    if (nb_steps > max_steps) {
      std::ostringstream oss;
      oss << "MultiKickSinglePlayer::performApproach: kickable state not reached after "
          << max_steps << std::endl
          << "state: " << pa_status.successor.transpose() << std::endl;
      throw std::runtime_error(oss.str());
    }
  }
  // TODO: treat collisions between robot and ball
  // Update reward and final state
  status->successor = toMultiKickSinglePlayerState(pa_status.successor,
                                                   ball_x, ball_y,
                                                   kick_theta_field);
  status->reward += nb_steps * approach_step_reward;  
}

Eigen::VectorXd MultiKickSinglePlayer::toPolarApproachState(const Eigen::VectorXd & opk_state,
                                                            double kick_theta_field) const
{
  // Computing basic properties
  double ball_x_field = opk_state(0);
  double ball_y_field = opk_state(1);
  double player_x_field = opk_state(2);
  double player_y_field = opk_state(3);
  double player_theta   = opk_state(4);
  // Computing position in robot referential
  double dx = ball_x_field - player_x_field;
  double dy = ball_y_field - player_y_field;
  double ball_dist = std::sqrt(dx * dx + dy * dy);
  double ball_dir_field = atan2(dy, dx);
  double ball_dir_robot = normalizeAngle(ball_dir_field - player_theta);
  double kick_theta_robot = normalizeAngle(kick_theta_field - player_theta);
  // Building the input for the approach problem (initial speed is 0)
  Eigen::VectorXd pa_state = Eigen::VectorXd::Zero(6);
  pa_state(0) = ball_dist;
  pa_state(1) = ball_dir_robot;
  pa_state(2) = kick_theta_robot;
  return pa_state;
}


Eigen::VectorXd
MultiKickSinglePlayer::toMultiKickSinglePlayerState(const Eigen::VectorXd & pa_state,
                                                    double ball_x,
                                                    double ball_y,
                                                    double kick_theta_field) const
{
  // Getting basic data
  double ball_dist = pa_state(0);
  double ball_dir_robot = pa_state(1);
  double target_angle = pa_state(2);
  // Computing player orientation
  double robot_theta_field = normalizeAngle(kick_theta_field - target_angle);
  double robot2ball_theta_field = normalizeAngle(ball_dir_robot + robot_theta_field);
  // Filling state
  Eigen::VectorXd opk_state(5);
  opk_state(0) = ball_x;
  opk_state(1) = ball_y;
  opk_state(2) = ball_x - ball_dist * cos(robot2ball_theta_field);
  opk_state(3) = ball_y - ball_dist * sin(robot2ball_theta_field);
  opk_state(4) = robot_theta_field;
  return opk_state;
}

void MultiKickSinglePlayer::to_xml(std::ostream & out) const
{
  (void) out;
  throw std::logic_error("MultiKickSinglePlayer::to_xml: unimplemented");
}

void MultiKickSinglePlayer::from_xml(TiXmlNode * node)
{
  xml_tools::try_read<double>(node, "step_initial_noise"     , step_initial_noise     );
  xml_tools::try_read<double>(node, "goal_reward"            , goal_reward            );
  xml_tools::try_read<double>(node, "approach_step_reward"   , approach_step_reward   );
  xml_tools::try_read<double>(node, "failure_reward"         , failure_reward         );
  xml_tools::try_read<double>(node, "field_width"            , field_width            );
  xml_tools::try_read<double>(node, "field_length"           , field_length           );
  xml_tools::try_read<double>(node, "goal_width"             , goal_width             );
  xml_tools::try_read<double>(node, "goal_area_size_x"       , goal_area_size_x       );
  xml_tools::try_read<double>(node, "goal_area_size_y"       , goal_area_size_y       );
  xml_tools::try_read<double>(node, "goalkeeper_success_rate", goalkeeper_success_rate);
  TiXmlNode* values = node->FirstChild("kick_options");
  kick_options.clear();
  for ( TiXmlNode* child = values->FirstChild(); child != NULL; child = child->NextSibling())
  {
    std::unique_ptr<KickOption> ko(new KickOption);
    ko->from_xml(child);
    kick_options.push_back(std::move(ko));
  }
  updateApproachesLimits();
  updateActionsLimits();
}

std::string MultiKickSinglePlayer::class_name() const
{
  return "multi_kick_single_player";
}

void MultiKickSinglePlayer::updateApproachesLimits()
{
  for (size_t ko_id = 0; ko_id < kick_options.size(); ko_id++) {
    /// Ball can be a lot further than in usual approach_model
    kick_options[ko_id]->approach_model.setMaxDist(field_width + field_length);//Extra margin is not an issue
  }
}

void MultiKickSinglePlayer::updateActionsLimits()
{
  std::vector<Eigen::MatrixXd> kick_action_limits;
  std::vector<std::vector<std::string>> kick_action_names;
  for (size_t ko_id = 0; ko_id < kick_options.size(); ko_id++) {
    // Updating the action limits for the policies
    std::vector<Eigen::MatrixXd> actions_limits;
    actions_limits = kick_options[ko_id]->approach_model.getActionsLimits();
    kick_options[ko_id]->approach_policy->setActionLimits(actions_limits);
    // Filling action spaces and names for this problem
    kick_action_limits.push_back(kick_options[ko_id]->kick_model->getActionsLimits());
    kick_action_names.push_back(kick_options[ko_id]->kick_model->getActionsNames());
  }
  setActionLimits(kick_action_limits);
  setActionsNames(kick_action_names);
}

void MultiKickSinglePlayer::initialBallNoise(double ball_x, double ball_y,
                                             double * ball_real_x, double * ball_real_y,
                                             std::default_random_engine * engine) const
{
  std::uniform_real_distribution<double> noise_distrib(0, step_initial_noise);
  double dist = 2 * step_initial_noise;
  double dx(0), dy(0);
  while (dist > step_initial_noise)
  {
    dx = noise_distrib(*engine);
    dy = noise_distrib(*engine);
    dist = dx * dx + dy * dy;
  }
  *ball_real_x = ball_x + dx;
  *ball_real_y = ball_y + dy;
}

bool MultiKickSinglePlayer::isGoal(double src_x, double src_y, double dst_x, double dst_y) const
{
  double dx = dst_x - src_x;
  if (debug_failures)
  {
    std::cout << "dx: " << dx << std::endl;
  }
  // Impossible to score a goal by kicking backward or if ball did not cross the line
  if (dx <= 0 || dst_x < field_length / 2) return false;
  // Finding equation a*x +b;
  double dy = dst_y - src_y;
  double a = dy / dx;
  double b = src_y - a * src_x;
  // Finding intersection with goal line
  double intercept_y = a * field_length / 2 + b;
  if (debug_failures)
  {
    std::cout << "dy: " << dy << std::endl;
    std::cout << "a : " << a << std::endl;
    std::cout << "b : " << b << std::endl;
    std::cout << "y : " << intercept_y << std::endl;
  }
  // Is intersection inside the goal
  return std::fabs(intercept_y) < goal_width / 2;
}

bool MultiKickSinglePlayer::isGoalArea(double player_x, double player_y) const
{
  bool xOk = player_x > (field_length/2 - goal_area_size_x) && player_x < field_length/2;
  bool yOk = std::fabs(player_y) < (field_width - goal_area_size_y) / 2;
  return xOk && yOk;
}

}
