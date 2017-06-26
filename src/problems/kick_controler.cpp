#include "problems/kick_controler.h"

#include "kick_model/kick_model_factory.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_random/tools.h"
#include "rosban_utils/xml_tools.h"

using namespace rosban_utils;

static double normalizeAngle(double angle)
{
  return angle - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

namespace csa_mdp
{

void KickControler::KickOption::to_xml(std::ostream & out) const {
  (void) out;
  //TODO
  throw std::logic_error("KickControler::KickOption::to_xml: not implemented");
}

void KickControler::KickOption::from_xml(TiXmlNode * node)
{
  kick_model = KickModelFactory().read(node, "kick_model");
  approach_model.from_xml(node->FirstChild("approach_model"));
  approach_policy = PolicyFactory().read(node, "policy");
}

std::string KickControler::KickOption::class_name() const
{
  return "kick_controler_kick_option";
}

void KickControler::Player::to_xml(std::ostream & out) const {
  (void) out;
  //TODO
  throw std::logic_error("KickControler::Player::to_xml: not implemented");
}

void KickControler::Player::from_xml(TiXmlNode * node)
{
  name = rosban_utils::xml_tools::read<std::string>(node, "name");
  TiXmlNode* values = node->FirstChild("kick_options");
  kick_options.clear();
  for ( TiXmlNode* child = values->FirstChild(); child != NULL; child = child->NextSibling())
  {
    std::unique_ptr<KickOption> ko(new KickOption);
    ko->from_xml(child);
    kick_options.push_back(std::move(ko));
  }
  navigation_approach.from_xml(node->FirstChild("approach_model"));
  approach_policy = PolicyFactory().read(node, "policy");
}

std::string KickControler::Player::class_name() const
{
  return "kick_controler_player";
}

KickControler::KickControler()
  : step_initial_stddev(0.25),
    goal_reward(0),
    approach_step_reward(-1),
    failure_reward(-500),
    field_width(6),
    field_length(9),
    goal_width(2.6),
    ball_radius(0.07),
    goal_area_size_x(1),
    goal_area_size_y(5),
    goalkeeper_success_rate(0.05),
    goalie_x(field_length/2 - 0.1),
    goalie_y(0),
    goalie_thickness(0.2),
    goalie_width(0.4)
{
}

Problem::Result KickControler::getSuccessor(const Eigen::VectorXd & state,
                                            const Eigen::VectorXd & action,
                                            std::default_random_engine * engine) const
{
  // Checking state dimension
  if (state.rows() != 2 + 3 * (int)players.size()) {
    std::ostringstream oss;
    oss << "KickControler::getSuccessor: invalid dimension for state: "
        << state.rows() << " (expected " << (2+3*players.size()) << ")";
    throw std::logic_error(oss.str());
  }
  // Getting player_id and kick_id:
  int action_id = (int)action(0);
  int kicker_id, kick_option_id;
  analyzeActionId(action_id, &kicker_id, &kick_option_id);
  const KickOption & kick_option = *(players[kicker_id]->kick_options[kick_option_id]);
  // Checking action dimension (depending on kick_option)
  int kick_dims = kick_option.kick_model->getActionsLimits().rows();
  int expected_action_dimension = 1 + kick_dims + 3 * (players.size()-1);
  if (action.rows() != expected_action_dimension) {
    std::ostringstream oss;
    oss << "KickControler::getSuccessor: invalid dimension for action: "
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
  // T1: Adding noise to get ball_real
  double ball_real_x, ball_real_y;
  initialBallNoise(ball_x, ball_y, &ball_real_x, &ball_real_y, engine);
  // T2: Move the ball to its real position
  bool early_terminal = false;
  moveBall(ball_x, ball_y, &ball_real_x, &ball_real_y, &early_terminal, &result.reward);
  if (early_terminal)
  {
    // Update ball position and force it to be terminal
    result.successor(0) = ball_real_x;
    result.successor(1) = ball_real_y;
    result.terminal = true;
    // No need to perform additional computations
    return result;
  }
  // T3: Simulate players approach (end approach if one of the robot commited illegal attack)
  int max_steps = 500;
  runSteps(max_steps, action, kicker_id, kick_option_id, true, &result, engine);
  if (result.terminal) {
    return result;
  }
  // T4: Apply kick with noise
  double ball_final_x, ball_final_y;
  double kick_reward;
  kick_option.kick_model->applyKick(ball_real_x, ball_real_y,
                                    action.segment(1, kick_dims), engine,
                                    &ball_final_x, &ball_final_y, &kick_reward);
  result.reward += kick_reward;
  // T5: TODO: move other robots
  int extra_steps = (int)-kick_reward;
  runSteps(extra_steps, action, kicker_id, kick_option_id, false, &result, engine);
  // T6: Testing if ball left the field after kick
  bool late_terminal = false;
  moveBall(ball_real_x, ball_real_y, &ball_final_x, &ball_final_y, &late_terminal, &result.reward);
  result.successor(0) = ball_final_x;
  result.successor(1) = ball_final_y;
  result.terminal = late_terminal;
  return result;
}

Eigen::VectorXd KickControler::getStartingState(std::default_random_engine * engine) const
{
  Eigen::Matrix<double,3,2> limits = getPlayerLimits();
  std::vector<Eigen::VectorXd> random_positions;
  random_positions = rosban_random::getUniformSamples(limits, players.size()+1, engine);
  Eigen::VectorXd state(2 + 3 * players.size());
  state.segment(0,2) = random_positions[0].segment(0,2);// Ball pos
  for (size_t player = 0; player < players.size(); player++) {
    state.segment(2 + player * 3,3) = random_positions[player+1];// Player pos
  }
  return state;
}

void KickControler::runSteps(int max_steps,
                             const Eigen::VectorXd & action,
                             int kicker_id,
                             int kick_option,
                             bool kicker_enabled,
                             Problem::Result * status,
                             std::default_random_engine * engine) const
{
  // Variables used globally in the function
  std::uniform_real_distribution<double> failure_distrib(0,1);
  // Step 1: gather all players states according to their polar_approach
  std::vector<Problem::Result> approach_status;
  std::vector<Eigen::Vector3d> targets;
  for (int player_id = 0; player_id < (int)players.size(); player_id++) {
    // Build player_state and target
    Eigen::Vector3d player_state, target;
    player_state = getPlayerState(status->successor, player_id);
    target = getTarget(status->successor, action, player_id, kicker_id, kick_option);
    // Build player_status
    Problem::Result player_status;
    player_status.successor = toPolarApproachState(player_state, target);
    player_status.reward = 0;
    player_status.terminal = false;
    // Storing necessary values
    approach_status.push_back(player_status);
    targets.push_back(target);
  }
  // Step 2: Simulate until one of these conditions is filled
  // - Max steps is reached
  // - kicker is enabled and has reached target
  // - One of the robot has failed (goal area)
  int nb_steps = 0;
  std::vector<bool> reached_target(players.size(), false);
  bool failed = false;
  while(nb_steps <= max_steps && !reached_target[kicker_id] && !failed) {
    // Simulate all robot iteratively
    for (int player_id = 0; player_id < (int)players.size(); player_id++) {
      // Skip player who have already reached destination
      if (reached_target[player_id]) continue;
      // TODO: avoid code duplication between kickers and non kickers
      if (player_id == kicker_id) {
        if (!kicker_enabled) continue;//Skip kicker if disabled
        const PolarApproach & model = players[player_id]->kick_options[kick_option]->approach_model;
        const csa_mdp::Policy & policy = *(players[player_id]->kick_options[kick_option]->approach_policy);
        // Get action 
        Eigen::VectorXd pa_action = policy.getAction(approach_status[player_id].successor, engine);
        // Simulate approach action
        approach_status[player_id] = model.getSuccessor(approach_status[player_id].successor,
                                                        pa_action, engine);
        // Update 'reached target'
        reached_target[player_id] = model.isKickable(approach_status[player_id].successor);
      }
      else {
        // Retrieve model
        const PolarApproach & model = players[player_id]->navigation_approach;
        const csa_mdp::Policy & policy = *(players[player_id]->approach_policy);
        // Get action 
        Eigen::VectorXd pa_action = policy.getAction(approach_status[player_id].successor, engine);
        // Simulate approach action
        approach_status[player_id] = model.getSuccessor(approach_status[player_id].successor,
                                                        pa_action, engine);
        // Update 'reached target'
        reached_target[player_id] = model.isKickable(approach_status[player_id].successor);
      }
      // Update global status
      status->successor = toKickControlerState(approach_status[player_id].successor,
                                               status->successor,
                                               player_id,
                                               targets[player_id]);
      // Checking if player is inside the goal area
      Eigen::Vector3d player_state = getPlayerState(status->successor, player_id);
      if (isGoalArea(player_state(0), player_state(1))) {
        if (failure_distrib(*engine) < goalkeeper_success_rate) {
          failed = true;
        }
      }
    }
    //Increasing number of steps
    nb_steps++;
  }
  // if kicker is enabled, max_steps should never be reached -> throw error
  if (kicker_enabled && nb_steps >= max_steps) {
    std::ostringstream oss;
    oss << "KickControler::performApproach: kickable state not reached after "
        << max_steps << std::endl
        << "state: " << approach_status[kicker_id].successor.transpose() << std::endl;
    throw std::runtime_error(oss.str());
  }
  // Use failure reward if 
  if (failed) {
    status->reward += failure_reward;
    status->terminal = true;
  }
  // If kicker is enabled, increase status cost
  if (kicker_enabled) status->reward += nb_steps * approach_step_reward;
}

Eigen::Vector3d KickControler::getPlayerState(const Eigen::VectorXd & state,
                                              int player_id) const
{
  return state.segment(2+3*player_id, 3);
}

Eigen::Vector3d KickControler::getTarget(const Eigen::VectorXd & state,
                                         const Eigen::VectorXd & action,
                                         int player_id,
                                         int kicker_id,
                                         int kick_option) const
{
  // Importing basic kick properties
  const KickModel & kick_model = *(players[kicker_id]->kick_options[kick_option]->kick_model);
  int kick_dims = kick_model.getActionsLimits().rows();
  // Computing target
  Eigen::Vector3d target;
  // If player is a kicker, target depend on chosen kick
  if (player_id == kicker_id) {
    // Renaming variables to improve readability
    double ball_x = state(0);
    double ball_y = state(1);
    // Target is the ball
    target(0) = ball_x;
    target(1) = ball_y;
    // Importing kick_action
    Eigen::VectorXd kick_action = action.segment(1, kick_dims);
    // Using kick model to determine where action 
    target(2) = kick_model.getWishedDir(ball_x, ball_y, kick_action);
  }
  else {
    int start_row = 1 + kick_dims + 3 * player_id;
    if (player_id > kicker_id) start_row -= 3;
    target = action.segment(start_row, 3);
  }
  return target;
}

Eigen::VectorXd KickControler::toPolarApproachState(const Eigen::Vector3d & player_state,
                                                    const Eigen::Vector3d & target) const
{
  // Computing basic properties
  double ball_x_field     = target(0);
  double ball_y_field     = target(1);
  double kick_theta_field = target(2);
  double player_x_field = player_state(0);
  double player_y_field = player_state(1);
  double player_theta   = player_state(2);
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
KickControler::toKickControlerState(const Eigen::VectorXd & pa_state,
                                    const Eigen::VectorXd & prev_kc_state,
                                    int player_id,
                                    const Eigen::Vector3d & target) const
{
  // Getting basic data
  double ball_dist = pa_state(0);
  double ball_dir_robot = pa_state(1);
  double target_angle = pa_state(2);
  double ball_x = target(0);
  double ball_y = target(1);
  double kick_theta_field = target(2);
  // Computing player orientation
  double robot_theta_field = normalizeAngle(kick_theta_field - target_angle);
  double robot2ball_theta_field = normalizeAngle(ball_dir_robot + robot_theta_field);
  // Filling state
  Eigen::VectorXd kc_state = prev_kc_state;
  kc_state(2 + 3*player_id) = ball_x - ball_dist * cos(robot2ball_theta_field);
  kc_state(3 + 3*player_id) = ball_y - ball_dist * sin(robot2ball_theta_field);
  kc_state(4 + 3*player_id) = robot_theta_field;
  return kc_state;
}

void KickControler::to_xml(std::ostream & out) const
{
  (void) out;
  throw std::logic_error("KickControler::to_xml: unimplemented");
}

void KickControler::from_xml(TiXmlNode * node)
{
  xml_tools::try_read<double>(node, "step_initial_stddev"    , step_initial_stddev    );
  xml_tools::try_read<double>(node, "goal_reward"            , goal_reward            );
  xml_tools::try_read<double>(node, "approach_step_reward"   , approach_step_reward   );
  xml_tools::try_read<double>(node, "failure_reward"         , failure_reward         );
  xml_tools::try_read<double>(node, "field_width"            , field_width            );
  xml_tools::try_read<double>(node, "field_length"           , field_length           );
  xml_tools::try_read<double>(node, "goal_width"             , goal_width             );
  xml_tools::try_read<double>(node, "goal_area_size_x"       , goal_area_size_x       );
  xml_tools::try_read<double>(node, "goal_area_size_y"       , goal_area_size_y       );
  xml_tools::try_read<double>(node, "goalkeeper_success_rate", goalkeeper_success_rate);

  //TODO: improve format, currently very verbose and redundant
  TiXmlNode* values = node->FirstChild("players");
  players.clear();
  for ( TiXmlNode* child = values->FirstChild(); child != NULL; child = child->NextSibling())
  {
    std::unique_ptr<Player> p(new Player);
    p->from_xml(child);
    players.push_back(std::move(p));
  }
  updateStateLimits();
  updateApproachesLimits();
  updateActionsLimits();
}

std::string KickControler::class_name() const
{
  return "kick_controler";
}

size_t KickControler::getNbPlayers() const
{
  return players.size();
}

void KickControler::analyzeActionId(int action_id, int * kicker_id, int * kick_id) const
{
  int cpt = 0;
  for (int player_id = 0; player_id < (int) players.size(); player_id++) {
    for (size_t opt_id = 0 ; opt_id < players[player_id]->kick_options.size(); opt_id++) {
      if (cpt == action_id) {
        *kicker_id = player_id;
        *kick_id = opt_id;
        return;
      }
      cpt++;
    }
  }
  if (action_id < 0 || action_id >= cpt) {
    std::ostringstream oss;
    oss << "KickControler::getSuccessor: invalid value for action_id: "
        << action_id << " (expecting value in [" << 0 << "," << cpt << "[)";
    throw std::logic_error(oss.str());
  }
}
double KickControler::getKickDir(const Eigen::VectorXd & state,
                                 const Eigen::VectorXd & action) const
{
  int action_id = action(0);
  int kicker_id(0), kick_option(0);
  analyzeActionId(action_id, &kicker_id, &kick_option);
  // Importing basic kick properties
  const KickModel & kick_model = *(players[kicker_id]->kick_options[kick_option]->kick_model);
  int kick_dims = kick_model.getActionsLimits().rows();
  // Renaming variables to improve readability
  double ball_x = state(0);
  double ball_y = state(1);
  // Importing kick_action
  Eigen::VectorXd kick_action = action.segment(1, kick_dims);
  // Using kick model to determine where action 
  return kick_model.getWishedDir(ball_x, ball_y, kick_action);
}

Eigen::Matrix<double,2,2> KickControler::getFieldLimits() const
{
  Eigen::Matrix<double,2,2> field_limits;
  field_limits <<
    -field_length / 2, field_length / 2,
    -field_width / 2, field_width / 2;
  return field_limits;
}

Eigen::Matrix<double,3,2> KickControler::getPlayerLimits() const
{
  Eigen::Matrix<double,3,2> player_limits;
  player_limits <<
    -field_length / 2, field_length / 2,
    -field_width / 2, field_width / 2,
    -M_PI, M_PI;
  return player_limits;
}

void KickControler::updateStateLimits()
{
  int nb_players = players.size();
  /// Updating limits
  Eigen::MatrixXd state_limits(2 + 3 *nb_players,2);
  std::vector<std::string> state_names = {"ball_x","ball_y"};
  state_limits.block(0,0,2,2) = getFieldLimits();
  for (int player_id = 0; player_id < nb_players; player_id++) {
    state_limits.block(2+3*player_id,0,3,2) = getPlayerLimits();
    for (const std::string & suffix : {"x","y","theta"}) {
      std::ostringstream oss;
      oss << players[player_id]->name << "_" << suffix;
      state_names.push_back(oss.str());
    }
  }
  setStateLimits(state_limits);
  setStateNames(state_names);
}

void KickControler::updateApproachesLimits()
{
  /// Ball can be a lot further than in usual approach_model
  double max_dist = field_width + field_length;//Extra margin is not an issue
  for (size_t player_id = 0; player_id < players.size(); player_id++) {
    players[player_id]->navigation_approach.setMaxDist(max_dist);
    for (size_t ko_id = 0; ko_id < players[player_id]->kick_options.size(); ko_id++) {
      players[player_id]->kick_options[ko_id]->approach_model.setMaxDist(max_dist);
    }
  }
}

void KickControler::updateActionsLimits()
{
  // Updating
  std::vector<Eigen::MatrixXd> kick_action_limits;
  std::vector<std::vector<std::string>> kick_action_names;
  for (size_t p_id = 0; p_id < players.size(); p_id++) {
    // Update actions limits for navigation
    std::vector<Eigen::MatrixXd> nav_action_limits;
    nav_action_limits = players[p_id]->navigation_approach.getActionsLimits();
    players[p_id]->approach_policy->setActionLimits(nav_action_limits);
    for (size_t ko_id = 0; ko_id < players[p_id]->kick_options.size(); ko_id++) {
      // kick_model
      const KickModel & kick_model = *(players[p_id]->kick_options[ko_id]->kick_model);
      // Updating the approach action limits for approach policies
      std::vector<Eigen::MatrixXd> app_action_limits;
      app_action_limits = players[p_id]->kick_options[ko_id]->approach_model.getActionsLimits();
      players[p_id]->kick_options[ko_id]->approach_policy->setActionLimits(app_action_limits);
      // Getting current kick model parameters limits
      Eigen::MatrixXd kick_limits = kick_model.getActionsLimits();
      int kick_dims = kick_limits.rows();
      // Filling action spaces and names for this problem
      Eigen::MatrixXd action_limits(kick_dims + 3 * (players.size()-1),2);
      action_limits.block(0,0,kick_dims,2) = kick_limits;
      std::vector<std::string> action_names = kick_model.getActionsNames();
      // A target is provided for each robot
      for (size_t non_kicker_id = 0; non_kicker_id < players.size(); non_kicker_id++) {
        int start_row = kick_dims + non_kicker_id * 3;
        if (non_kicker_id == p_id) continue;
        if (non_kicker_id > p_id) start_row -= 3;
        action_limits.block(start_row,0,3,2) = getPlayerLimits();
        for (const std::string & suffix : {"x","y","dir"}) {
          std::ostringstream oss;
          oss << players[non_kicker_id]->name << "_" << suffix;
          action_names.push_back(oss.str());
        }
      }
      std::cout << "Action_limits: " << kick_action_limits.size() << std::endl
                << action_limits << std::endl;
      kick_action_limits.push_back(action_limits);
      kick_action_names.push_back(action_names);
    }
  }
  setActionLimits(kick_action_limits);
  setActionsNames(kick_action_names);
}

void KickControler::initialBallNoise(double ball_x, double ball_y,
                                     double * ball_real_x, double * ball_real_y,
                                     std::default_random_engine * engine) const
{
  std::normal_distribution<double> noise_distrib(0, step_initial_stddev);
  *ball_real_x = ball_x + noise_distrib(*engine);
  *ball_real_y = ball_y + noise_distrib(*engine);
}

void KickControler::moveBall(double src_x, double src_y,
                             double * dst_x, double * dst_y,
                             bool * terminal, double * reward) const
{
  if (isCollidingGoalie(src_x,src_y,dst_x,dst_y)) {
    *reward = *reward + failure_reward;
    *terminal = true;
    return;
  }
  else if (isGoal(src_x,src_y,dst_x,dst_y)) {
    *reward = *reward + goal_reward;
    *terminal = true;
    return;
  }
  else if (isOut(src_x,src_y,dst_x,dst_y)) {
    *reward = *reward + failure_reward;
    *terminal = true;
    return;
  }
  *terminal = false;
}


bool KickControler::isCollidingGoalie(double src_x, double src_y,
                                      double * dst_x, double * dst_y) const
{
  // 0: precomputing values (using ball radius)
  double min_goalie_x = goalie_x - goalie_thickness / 2 - ball_radius;
  double max_goalie_x = goalie_x + goalie_thickness / 2 + ball_radius;
  double min_goalie_y = goalie_y - goalie_width / 2 - ball_radius;
  double max_goalie_y = goalie_y + goalie_width / 2 + ball_radius;
  double dx = *dst_x - src_x;
  double dy = *dst_y - src_y;
  // 1: If both points are above or below specific values, collision is not possible
  bool below_x = src_x < min_goalie_x && *dst_x < min_goalie_x;
  bool above_x = src_x > max_goalie_x && *dst_x > max_goalie_x;
  bool below_y = src_y < min_goalie_y && *dst_y < min_goalie_y;
  bool above_y = src_y > max_goalie_y && *dst_y > max_goalie_y;
  if (below_x || above_x || below_y || above_y) return false;
  // 2: Special case, ball moving laterally: spot first collision (mandatory)
  if (dx == 0) {
    double dist_to_min = std::fabs(src_y - min_goalie_y);
    double dist_to_max = std::fabs(src_y - max_goalie_y);
    if (dist_to_min < dist_to_max) {
      *dst_y = min_goalie_y;
    }
    else {
      *dst_y = max_goalie_y;
    }
    return true;
  }
  // 3: Computing trajectories under the form a*x + b
  double a = dy / dx;
  double b = src_y - a * src_x;
  // 4: Detecting potential collision:
  std::vector<double> collisions_x, collisions_y;
  double tmp_x, tmp_y;
  // 4.1: collision with min_goalie_x
  tmp_x = min_goalie_x;
  tmp_y = a * tmp_x + b;
  if (tmp_y > min_goalie_y && tmp_y < max_goalie_y) {
    collisions_x.push_back(tmp_x);
    collisions_y.push_back(tmp_y);
  }
  // 4.2: collision with max_goalie_x
  tmp_x = max_goalie_x;
  tmp_y = a * tmp_x + b;
  if (tmp_y > min_goalie_y && tmp_y < max_goalie_y) {
    collisions_x.push_back(tmp_x);
    collisions_y.push_back(tmp_y);
  }
  // 4.3: collision with min_goalie_y
  if (a != 0) {
    tmp_y = min_goalie_y;
    tmp_x = (tmp_y - b) / a;
    if (tmp_x > min_goalie_x && tmp_x < max_goalie_x) {
      collisions_x.push_back(tmp_x);
      collisions_y.push_back(tmp_y);
    }
  }
  // 4.4: collision with max_goalie_y
  if (a != 0) {
    tmp_y = max_goalie_y;
    tmp_x = (tmp_y - b) / a;
    if (tmp_x > min_goalie_x && tmp_x < max_goalie_x) {
      collisions_x.push_back(tmp_x);
      collisions_y.push_back(tmp_y);
    }
  }
  // 5: If no collision has been found return false
  if (collisions_x.size() == 0) return false;
  // 6: Choose the closest collision to 'src'
  double best_dist2 = std::numeric_limits<double>::max();
  for (int i = 0; i < (int) collisions_x.size(); i++) {
    double dx = collisions_x[i] - src_x;
    double dy = collisions_y[i] - src_y;
    double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      *dst_x = collisions_x[i];
      *dst_y = collisions_y[i];
    }
  }
  return true;
}

bool KickControler::isGoal(double src_x, double src_y,
                           double * dst_x, double * dst_y) const
{
  double dx = *dst_x - src_x;
  double limit_x = field_length / 2 + ball_radius;
  // Impossible to score a goal by kicking backward or if ball did not entirely cross the line
  if (dx <= 0 || *dst_x < limit_x) return false;
  // Finding equation a*x +b;
  double dy = *dst_y - src_y;
  double a = dy / dx;
  double b = src_y - a * src_x;
  // Finding intersection with goal line
  double intercept_y = a * field_length / 2 + b;
  bool intersect = std::fabs(intercept_y) < goal_width / 2;
  // If no intersection: 
  if (!intersect) return false;
  // Set x to a given limit without changing trajectory
  *dst_x = limit_x;
  *dst_y = a * limit_x + b;
  return true;
}

bool KickControler::isOut(double src_x, double src_y,
                          double * dst_x, double * dst_y) const
{
  (void)src_x;(void)src_y;
  // Computing limits
  double min_x = - field_length / 2 - ball_radius;
  double max_x =   field_length / 2 + ball_radius;
  double min_y = - field_width  / 2 - ball_radius;
  double max_y =   field_width  / 2 + ball_radius;
  // Getting values
  return (*dst_x < min_x || *dst_y < min_y || *dst_x > max_x || *dst_y > max_y);
  //TODO: modifies values of dst if ball is out
}

bool KickControler::isGoalArea(double player_x, double player_y) const
{
  bool xOk = player_x > (field_length/2 - goal_area_size_x) && player_x < field_length/2;
  bool yOk = std::fabs(player_y) < (field_width - goal_area_size_y) / 2;
  return xOk && yOk;
}

}
