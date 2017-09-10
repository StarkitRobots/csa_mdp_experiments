#include "problems/kick_controler.h"

#include "kick_model/kick_decision_model_factory.h"
#include "kick_model/kick_model_collection.h"
#include "kick_model/kick_model_factory.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_random/tools.h"
#include "rosban_utils/xml_tools.h"

using namespace rosban_utils;
using namespace rosban_utils::xml_tools;

// Normalize angle in [-pi,pi]
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
  kick_decision_model = KickDecisionModelFactory().read(node, "kick_decision_model");
  approach_model.tryRead(node,"approach_model");
  // Replace approach_policy if found
  PolicyFactory().tryRead(node, "policy", approach_policy);
  // Reading kick_model from name if found
  kick_model_names = read_vector<std::string>(node, "kick_model_names");
}

std::string KickControler::KickOption::class_name() const
{
  return "kick_controler_kick_option";
}

void KickControler::KickOption::syncKickZones(const KickModelCollection & kmc)
{
  approach_model.clearKickZones();
  for (const std::string & name : kick_model_names) {
    approach_model.addKickZone(kmc.getKickModel(name).getKickZone());
  }
}

void KickControler::Player::to_xml(std::ostream & out) const {
  (void) out;
  //TODO
  throw std::logic_error("KickControler::Player::to_xml: not implemented");
}

void KickControler::Player::from_xml(TiXmlNode * node)
{
  name = rosban_utils::xml_tools::read<std::string>(node, "name");
  navigation_approach.tryRead(node, "approach_model");
  // Replace approach_policy if found
  PolicyFactory().tryRead(node, "policy", approach_policy);
  // Reading kick options
  TiXmlNode* values = node->FirstChild("kick_options");
  kick_options.clear();
  for ( TiXmlNode* child = values->FirstChild(); child != NULL; child = child->NextSibling())
  {
    std::unique_ptr<KickOption> ko(new KickOption());
    // default values for approach_model are provided by navigation_approach
    ko->approach_model = navigation_approach;
    ko->from_xml(child);
    kick_options.push_back(std::move(ko));
  }
  // Update odometry on all kick_options models
  for (size_t kick_id = 0; kick_id < kick_options.size(); kick_id++) {
    kick_options[kick_id]->approach_model.setOdometry(navigation_approach.getOdometry());
  }
}

std::string KickControler::Player::class_name() const
{
  return "kick_controler_player";
}

KickControler::KickControler()
  : max_initial_dist(20.0),
    simulate_approaches(true),
    cartesian_speed(0.2),
    angular_speed(M_PI/4),
    step_initial_stddev(0.25),
    goal_reward(0),
    goal_collision_reward(-500),
    approach_step_reward(-1),
    failure_reward(-500),
    field_width(6),
    field_length(9),
    goal_width(2.6),
    ball_radius(0.07),
    use_goalie(true),
    goal_area_size_x(1),
    goal_area_size_y(5),
    goalkeeper_success_rate(0.05),
    goalie_x(field_length/2 - 0.1),
    goalie_y(0),
    goalie_thickness(0.2),
    goalie_width(0.4),
    kick_dist_ratio(0.85),
    intercept_dist(0.75)
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
  // Initializing result properties
  Problem::Result result;
  result.successor = state;
  result.reward = 0;
  result.terminal = false;
  // Getting player_id and kick_id:
  int action_id = (int)action(0);
  int kicker_id, kick_option_id;
  analyzeActionId(action_id, &kicker_id, &kick_option_id);
  // If there is a kicker specifed, use its own kick, otherwise, use default kick
  const KickOption & kick_option =
    kicker_id == -1 ? *(kick_options[kick_option_id])
    : *(players[kicker_id]->kick_options[kick_option_id]);
  // Checking action dimension (depending on kick_option)
  int kick_dims = kick_option.kick_decision_model->getActionsLimits().rows();
  int expected_action_dimension = 1 + kick_dims;
  if (action.rows() != expected_action_dimension) {
    std::ostringstream oss;
    oss << "KickControler::getSuccessor: invalid dimension for action: "
        << action.rows() << " (expected " << expected_action_dimension << ")";
    throw std::logic_error(oss.str());
  }
  // Importing the parameters of the state and action
  Eigen::Vector2d ball_seen = state.segment(0,2);
  Eigen::VectorXd decision_actions = action.segment(1, kick_dims);
  double ball_x = state(0);
  double ball_y = state(1);
  // Computing decision parameters
  const KickDecisionModel & kdm = *(kick_option.kick_decision_model);
  double kick_dir = kdm.computeKickDirection(ball_seen, decision_actions);
  Eigen::VectorXd kick_parameters = kdm.computeKickParameters(ball_seen,
                                                              decision_actions);
  // T1: Adding noise to get ball_real
  double ball_real_x, ball_real_y;
  initialBallNoise(ball_x, ball_y, &ball_real_x, &ball_real_y, engine);
  // T2: Move the ball to its real position
  bool early_terminal = false;
  moveBall(ball_x, ball_y, &ball_real_x, &ball_real_y, &early_terminal, &result.reward);
  Eigen::Vector2d ball_real(ball_real_x, ball_real_y);
  if (early_terminal)
  {
    // Update ball position and force it to be terminal
    result.successor.segment(0,2) = ball_real;
    result.terminal = true;
    // No need to perform additional computations
    return result;
  }
  // T3: Simulate players approach (end approach if one of the robot commited illegal attack)
  if (kicker_id != -1) {
    int max_steps = 500;
    if (simulate_approaches) {
      runSteps(max_steps, action, kicker_id, kick_option_id, true, &result, engine);
    } else {
      approximateKickerApproach(ball_real, action, kicker_id, kick_option_id, &result);
    }
    if (result.terminal) {
      return result;
    }
  }
  // T4.0: Find the kick which need to be applied
  //       (or return if none can be applied)
  //       if no players are available, then only one kick should be available
  if (kick_option.kick_model_names.size() == 0) {
    throw std::logic_error("no kick_model_names for current kick_option");
  }
  std::string kick_name;
  if (kicker_id == -1) {
    if (kick_option.kick_model_names.size() > 1) {
      // Another choice could be to choose name randomly
      throw std::logic_error("With no players, kick_model only support a single kick_model_name");
    }
    kick_name = kick_option.kick_model_names[0];
  } else {
    const Eigen::Vector3d & kicker_state =
      getPlayerState(result.successor, kicker_id);
    for (const std::string & name : kick_option.kick_model_names) {
      const KickZone & kick_zone = kmc.getKickModel(name).getKickZone();
      if (kick_zone.isKickable(ball_real, kicker_state, kick_dir)) {
        kick_name = name;
        break;
      }
    }
    if (kick_name == "") {
      return result;
    }
  }
  const KickModel & kick_model = kmc.getKickModel(kick_name);
  // T4.1: Apply kick with noise
  double ball_final_x, ball_final_y;
  double kick_reward;
  Eigen::Vector2d ball_final;
  ball_final = kick_model.applyKick(ball_real, kick_dir, kick_parameters, engine);
  ball_final_x = ball_final(0);
  ball_final_y = ball_final(1);
  kick_reward = kick_model.getReward();
  // T5: move other robots
  if (kicker_id != -1) {
    int extra_time = (int)-kick_reward;
    if (simulate_approaches) {
      runSteps(extra_time, action, kicker_id, kick_option_id, false, &result, engine);
    } else {
      Eigen::Vector2d expected_ball_pos = kick_model.applyKick(ball_real, kick_dir);
      moveNonKickers(extra_time, ball_real, expected_ball_pos, kicker_id, &result);
    }
  }
  // T6: Testing if ball left the field after kick (scoring or not scoring goal is computed)
  bool late_terminal = false;
  moveBall(ball_real_x, ball_real_y, &ball_final_x, &ball_final_y, &late_terminal, &result.reward);
  result.successor(0) = ball_final_x;
  result.successor(1) = ball_final_y;
  result.terminal = late_terminal;
  // When there is no robot considered, and ball is not out, extra-cost depends
  // on cartesian distance between source and target
  if (kicker_id == -1 && !late_terminal) {
    double traveled_dist = (ball_real - ball_final).norm();
    kick_reward -= traveled_dist / cartesian_speed;
  }
  result.reward += kick_reward;
  return result;
}

Eigen::VectorXd KickControler::getStartingState(std::default_random_engine * engine) const
{
  Eigen::VectorXd state(2 + 3 * players.size());
  // Getting ball position
  Eigen::VectorXd ball_pos =
    rosban_random::getUniformSample(getFieldLimits(), engine);
  state.segment(0,2) = ball_pos;
  // Sampling player position
  for (size_t player = 0; player < players.size(); player++) {
    int start_idx = 2 + player * 3;
    // Ensuring that distance from players to ball is below max_initial_dist
    bool valid = false;
    while (!valid) {
      state.segment(start_idx,3) = 
        rosban_random::getUniformSample(getPlayerLimits(), engine);
      valid = (state.segment(start_idx,2) - ball_pos).norm() < max_initial_dist;
    }
  }
  return state;
}

const csa_mdp::Policy & KickControler::getPolicy(int player_id,
                                                 int kicker_id,
                                                 int kick_option_id) const
{
  const Player & player = *(players[player_id]);
  // For kickers, try to use a policy specific to kick option if available
  if (player_id == kicker_id) {
    const KickOption & ko = *(player.kick_options[kick_option_id]);
    if (ko.approach_policy) return *(ko.approach_policy);
  }
  // Otherwise, use custom player approach policy if provided
  if (player.approach_policy) return *(player.approach_policy);
  // If none policy has been found, throw an explicit error
  std::ostringstream oss;
  oss << "KickControler::getPolicy: no policy found for player "
      << player_id
      << "(kicker_id: " << kicker_id << " & kick_option_id: " << kick_option_id << ")";
  throw std::logic_error(oss.str());
}

void KickControler::runSteps(int max_steps,
                             const Eigen::VectorXd & action,
                             int kicker_id,
                             int kick_option_id,
                             bool kicker_enabled,
                             Problem::Result * status,
                             std::default_random_engine * engine) const
{
  if (!simulate_approaches) {
    throw std::logic_error("KickControler::runSteps: simulate_approaches is not enabled");
  }
  // Variables used globally in the function
  std::uniform_real_distribution<double> failure_distrib(0,1);
  const KickOption & kick_option = *(players[kicker_id]->kick_options[kick_option_id]);
  // Step 1: gather all players states according to their polar_approach
  std::vector<Problem::Result> approach_status;
  std::vector<Eigen::Vector3d> targets;
  for (int player_id = 0; player_id < (int)players.size(); player_id++) {
    // Build player_state and target
    Eigen::Vector3d player_state, target;
    player_state = getPlayerState(status->successor, player_id);
    target = getTarget(status->successor, action, player_id, kicker_id, kick_option_id);
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
      // Import policy
      const csa_mdp::Policy & policy = getPolicy(player_id, kicker_id, kick_option_id);
      // Get action 
      Eigen::VectorXd pa_action = policy.getAction(approach_status[player_id].successor, engine);
      // TODO: avoid code duplication between kickers and non kickers
      if (player_id == kicker_id) {
        if (!kicker_enabled) continue;//Skip kicker if disabled
        const PolarApproach & model = kick_option.approach_model;
        // Simulate approach action
        approach_status[player_id] = model.getSuccessor(approach_status[player_id].successor,
                                                        pa_action, engine);
        // Update 'reached target'
        reached_target[player_id] = model.isKickable(approach_status[player_id].successor);
      }
      else {
        // Retrieve model
        const PolarApproach & model = players[player_id]->navigation_approach;
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
        << "kicker: " << kicker_id << std::endl
        << "kick_option: " << kick_option_id << std::endl
        << "state: " << approach_status[kicker_id].successor.transpose() << std::endl;
    throw std::runtime_error(oss.str());
  }
  // Use failure reward if approach failed
  if (failed) {
    status->reward += failure_reward;
    status->terminal = true;
  }
  // If kicker is enabled, increase status cost
  if (kicker_enabled) status->reward += nb_steps * approach_step_reward;
}

double KickControler::getApproximatedTime(const Eigen::VectorXd & src,
                                          const Eigen::VectorXd & dst) const
{
  double dist = (src.segment(0,2) - dst.segment(0,2)).norm();
  double ang_dist = std::fabs(std::fmod(src(2) - dst(2), 2 * M_PI));
  if (ang_dist > M_PI) ang_dist = 2 * M_PI - ang_dist;
  return dist / cartesian_speed + ang_dist / angular_speed;
}

void KickControler::approximateKickerApproach(const Eigen::Vector2d & ball_real_pos,
                                              const Eigen::VectorXd & action,
                                              int kicker_id,
                                              int kick_option_id,
                                              Problem::Result * status) const
{
  // Importing variables
  Eigen::Vector3d kicker_state = getPlayerState(status->successor, kicker_id);
  const KickOption & kick_option = *(players[kicker_id]->kick_options[kick_option_id]);
  if (kick_option.kick_model_names.size() != 1) {
    throw std::logic_error("Cannot approximateKickerApproach for problems with multiple kicks available");
  }
  std::string kick_name = kick_option.kick_model_names[0];
  const KickModel & kick_model = kmc.getKickModel(kick_name);
  const KickZone & kick_zone = kick_model.getKickZone();
  double kick_wished_dir = getKickDir(status->successor, action);
  // Compute target and time for kicker (best between left and right)
  double min_time = std::numeric_limits<double>::max();
  Eigen::Vector3d kicker_target;
  for (bool use_right_foot : {true, false}) {
    Eigen::Vector3d target = kick_zone.getWishedPosInField(ball_real_pos, kick_wished_dir, use_right_foot);
    double time = getApproximatedTime(kicker_state,target);
    if (time < min_time) {
      min_time = time;
      kicker_target = target;
    }
  }
  // Set player state in status and add cost
  status->reward -= min_time;
  status->successor.segment(2+3*kicker_id, 3) = kicker_target;
  // Move all remaining players
  Eigen::Vector2d expected_ball_pos = kick_model.applyKick(ball_real_pos, kick_wished_dir);
  moveNonKickers(min_time, ball_real_pos, expected_ball_pos, kicker_id, status);
}

void KickControler::moveNonKickers(double allowed_time,
                                   const Eigen::Vector2d & ball_start,
                                   const Eigen::Vector2d & ball_end,
                                   int kicker_id,
                                   Problem::Result * status) const
{
  for (int robot_id = 0; robot_id < (int)players.size(); robot_id++) {
    if (kicker_id != robot_id) {
      moveRobot(allowed_time, ball_start, ball_end, robot_id, status);
    }
  }
}

void KickControler::moveRobot(double allowed_time,
                              const Eigen::Vector2d & ball_start,
                              const Eigen::Vector2d & ball_end,
                              int robot_id,
                              Problem::Result * status) const
{
  // Get placing targets
  Eigen::Vector3d robot_state = getPlayerState(status->successor, robot_id);
  Eigen::Vector3d best_target = getBestTarget(ball_start, ball_end, robot_state);
  // First, move toward desired point
  Eigen::Vector2d required_move = best_target.segment(0,2) - robot_state.segment(0,2);
  double move_time = required_move.norm() / cartesian_speed;
  if (move_time > allowed_time) {
    Eigen::Vector2d normalized_move = required_move / required_move.norm();
    double traveled_dist = cartesian_speed * allowed_time;
    Eigen::Vector2d move = traveled_dist * normalized_move;
    Eigen::Vector2d new_pos = robot_state.segment(0,2) + move;
    status->successor.segment(2+3*robot_id,2) = new_pos;
    return;
  } else {
    status->successor.segment(2+3*robot_id,2) = best_target.segment(0,2);
    allowed_time -= move_time;
  }
  // Then, turn toward desired direction
  double required_turn = normalizeAngle(best_target(2) - robot_state(2));
  double turn_time = std::fabs(required_turn) / angular_speed;
  if (turn_time > allowed_time) {
    double turn = angular_speed * allowed_time;
    if (required_turn < 0) turn *= -1;
    double new_dir = normalizeAngle(robot_state(2) + turn);
    status->successor(2+3*robot_id+2) = new_dir;
  } else {
    status->successor(2+3*robot_id+2) = best_target(2);
  }
}

Eigen::Vector3d KickControler::getBestTarget(const Eigen::Vector2d & ball_start,
                                             const Eigen::Vector2d & ball_end,
                                             const Eigen::Vector3d & robot_state) const
{
  double min_time = std::numeric_limits<double>::max();
  Eigen::Vector3d best_target;
  for (const Eigen::Vector3d & target :getTargets(ball_start, ball_end)) {
    double time = getApproximatedTime(robot_state, target);
    if (time < min_time) {
      best_target = target;
      min_time = time;
    }
  }
  return best_target;
}

std::vector<Eigen::Vector3d> KickControler::getTargets(const Eigen::Vector2d & ball_start,
                                                       const Eigen::Vector2d & ball_end) const
{
  // Precomputing data
  Eigen::Vector2d ball_move = ball_end - ball_start;
  double kick_dir = atan2(ball_move(1), ball_move(0));
  // Heuristic used to place the non-kickers
  Eigen::Vector2d ball_intercept = ball_start + kick_dist_ratio * ball_move;
  // Get the placing targets
  std::vector<Eigen::Vector3d> targets;
  for (double dir_offset : {-M_PI/2, M_PI/2}) {
    Eigen::Vector2d intercept_offset(intercept_dist * cos(kick_dir + dir_offset),
                                     intercept_dist * sin(kick_dir + dir_offset));
    Eigen::Vector3d target;
    target.segment(0,2) = ball_intercept + intercept_offset;
    target(2) = normalizeAngle(kick_dir - dir_offset);// Robot is facing the ball
    targets.push_back(target);
  }
  return targets;
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
  const KickOption & ko = *(players[kicker_id]->kick_options[kick_option]);
  const KickDecisionModel & kdm = *(ko.kick_decision_model);
  if (ko.kick_model_names.size() != 1) {
    throw std::logic_error(
      "KickControler::getTarget: only single kick types are supported");
  }
  std::string kick_name = (ko.kick_model_names[0]);
  const KickModel & km = kmc.getKickModel(kick_name);
  int kick_dims = kdm.getActionsLimits().rows();
  // Renaming variables to improve readability
  Eigen::Vector2d ball_seen = state.segment(0,2);
  Eigen::VectorXd kick_actions = action.segment(1, kick_dims);
  // Using kick decision model to determine direction of the kick
  double kick_dir = kdm.computeKickDirection(ball_seen, kick_actions);
  // Computing target
  Eigen::Vector3d target;
  // If player is a kicker, target depend on chosen kick
  if (player_id == kicker_id) {
    // Target is the ball and direction of the kick as computed previously
    target.segment(0,2) = ball_seen;
    target(2) = kick_dir;
  }
  // If player is not a kicker, determine best target using approximation of speed
  else {
    Eigen::Vector2d expected_ball_pos = km.applyKick(ball_seen, kick_dir);
    Eigen::Vector3d robot_state = getPlayerState(state, player_id);
    target = getBestTarget(ball_seen, expected_ball_pos, robot_state);
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
  xml_tools::try_read<bool>  (node, "simulate_approaches"    , simulate_approaches    );
  xml_tools::try_read<double>(node, "max_initial_dist"       , max_initial_dist       );
  xml_tools::try_read<double>(node, "cartesian_speed"        , cartesian_speed        );
  xml_tools::try_read<double>(node, "angular_speed"          , angular_speed          );
  xml_tools::try_read<double>(node, "step_initial_stddev"    , step_initial_stddev    );
  xml_tools::try_read<double>(node, "goal_reward"            , goal_reward            );
  xml_tools::try_read<double>(node, "goal_collision_reward"  , goal_collision_reward  );
  xml_tools::try_read<double>(node, "approach_step_reward"   , approach_step_reward   );
  xml_tools::try_read<double>(node, "failure_reward"         , failure_reward         );
  xml_tools::try_read<double>(node, "field_width"            , field_width            );
  xml_tools::try_read<double>(node, "field_length"           , field_length           );
  xml_tools::try_read<double>(node, "goal_width"             , goal_width             );
  xml_tools::try_read<bool>  (node, "use_goalie"             , use_goalie             );
  xml_tools::try_read<double>(node, "goalie_x"               , goalie_x               );
  xml_tools::try_read<double>(node, "goalie_y"               , goalie_y               );
  xml_tools::try_read<double>(node, "goalie_thickness"       , goalie_thickness       );
  xml_tools::try_read<double>(node, "goalie_width"           , goalie_width           );
  xml_tools::try_read<double>(node, "goal_area_size_x"       , goal_area_size_x       );
  xml_tools::try_read<double>(node, "goal_area_size_y"       , goal_area_size_y       );
  xml_tools::try_read<double>(node, "goalkeeper_success_rate", goalkeeper_success_rate);
  xml_tools::try_read<double>(node, "kick_dist_ratio"        , kick_dist_ratio        );
  xml_tools::try_read<double>(node, "intercept_dist"         , intercept_dist         );

  /// Reading optional path
  std::string kmc_path = xml_tools::read<std::string>(node, "kmc_path");
  kmc.load_file(kmc_path);

  //TODO: improve format, currently very verbose and redundant
  TiXmlNode* values = node->FirstChild("players");
  players.clear();
  for ( TiXmlNode* child = values->FirstChild(); child != NULL; child = child->NextSibling())
  {
    std::unique_ptr<Player> p(new Player);
    p->from_xml(child);
    for (size_t kick_id = 0; kick_id < p->kick_options.size(); kick_id++) {
      p->kick_options[kick_id]->syncKickZones(kmc);
    }
    players.push_back(std::move(p));
  }

  // Loading kick options if found
  kick_options.clear();
  TiXmlNode* ko_values = node->FirstChild("kick_options");
  if (ko_values != nullptr) {
    for ( TiXmlNode* child = ko_values->FirstChild(); child != NULL; child = child->NextSibling())
    {
      std::unique_ptr<KickOption> ko(new KickOption());
      // no default values for approach_model
      ko->from_xml(child);
      kick_options.push_back(std::move(ko));
    }
  }

  // Consistency check
  if (players.size() == 0 && simulate_approaches) {
    throw std::runtime_error("KickControler::from_xml: no players is incompatible with enabling simulated approaches");
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
  // Specific case when there is no player considered
  if (players.size() == 0) {
    *kicker_id = -1;
    *kick_id = action_id;
    cpt = kick_options.size();
  }
  // Determining which player kicks and with which kind of kick
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

const std::vector<std::string> & KickControler::getAllowedKicks(int kicker_id,
                                                                int kick_option)
{
  return players[kicker_id]->kick_options[kick_option]->kick_model_names;
}

double KickControler::getKickDir(const Eigen::VectorXd & state,
                                 const Eigen::VectorXd & action) const
{
  int action_id = action(0);
  int kicker_id(0), kick_option(0);
  analyzeActionId(action_id, &kicker_id, &kick_option);
  // Importing basic kick properties
  const KickDecisionModel & kdm =
    *(players[kicker_id]->kick_options[kick_option]->kick_decision_model);
  int kick_dims = kdm.getActionsLimits().rows();
  // Renaming variables to improve readability
  Eigen::Vector2d ball_seen = state.segment(0,2);
  // Importing kick_action
  Eigen::VectorXd kick_action = action.segment(1, kick_dims);
  // Using kick model to determine where action 
  return kdm.computeKickDirection(ball_seen, kick_action);
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
    if (players[p_id]->approach_policy) {
      players[p_id]->approach_policy->setActionLimits(nav_action_limits);
    }
    for (size_t ko_id = 0; ko_id < players[p_id]->kick_options.size(); ko_id++) {
      // kick_model
      const KickDecisionModel & kdm =
        *(players[p_id]->kick_options[ko_id]->kick_decision_model);
      // Updating the approach action limits for approach policies
      std::vector<Eigen::MatrixXd> app_action_limits;
      app_action_limits = players[p_id]->kick_options[ko_id]->approach_model.getActionsLimits();
      if (players[p_id]->kick_options[ko_id]->approach_policy) {
        players[p_id]->kick_options[ko_id]->approach_policy->setActionLimits(app_action_limits);
      }
      // Getting current kick model parameters limits
      Eigen::MatrixXd kick_limits = kdm.getActionsLimits();
      int kick_dims = kick_limits.rows();
      // Filling action spaces and names for this problem
      Eigen::MatrixXd action_limits(kick_dims,2);
      action_limits.block(0,0,kick_dims,2) = kick_limits;
      std::vector<std::string> action_names = kdm.getActionsNames();
      // Adding action limits and names
      kick_action_limits.push_back(action_limits);
      kick_action_names.push_back(action_names);
    }
  }
  // If no players, add global kick options
  if (players.size() == 0) {
    for (size_t ko_id = 0; ko_id < kick_options.size(); ko_id++) {
      // kick_model
      const KickDecisionModel & kdm = *(kick_options[ko_id]->kick_decision_model);
      // Getting informations from kdm
      Eigen::MatrixXd action_limits = kdm.getActionsLimits();
      std::vector<std::string> action_names = kdm.getActionsNames();
      // Filling actions
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
  if (use_goalie && isCollidingGoalie(src_x,src_y,dst_x,dst_y)) {
    *reward = *reward + goal_collision_reward;
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
