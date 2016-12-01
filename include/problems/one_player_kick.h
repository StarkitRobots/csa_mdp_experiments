#pragma once

#include "rosban_csa_mdp/core/black_box_problem.h"
#include "problems/polar_approach.h"

#include "rosban_csa_mdp/core/policy.h"

#include <memory>

namespace csa_mdp
{

/// This problem consist of choosing the best kick direction and power to
/// reduce the time spent before scoring a goal for a humanoid robot. Each time
/// step consists of: an approach toward the kick position and then a kick with
/// the provided kick power
///
/// Referentials:
/// - x axis: from our goal to the opposite goal
/// - y axis: moving laterally on the field (z aims to the roof of the field)
/// - (0,0)
///
/// The state space is the following:
/// 0. ball_x
/// 1. ball_y
/// 2. robot_x
/// 3. robot_y
/// 4. robot_theta
///
/// The action space is the following
/// 0. kick_direction in [-pi, pi] (according to the coordinate)
/// 1. kick_power in [kick_power_min, kick_power_max]
///
///
/// TRANSITION FUNCTION:
/// To simulate the fact that:
/// - the ball position is unknown
/// - the player places himself according to the angle specified by kick_direction
/// The following transition function is used:
/// 1. The ball is moved from *ball* to *ball_real* by adding noise on the ball position.
/// 2. If *ball_real* is outside of the field:
///    a) The segment *ball* to *ball_real* intersect with the goal segment
///       - A goal is scored and a specific value is returned (cf. below)
///    b) The segment *ball* to *ball_real* do not intersect with the goal segment
///       - A failure is detected and a specific value is returned (cf. below)
/// 3. The player is placed at: *ball* - Polar(kick_range, kick_direction)
/// 4. If player is inside goal_area, there is a probability: *goalkeeper_success_rate* that
///    the approach is a failure
///    NOTE: failure rate might depend on the time spent in the goal_area (more complex)
/// 5. Randomized shoot:
///    - A random noise is added on *kick_direction* to create *kick_real_direction*
///    - A random noise is added on *kick_power* to create *kick_real_power*
///    - *ball_final* is computed: *ball_real* + polar(*kick_real_power*, *kick_real_direction*)
/// 6. If *ball_final* is outside of the field, the same process as in 2. is used
///
/// WARNING: Due to a flaw in software design, problems where the reward and the
///          terminal function are part of the event and not simply function of (s,a,s')
///          cannot be handled properly, therefore a special encoding is used to 
///          describe all the data necessary for answering consistently to the question
///          *isTerminal* and *getReward*
///
/// 1. Terminal states:
///    - On success (goal): ball position becomes ( field_length, 0)
///    - On failure       : ball position becomes (-field_length, 0)
/// 2. Approach cost
///    - On normal cases        : nothing special
///    - When robot did not move: player position becomes (field_length, 0, 0)
///      - This special case might happen if *ball_real* was outside of the field
class OnePlayerKick : public BlackBoxProblem
{
public:
  OnePlayerKick();

  /// cf class description
  bool isTerminal(const Eigen::VectorXd & state) const override;

  /// cf class description
  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) const override;

  /// cf class description
  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  /// At starting state:
  /// Player 1 and Player 2 are placed randomly on the field
  /// Ball is placed randomly
  Eigen::VectorXd getStartingState(std::default_random_engine * engine) const override;

  /// Return the expected reward for the given kicker during approach
  /// toward kicking the ball with target angle
  double getApproachReward(const Eigen::VectorXd & state,
                           const Eigen::VectorXd & action) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:

  /// Add a ball
  void initialBallNoise(double ball_x, double ball_y,
                        double * ball_real_x, double * ball_real_y,
                        std::default_random_engine * engine) const;

  /// Add a ball
  void applyKick(double ball_real_x, double ball_real_y,
                 double kick_power, double kick_theta,
                 double * ball_final_x, double * ball_final_y,
                 std::default_random_engine * engine) const;

  /// Has a goal been scored?
  bool isGoal(double src_x, double src_y, double dst_x, double dst_y) const;

  /// Is the player inside of the goal area
  bool isGoalArea(double player_x, double player_y) const;


  /// #KICK PROPERTIES
  /// Average distance of the minimum shoot [m]
  double kick_power_min;
  /// Average distance of the maximum shoot [m]
  double kick_power_max;
  /// Relative noise on the kick: traveled_dist = kick_power * (1 - uniform_rand(-kpn,kpn))
  double kick_dist_rel_noise;
  /// Angular noise on the kick [rad]: real_theta = theoric_theta + uniform_rand(-kdn, kdn)
  double kick_direction_noise;
  /// Distance between the ball and the robot when the robot is kicking [m]
  double kick_range;
  /// Amplitude of the initial noise on the ball position [m]
  double kick_initial_noise;

  /// #TRANSITION FUNCTION
  /// The problem used to simulate the transitions
  PolarApproach polar_approach;
  /// The policy used for the approach problem
  /// S: (ball_dist, ball_dir, target_angle, last_step_x, last_step_y, last_step_theta)
  /// A: (dstep_x, dstep_y, dstep_theta)
  std::unique_ptr<csa_mdp::Policy> approach_policy;

  /// #REWARD FUNCTION
  /// The reward of a kick
  double kick_reward;
  /// Goal reward
  double goal_reward;
  /// Approach step reward
  double approach_step_reward;
  /// Failure reward (ball out of field or goalkeeper collision)
  double failure_reward;

  /// #FIELD PROPERTIES
  /// Total width of the field [m]
  double field_width;
  /// Total length of the field [m]
  double field_length;
  /// Goal width [m]
  double goal_width;

  /// #GOALKEEPER PROPERTIES
  /// Goal area: size along x-axis [m]
  double goal_area_size_x;
  /// Goal area: size along y-axis [m]
  double goal_area_size_y;
  /// Success rate of the goalkeeper once player entered his area [0,1]
  double goalkeeper_success_rate;
};

}
