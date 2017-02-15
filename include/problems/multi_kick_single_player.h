#pragma once

#include "problems/polar_approach.h"
#include "kick_model/kick_model.h"

#include "rosban_csa_mdp/core/black_box_problem.h"

#include "rosban_csa_mdp/core/policy.h"

#include <memory>

namespace csa_mdp
{

/// In this problem, a humanoid robot plays alone versus empty goals. It has
/// to choose between several kick options. Each action consist of walking
/// toward the ball, and then applying the chosen kick.
///
/// Referentials:
/// - x axis: from our goal to the opposite goal
/// - y axis: moving laterally on the field (z aims to the roof of the field)
/// - (0,0) is the center of the field
///
/// The state space is the following:
/// 0. ball_x
/// 1. ball_y
/// 2. robot_x
/// 3. robot_y
/// 4. robot_theta
///
/// The action space is the following
/// 0. Kick type
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
/// 3. Simulate the player approach using the appropriate policy 
/// 4. Randomized shoot:
///    - *ball_final* is computed from *ball_real* using the kick_model and the parameters
/// 5. If *ball_final* is outside of the field, the same process as in 2. is used
class MultiKickSinglePlayer : public BlackBoxProblem
{
public:
  /// Several kick options might be available, but each kick options has its own:
  /// - Kick model     : To determine the eventual results
  /// - Approach model : To determine in which states it is allowable to kick
  /// - Approach policy: To choose the orders sent to the walk when simulating the approach
  class KickOption : rosban_utils::Serializable {
  public:
    std::unique_ptr<KickModel> kick_model;
    PolarApproach approach_model;
    /// The policy used for the approach problem
    /// S: (ball_dist, ball_dir, target_angle, last_step_x, last_step_y, last_step_theta)
    /// A: (dstep_x, dstep_y, dstep_theta)
    std::unique_ptr<csa_mdp::Policy> approach_policy;

    void to_xml(std::ostream & out) const override;
    void from_xml(TiXmlNode * node) override;
    std::string class_name() const override;
  };

public:
  MultiKickSinglePlayer();

  /// cf class description
  Problem::Result getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  /// At starting state:
  /// Player is placed randomly on the field
  /// Ball is placed randomly
  Eigen::VectorXd getStartingState(std::default_random_engine * engine) const override;

  void performApproach(const KickOption & kick_option,
                       Problem::Result * status,
                       double kick_theta_field,
                       std::default_random_engine * engine) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

private:
  /// Ensure that the maximal distance for approaches is high enough
  void updateApproachesLimits();

  /// Update the actions limits according to the kick options
  void updateActionsLimits();

  /// Add a ball
  void initialBallNoise(double ball_x, double ball_y,
                        double * ball_real_x, double * ball_real_y,
                        std::default_random_engine * engine) const;

  /// Examine the result of moving the ball from 'src' to 'dst'
  /// If one of these 3 events happen:
  /// - Ball collides with the goalie
  /// - Ball enters the goal
  /// - Ball leaves the field
  /// then:
  /// - dst is updated
  /// - terminal value is set to true
  /// - reward is updated (a reward is accorded according to the event)
  void moveBall(double src_x, double src_y,
                double * dst_x, double * dst_y,
                bool * terminal, double * reward) const;

  /// Is the given kick colliding the goalie ?
  /// if colliding: return true and modifies dst_x and dst_y
  bool isCollidingGoalie(double src_x, double src_y,
                         double * dst_x, double * dst_y) const;

  /// Is the given kick resulting with a goal?
  /// If goal: return true and modifies dst_x and dst_y
  bool isGoal(double src_x, double src_y,
              double * dst_x, double * dst_y) const;


  /// Is the ball outside of the field after the given kick
  /// If ball is outside: return true and modifies dst_x and dst_y
  /// src is expected to be inside the field
  bool isOut(double src_x, double src_y,
             double * dst_x, double * dst_y) const;

  /// Is the player inside of the goal area
  bool isGoalArea(double player_x, double player_y) const;

  /// Extract a polar_approach state from a one_player_kick state and the wished
  /// direction for the kick. The speed of the robot is considered as 0
  Eigen::VectorXd toPolarApproachState(const Eigen::VectorXd & mksp_state,
                                       double kick_theta_field) const;

  /// Extract a one_player_kick state from a polar_approach state
  Eigen::VectorXd toMultiKickSinglePlayerState(const Eigen::VectorXd & pa_state,
                                               double ball_x, double ball_y,
                                               double kick_theta_field) const;
  /// #TRANSITION FUNCTION
  /// Which KickOptions are available?
  std::vector<std::unique_ptr<KickOption>> kick_options;

  /// Noise added on ball position at the beginning of every step
  double step_initial_noise;

  /// #REWARD FUNCTION
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
  /// Ball radius [m]
  double ball_radius;

  /// #GOALKEEPER PROPERTIES
  /// Goal area: size along x-axis [m]
  double goal_area_size_x;
  /// Goal area: size along y-axis [m]
  double goal_area_size_y;
  /// Success rate of the goalkeeper once player entered his area [0,1]
  double goalkeeper_success_rate;
  /// Goalie position x [m]
  double goalie_x;
  /// Goalie position y [m]
  double goalie_y;
  /// Goalie size along x-axis
  double goalie_thickness;
  /// Goalie size along y-axis
  double goalie_width;
};

}
