#pragma once

#include "problems/polar_approach.h"
#include "kick_model/kick_decision_model.h"
#include "kick_model/kick_model_collection.h"

#include "rosban_csa_mdp/core/black_box_problem.h"

#include "rosban_csa_mdp/core/policy.h"

#include <memory>

namespace csa_mdp
{

/// In this problem, one or several humanoid robots play versus a static goalie.
/// At each time step, they have to choose:
/// - Which player will kick?
/// - Which type of kick will he use? (Power kick, accurate kick, lateral kick etc...)
/// - What parameters will be used for the kick? (kick_direction etc)
/// - What targets will be used for the non-kicker robots
///   - Target_x, target_y, target_theta
///
/// Referentials:
/// - x axis: from kickers goal to the opposite goal
/// - y axis: moving laterally on the field (z aims to the ceiling)
/// - (0,0) is the center of the field
///
/// The state space is the following:
/// 0    : ball_x
/// 1    : ball_y
/// 2+3*k: position x of robot k
/// 3+3*k: position y of robot k
/// 4+3*k: direction  of robot k
///
/// Actions identifiers:
/// 0-n: Who will kick with what kick options
///
/// TRANSITION FUNCTION:
/// The following transition function is used:
/// 1. The ball is moved from *ball* to *ball_real* by adding noise on the ball position.
///    - This simulates the fact that the position of the ball is unknown
///      - Not perfectly correct, but partial observability is not modeled in this framework
/// 2. If *ball_real* is outside of the field:
///    a) The segment *ball* to *ball_real* intersect with the goal segment
///       - A goal is scored and a specific value is returned (cf. below)
///    b) The segment *ball* to *ball_real* do not intersect with the goal segment
///       - A failure is detected and a specific value is returned (cf. below)
/// 3. Simulate the kicker approach using the appropriate policy
/// 4. Randomized shoot:
///    - *ball_final* is computed from *ball_real* using the kick_model and the parameters
/// 5. Simulate motions of the non-kicker robots
/// 6. If *ball_final* is outside of the field, the same process as in 2. is used
/// TODO: kicker and non-kickers should be simulated at the same time and collisions checked
class KickControler : public BlackBoxProblem
{
public:
  /// Several kick options might be available, but each kick options has its own:
  /// - Kick model     : To determine the eventual results
  /// - Approach model : To determine in which states it is allowable to kick
  /// - Approach policy: To choose the orders sent to the walk when simulating the approach
  class KickOption : rosban_utils::Serializable {
  public:
    //TODO: replace it
    std::unique_ptr<KickDecisionModel> kick_decision_model;
    std::vector<std::string> kick_model_names;
    PolarApproach approach_model;
    /// The policy used for the approach problem
    /// S: (ball_dist, ball_dir, target_angle, last_step_x, last_step_y, last_step_theta)
    /// A: (dstep_x, dstep_y, dstep_theta)
    std::unique_ptr<csa_mdp::Policy> approach_policy;

    void to_xml(std::ostream & out) const override;
    void from_xml(TiXmlNode * node) override;
    std::string class_name() const override;

    void syncKickZones(const KickModelCollection & kmc);
  };

  /// Each player has its own custom configuration
  class Player : rosban_utils::Serializable {
  public:
    /// Allows to identify player more easily
    std::string name;
    /// Different players might have different kicks available
    std::vector<std::unique_ptr<KickOption>> kick_options;
    /// Description of the approach problem used when the robot is not kicking
    PolarApproach navigation_approach;
    /// Description of the policy used when the robot is not kicking
    std::unique_ptr<csa_mdp::Policy> approach_policy;

    void to_xml(std::ostream & out) const override;
    void from_xml(TiXmlNode * node) override;
    std::string class_name() const override;
  };

public:
  KickControler();

  /// cf class description
  Problem::Result getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  /// At starting state:
  /// Players are placed randomly on the field
  /// Ball is placed randomly
  /// NOTE: Further development might consider a more complex initialization to
  ///       be more faire between problems with a different number of players.
  ///       e.g: One of the player start at a distance drawn from [0,max_dist]
  ///            and the others start at a random position but above max_dist
  Eigen::VectorXd getStartingState(std::default_random_engine * engine) const override;


  /// Run up to 'max_steps' of simulation on all players using the provided action.
  /// if kicker is enabled, cost is increased according to the number of steps and the
  /// step cost.
  /// Execution is interrupted if:
  /// - one of the robot fails (inside goalArea)
  /// - Kicker is enabled and has reached target
  void runSteps(int max_steps,
                const Eigen::VectorXd & action,
                int kicker_id,
                int kick_option,
                bool kicker_enabled,
                Problem::Result * status,
                std::default_random_engine * engine) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

  size_t getNbPlayers() const;

  /// Import kicker_id and kick_id from action_id
  void analyzeActionId(int action_id, int * kicker_id, int * kick_id) const;

  /// Get kick direction [rad] (in field basis)
  double getKickDir(const Eigen::VectorXd & state,
                    const Eigen::VectorXd & action) const;

  /// Return the names of kicks allowed for the given kick_option
  const std::vector<std::string> & getAllowedKicks(int kicker_id,
                                                   int kick_option);

private:
  /// Return the limits for the field (row1: field_x, row2: field_y)
  Eigen::Matrix<double,2,2> getFieldLimits() const;
  /// Return the limits for the field (row1: field_x, row2: field_y, row3: orientation)
  Eigen::Matrix<double,3,2> getPlayerLimits() const;

  /// Update state limits and names
  void updateStateLimits();

  /// Ensure that the maximal distance for approaches is high enough
  void updateApproachesLimits();

  /// Update the actions limits according to the player used
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

  Eigen::Vector3d getPlayerState(const Eigen::VectorXd & state,
                                 int player_id) const;

  Eigen::Vector3d getTarget(const Eigen::VectorXd & state,
                            const Eigen::VectorXd & action,
                            int player_id,
                            int kicker_id,
                            int kick_option) const;

  /// Extract a polar_approach state from the given player state and the given target
  /// Current speeds of the robots are set to [0,0,0]
  Eigen::VectorXd toPolarApproachState(const Eigen::Vector3d & player_state,
                                       const Eigen::Vector3d & target) const;

  /// Return an updated version
  Eigen::VectorXd toKickControlerState(const Eigen::VectorXd & pa_state,
                                       const Eigen::VectorXd & prev_kc_state,
                                       int player_id,
                                       const Eigen::Vector3d & target) const;
  /// #TRANSITION FUNCTION
  /// Which KickOptions are available?
  std::vector<std::unique_ptr<Player>> players;

  /// Noise added on ball position at the beginning of every step
  double step_initial_stddev;

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

  /// The collection of available kicks
  KickModelCollection kmc;
};

}
