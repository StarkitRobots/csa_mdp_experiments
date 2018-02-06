#pragma once
#include "learning_machine/learning_machine.h"

#include "problems/control_problem.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

namespace csa_mdp
{

class LearningMachineController : public LearningMachine
{
public:
  LearningMachineController();
  virtual ~LearningMachineController();

  virtual bool alive() override;

  virtual void openStreams() override;
  virtual void closeActiveStreams() override;

  /// Require to launch ros
  virtual void init() override;

  virtual void prepareRun() override;
  virtual void endRun() override;
  virtual void applyAction(const Eigen::VectorXd &action) override;
  
  virtual void setProblem(std::unique_ptr<csa_mdp::Problem> problem) override;

  virtual std::string getClassName() const override;
  void toJson(std::ostream &out) const override;
  void fromJson(TiXmlNode *node) override;

private:
  std::unique_ptr<ControlBridge> bridge;
  std::unique_ptr<JointListener> listener;
  std::unique_ptr<ros::Rate> rate;

  std::ofstream prepare_logs;

  bool broken_connection;

  /// Configuration of the controller
  rhoban_control::ControlConfig control_config;

  /// Access to another type of problem
  std::shared_ptr<ControlProblem> ctrl_problem;
};

}
