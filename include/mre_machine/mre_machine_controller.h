#include "mre_machine/mre_machine.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

class MREMachineController : public MREMachine
{
public:
  MREMachineController(std::shared_ptr<MREMachine::Config> config);
  virtual ~MREMachineController();

  virtual bool alive() override;

  /// Require to launch ros
  virtual void init() override;

  virtual void prepareRun() override;
  virtual void endRun() override;
  virtual void applyAction(const Eigen::VectorXd &action) override;


  class Config : public MREMachine::Config
  {
  public:
    Config();

    virtual std::string class_name() const override;
    void to_xml(std::ostream &out) const override;
    void from_xml(TiXmlNode *node) override;

    rosban_control::ControlConfig control_config;
  };

private:
  std::vector<std::string> effectors;
  std::vector<std::string> linear_sensors;
  std::vector<std::string> angular_sensors; 

  std::unique_ptr<ControlBridge> bridge;
  std::unique_ptr<JointListener> listener;
  ros::Rate rate;

  std::ofstream prepare_logs;

  bool broken_connection;
};
