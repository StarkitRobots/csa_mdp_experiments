#include "mre_machine/mre_machine_controller.h"

#include "interface/interface.h"

#include "problems/control_problem.h"

MREMachineController::Config::Config()
{
}

std::string MREMachineController::Config::class_name() const
{
  return "Config";
}

void MREMachineController::Config::to_xml(std::ostream &out) const
{
  MREMachine::Config::to_xml(out);
  control_config.write("control", out);
}

void MREMachineController::Config::from_xml(TiXmlNode *node)
{
  MREMachine::Config::from_xml(node);
  control_config.read(node, "control");
}

MREMachineController::MREMachineController(std::shared_ptr<MREMachine::Config> config)
  : MREMachine(config), rate(1.0), broken_connection(false)
{
  Config * conf = dynamic_cast<Config *>(config.get());
  effectors       = conf->control_config.effectors;
  linear_sensors  = conf->control_config.linear_sensors;
  angular_sensors = conf->control_config.angular_sensors;

  rate = ros::Rate(conf->control_config.frequency);

  prepare_logs.open("prepare_logs.csv");
}

MREMachineController::~MREMachineController()
{
  prepare_logs.close();
}

bool MREMachineController::alive()
{
  return ros::ok() && !broken_connection;
}

void MREMachineController::init()
{
  MREMachine::init();
  writeRunLogHeader(prepare_logs);
  Config * conf = dynamic_cast<Config *>(config.get());
  ros::NodeHandle nh;
  bridge = std::unique_ptr<ControlBridge>(new ControlBridge(nh,
                                                            effectors,
                                                            "/" + conf->control_config.robot + "/"));
  listener = std::unique_ptr<JointListener>(new JointListener(nh,
                                                              "/" + conf->control_config.robot + "/joint_states"));

  while(alive())
  {
    ros::spinOnce();
    auto joints = listener->getStatus();
    try
    {
      // Analyzing state (exception if thrown if something goes wrong)
      current_state = joints2State(joints, conf->control_config);
      // If state was properly read, break the loop
      break;
    }
    catch (const std::out_of_range &exc)
    {
      std::cerr << exc.what() << std::endl;
    }
    rate.sleep();
  }
}

void MREMachineController::prepareRun()
{
  ControlProblem * control_problem = dynamic_cast<ControlProblem *>(problem.get());
  int prepare_step = 0;
  rate.reset();//Updating value might have used some time, need to update
  while (alive() && (prepare_step == 0 || !control_problem->isValidStart(current_state)))
  {
    Eigen::VectorXd last_state = current_state;
    Eigen::VectorXd cmd = control_problem->getResetCmd(current_state);
    applyAction(cmd);
    writeRunLog(prepare_logs, run, prepare_step, last_state, cmd, current_reward);
    prepare_step++;
  }
}

void MREMachineController::endRun()
{
  ControlProblem * control_problem = dynamic_cast<ControlProblem *>(problem.get());
  MREMachine::endRun();
  applyAction(control_problem->getNeutralCmd());
}

void MREMachineController::applyAction(const Eigen::VectorXd &action)
{
  // Send command to motors
  std::map<std::string, double> motors_orders;
  for (int i = 0; i < action.rows(); i++)
  {
    std::string motor_name = effectors[i];
    motors_orders[motor_name] = action(i);
  }
  bridge->send(motors_orders);
  // Sleep if necessary
  rate.sleep();
  // Treating messages
  ros::spinOnce();
  // Reading state
  Eigen::VectorXd last_state = current_state;
  auto joints = listener->getStatus();
  try
  {
    Config * conf = dynamic_cast<Config *>(config.get());
    current_state = joints2State(joints, conf->control_config);
  }
  // Sensors were missing
  catch (const std::out_of_range &exc)
  {
    std::cerr << exc.what() << std::endl;
    std::cerr << "Connection has been lost" << std::endl;
    broken_connection = true;
  }
  current_reward = problem->getReward(last_state, action, current_state);
}
