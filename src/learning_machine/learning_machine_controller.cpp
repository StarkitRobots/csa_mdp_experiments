#include "learning_machine/learning_machine_controller.h"

#include "interface/interface.h"

#include "problems/control_problem.h"

namespace csa_mdp
{

LearningMachineController::LearningMachineController()
  : broken_connection(false)
{
}

LearningMachineController::~LearningMachineController() {}

bool LearningMachineController::alive()
{
  return ros::ok() && !broken_connection;
}

void LearningMachineController::openStreams()
{
  LearningMachine::openStreams();
  //TODO make it optional with a parameter
  prepare_logs.open("prepare_logs.csv");
}

void LearningMachineController::closeActiveStreams()
{
  LearningMachine::closeActiveStreams();
  prepare_logs.close();
}

void LearningMachineController::init()
{
  LearningMachine::init();

  rate = std::unique_ptr<ros::Rate>(new ros::Rate(control_config.frequency));

  writeRunLogHeader(prepare_logs);

  ros::NodeHandle nh;
  bridge = std::unique_ptr<ControlBridge>(new ControlBridge(nh,
                                                            control_config.effectors,
                                                            "/" + control_config.robot + "/"));
  listener = std::unique_ptr<JointListener>(new JointListener(nh,
                                                              "/" + control_config.robot + "/joint_states"));

  while(alive())
  {
    ros::spinOnce();
    auto joints = listener->getStatus();
    try
    {
      // Analyzing state (exception if thrown if something goes wrong)
      current_state = joints2State(joints, control_config);
      // If state was properly read, break the loop
      break;
    }
    catch (const std::out_of_range &exc)
    {
      std::cerr << exc.what() << std::endl;
    }
    rate->sleep();
  }
}

void LearningMachineController::prepareRun()
{
  LearningMachine::prepareRun();
  int prepare_step = 0;
  rate->reset();//Updating value might have used some time, need to update
  while (alive() && (prepare_step == 0 || !ctrl_problem->isValidStart(current_state)))
  {
    Eigen::VectorXd last_state = current_state;
    Eigen::VectorXd cmd = ctrl_problem->getResetCmd(current_state);
    applyAction(cmd);
    writeRunLog(prepare_logs, run, prepare_step, last_state, cmd, current_reward);
    prepare_step++;
  }
}

void LearningMachineController::endRun()
{
  LearningMachine::endRun();
  applyAction(ctrl_problem->getNeutralCmd());
}

void LearningMachineController::applyAction(const Eigen::VectorXd &action)
{
  // Send command to motors
  std::map<std::string, double> motors_orders;
  for (int i = 0; i < action.rows(); i++)
  {
    std::string motor_name = control_config.effectors[i];
    motors_orders[motor_name] = action(i);
  }
  bridge->send(motors_orders);
  // Sleep if necessary
  rate->sleep();
  // Treating messages
  ros::spinOnce();
  // Reading state
  Eigen::VectorXd last_state = current_state;
  auto joints = listener->getStatus();
  try
  {
    current_state = joints2State(joints, control_config);
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

void LearningMachineController::setProblem(std::unique_ptr<csa_mdp::Problem> new_problem)
{
  // Apply everything from the parent class
  LearningMachine::setProblem(std::move(new_problem));
  // Custom task
  ctrl_problem = std::dynamic_pointer_cast<ControlProblem>(problem);
  if (!ctrl_problem) {
    throw std::logic_error("Trying to run a LearningMachineController on a NOT controller problem");
  }
}

std::string LearningMachineController::getClassName() const
{
  return "LearningMachineController";
}

void LearningMachineController::toJson(std::ostream &out) const
{
  LearningMachine::toJson(out);
  control_config.write("control", out);
}

void LearningMachineController::fromJson(TiXmlNode *node)
{
  LearningMachine::fromJson(node);
  control_config.read(node, "control");
}

}
