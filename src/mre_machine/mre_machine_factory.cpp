#include "mre_machine/mre_machine_factory.h"

#include "mre_machine/mre_machine_blackbox.h"
#include "mre_machine/mre_machine_controller.h"

static std::unique_ptr<MREMachine> buildBlackBox(TiXmlNode * node)
{
  TiXmlNode * mre_machine_node = node->FirstChild("mre_machine");
  if(!mre_machine_node) throw std::runtime_error("Failed to find node 'mre_machine'");
  MREMachine::Config * config = new MREMachine::Config();
  config->from_xml(mre_machine_node);
  std::shared_ptr<MREMachine::Config> machine_config(config);
  MREMachineBlackBox * machine = new MREMachineBlackBox(machine_config);
  return std::unique_ptr<MREMachineBlackBox>(machine);
}

static std::unique_ptr<MREMachine> buildControl(TiXmlNode * node)
{
  TiXmlNode * mre_machine_node = node->FirstChild("mre_machine");
  if(!mre_machine_node) throw std::runtime_error("Failed to find node 'mre_machine'");
  MREMachine::Config * config = new MREMachineController::Config();
  config->from_xml(mre_machine_node);
  std::shared_ptr<MREMachine::Config> machine_config(config);
  MREMachineController * machine = new MREMachineController(machine_config);
  return std::unique_ptr<MREMachineController>(machine);
}

MREMachineFactory::MREMachineFactory()
{
  registerBuilder("blackbox", &buildBlackBox);
  registerBuilder("control" , &buildControl);
}
