#include "mre_machine/mre_machine_factory.h"

#include "mre_machine/mre_machine_blackbox.h"
#include "mre_machine/mre_machine_controller.h"

static MREMachine * buildBlackBox(TiXmlNode * node)
{
  TiXmlNode * mre_machine_node = node->FirstChild("mre_machine");
  if(!mre_machine_node) throw std::runtime_error("Failed to find node 'mre_machine'");
  MREMachine::Config * config = new MREMachine::Config();
  config->from_xml(mre_machine_node);
  return new MREMachineBlackBox(std::shared_ptr<MREMachine::Config>(config));
}

static MREMachine * buildControl(TiXmlNode * node)
{
  TiXmlNode * mre_machine_node = node->FirstChild("mre_machine");
  if(!mre_machine_node) throw std::runtime_error("Failed to find node 'mre_machine'");
  MREMachine::Config * config = new MREMachineController::Config();
  config->from_xml(mre_machine_node);
  return new MREMachineController(std::shared_ptr<MREMachine::Config>(config));
}

MREMachineFactory::MREMachineFactory()
{
  registerBuilder("blackbox", &buildBlackBox);
  registerBuilder("control" , &buildControl);
}
