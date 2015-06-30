#include <vigir_pluginlib/plugin.h>

namespace vigir_pluginlib
{
using namespace vigir_generic_params;

Plugin::Plugin(const std::string& name, const std::string& type_id, const ParameterSet& params)
  : name(name)
  , type_id(type_id)
{
  loadParams(params);
}

Plugin::Plugin(const std::string& name, const std::string& type_id)
  : Plugin(name, type_id, ParameterManager::getActive())
{
}

Plugin::~Plugin()
{
}

void Plugin::loadParams(const ParameterSet& /*params*/)
{
}

bool Plugin::initialize(ros::NodeHandle& /*nh*/, const ParameterSet& params)
{
  loadParams(params);
  return true;
}

const std::string& Plugin::getName() const
{
  return name;
}

const std::string& Plugin::getTypeId() const
{
  return type_id;
}

bool Plugin::isUnique() const
{
  return true;
}
}
