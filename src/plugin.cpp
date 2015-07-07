#include <vigir_pluginlib/plugin.h>

namespace vigir_pluginlib
{
Plugin::Plugin(const std::string& name, const std::string& type_id, const vigir_generic_params::ParameterSet& params)
  : name(name)
  , type_id(type_id)
{
  loadParams(params);
}

Plugin::Plugin(const std::string& name, const std::string& type_id)
  : Plugin(name, type_id, vigir_generic_params::ParameterManager::getActive())
{
}

Plugin::~Plugin()
{
}

void Plugin::loadParams(const vigir_generic_params::ParameterSet& /*params*/)
{
}

bool Plugin::initialize(ros::NodeHandle& /*nh*/, const vigir_generic_params::ParameterSet& params)
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
