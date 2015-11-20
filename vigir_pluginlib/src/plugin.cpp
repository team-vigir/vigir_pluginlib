#include <vigir_pluginlib/plugin.h>

namespace vigir_pluginlib
{
Plugin::Plugin(const std::string& name, const std::string& type_id, const vigir_generic_params::ParameterSet& params)
  : name(name)
  , type_id(type_id)
{
  description.name.data = name;
  loadParams(params);
}

Plugin::Plugin(const std::string& name, const std::string& type_id)
  : Plugin(name, type_id, vigir_generic_params::ParameterManager::getActive())
{
}

Plugin::~Plugin()
{
}

bool Plugin::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  root_nh = nh;

  try
  {
    plugin_nh = ros::NodeHandle(nh, name);
  }
  catch (std::exception& /*e*/)
  {
    plugin_nh = nh;
    ROS_DEBUG("[Plugin] initialize: No private namespace found for plugin with name '%s'. Defaulting to root namespace '%s'.", name.c_str(), root_nh.getNamespace().c_str());
  }

  loadParams(params);
  return true;
}

void Plugin::loadParams(const vigir_generic_params::ParameterSet& /*params*/)
{
}

const msgs::PluginDescription& Plugin::getDescription() const
{
  return description;
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
