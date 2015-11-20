#include <vigir_pluginlib/plugin.h>

namespace vigir_pluginlib
{
Plugin::Plugin(const msgs::PluginDescription& description, const vigir_generic_params::ParameterSet& params)
  : description(description)
{
  loadParams(params);
}

Plugin::Plugin(const msgs::PluginDescription& description)
  : description(description)
{
}

Plugin::Plugin(const std::string& name, const std::string& type_class, const vigir_generic_params::ParameterSet& params)
{
  description.name.data = name;
  description.type_class.data = type_class;
  loadParams(params);
}

Plugin::Plugin(const std::string& name, const std::string& type_class)
{
  description.name.data = name;
  description.type_class.data = type_class;
}

Plugin::~Plugin()
{
}

bool Plugin::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  root_nh = nh;

  try
  {
    plugin_nh = ros::NodeHandle(nh, getName().c_str());
  }
  catch (std::exception& /*e*/)
  {
    plugin_nh = nh;
    ROS_DEBUG("[Plugin] initialize: No private namespace found for plugin with name '%s'. Defaulting to root namespace '%s'.", getName().c_str(), root_nh.getNamespace().c_str());
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
  return description.name.data;
}

const std::string& Plugin::getTypeClassPackage() const
{
  return description.type_class_package.data;
}

const std::string& Plugin::getTypeClass() const
{
  return description.type_class.data;
}

const std::string& Plugin::getBaseClassPackage() const
{
  return description.base_class_package.data;
}

const std::string& Plugin::getBaseClass() const
{
  return description.base_class.data;
}

bool Plugin::isUnique() const
{
  return true;
}
}
