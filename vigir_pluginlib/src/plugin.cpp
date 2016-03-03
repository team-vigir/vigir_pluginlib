#include <vigir_pluginlib/plugin.h>

namespace vigir_pluginlib
{
Plugin::Plugin(const msgs::PluginDescription& description)
  : description_(description)
{
}

Plugin::Plugin(const std::string& name, const std::string& type_class)
{
  description_.name.data = name;
  description_.type_class.data = type_class;
}

Plugin::~Plugin()
{
}

bool Plugin::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  try
  {
    plugin_nh_ = ros::NodeHandle(nh, getName().c_str());
  }
  catch (std::exception& /*e*/)
  {
    plugin_nh_ = nh;
    ROS_DEBUG("[Plugin] initialize: No private namespace found for plugin with name '%s'. Defaulting to root namespace '%s'.", getName().c_str(), root_nh_.getNamespace().c_str());
  }

  loadParams(params);

  return true;
}

void Plugin::loadParams(const vigir_generic_params::ParameterSet& /*params*/)
{
}

const msgs::PluginDescription& Plugin::getDescription() const
{
  return description_;
}

const std::string& Plugin::getName() const
{
  return description_.name.data;
}

const std::string& Plugin::getTypeClassPackage() const
{
  return description_.type_class_package.data;
}

const std::string& Plugin::getTypeClass() const
{
  return description_.type_class.data;
}

const std::string& Plugin::getBaseClassPackage() const
{
  return description_.base_class_package.data;
}

const std::string& Plugin::getBaseClass() const
{
  return description_.base_class.data;
}
}
