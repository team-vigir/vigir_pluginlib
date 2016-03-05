#include <vigir_pluginlib/plugin.h>

namespace vigir_pluginlib
{
#ifdef __GNUG__
#include <cstdlib>
#include <memory>
#include <cxxabi.h>

// enable c++11 by passing the flag -std=c++11 to g++
std::string demangle(const char* name)
{
  int status = -4; // some arbitrary value to eliminate the compiler warning

  std::unique_ptr<char, void(*)(void*)> res
  {
    abi::__cxa_demangle(name, NULL, NULL, &status),
    std::free
  };

  return (status==0) ? res.get() : name ;
}

#else

// does nothing if not g++
std::string demangle(const char* name)
{
  return name;
}

#endif

Plugin::Plugin(const msgs::PluginDescription& description)
  : description_(description)
{
}

Plugin::Plugin(const std::string& name)
{
  description_.name.data = name;
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
  if (description_.type_class.data.empty())
    (const_cast<Plugin*>(this))->description_.type_class.data = _typeClass(this);
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
  if (description_.type_class.data.empty())
    (const_cast<Plugin*>(this))->description_.type_class.data = _typeClass(this);
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
