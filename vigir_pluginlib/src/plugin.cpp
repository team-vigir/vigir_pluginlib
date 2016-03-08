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
  int status = 0;

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

Plugin::Plugin(const std::string& name, const std::string& type_class_package, const std::string& base_class_package, const std::string& base_class)
{
  msgs::PluginDescription description;
  description.name.data = name;
  description.type_class_package.data = type_class_package;
  description.base_class_package.data = base_class_package;
  description.base_class.data = base_class;
  updateDescription(description);
}

Plugin::~Plugin()
{
}

bool Plugin::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  plugin_nh_ = nh;
  return loadParams(params);
}

const msgs::PluginDescription& Plugin::getDescription() const
{
  if (description_.type_class.data.empty())
  {
    (const_cast<Plugin*>(this))->description_.type_class.data = _typeClass(this);
    (const_cast<Plugin*>(this))->updateDescription(description_);
  }
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
  {
    (const_cast<Plugin*>(this))->description_.type_class.data = _typeClass(this);
    (const_cast<Plugin*>(this))->updateDescription(description_);
  }
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

void Plugin::updateDescription(const msgs::PluginDescription& description)
{
  description_ = description;

  rosparam_handler_.reset(new vigir_generic_params::RosparamHandler(description_.private_param_ns.data + "/params", description_.public_param_ns.data + "/params", plugin_nh_));
}
} // namespace
