#include <vigir_pluginlib/plugin.h>

#include <vigir_pluginlib/type_class_traits.h>



namespace vigir_pluginlib
{
Plugin::Plugin(const std::string& name, const std::string& type_class_package, const std::string& base_class_package, const std::string& base_class)
{
  description_.name = name;
  description_.type_class_package = type_class_package;
  description_.base_class_package = base_class_package;
  description_.base_class = base_class;
}

Plugin::~Plugin()
{
  nh_.shutdown();
}

bool Plugin::setup(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& /*params*/)
{
  setNodehandle(nh);

  return true;
}

const msgs::PluginDescription& Plugin::getDescription() const
{
  if (description_.type_class.empty())
  {
    (const_cast<Plugin*>(this))->description_.type_class = TypeClass::get(this);
    (const_cast<Plugin*>(this))->updateDescription(description_);
  }
  return description_;
}

const std::string& Plugin::getName() const
{
  return description_.name;
}

const std::string& Plugin::getTypeClassPackage() const
{
  return description_.type_class_package;
}

const std::string& Plugin::getTypeClass() const
{
  if (description_.type_class.empty())
  {
    (const_cast<Plugin*>(this))->description_.type_class = TypeClass::get(this);
    (const_cast<Plugin*>(this))->updateDescription(description_);
  }
  return description_.type_class;
}

const std::string& Plugin::getBaseClassPackage() const
{
  return description_.base_class_package;
}

const std::string& Plugin::getBaseClass() const
{
  return description_.base_class;
}

void Plugin::updateDescription(const msgs::PluginDescription& description)
{
  description_ = description;
  setParams(vigir_generic_params::ParameterSet(description_.params));
}

void Plugin::setNodehandle(const ros::NodeHandle& nh)
{
  nh_ = nh;
}

void Plugin::setParams(const vigir_generic_params::ParameterSet& params)
{
  params_ = params;
}
} // namespace
