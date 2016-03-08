#include <vigir_pluginlib/plugin_loader_base.h>



namespace vigir_pluginlib
{
PluginLoaderBase::PluginLoaderBase(std::string package, std::string base_class, std::string /*attrib_name*/, std::vector<std::string> /*plugin_xml_paths*/)
  : base_class_package_(package)
{
}

std::string PluginLoaderBase::getBaseClassPackage() const
{
  return base_class_package_;
}
} // namespace
