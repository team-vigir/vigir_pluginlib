#include <vigir_pluginlib_msgs/pluginlib_msgs.h>

namespace vigir_pluginlib
{
bool isDescriptionMatching(const msgs::PluginDescription& plugin_description, const msgs::PluginDescription& filter)
{
  if (!filter.name.empty() && filter.name != plugin_description.name)
    return false;
  if (!filter.type_class_name.empty() && filter.type_class_name != plugin_description.type_class_name)
    return false;
  if (!filter.type_class_package.empty() && filter.type_class_package != plugin_description.type_class_package)
    return false;
  if (!filter.type_class.empty() && filter.type_class != plugin_description.type_class)
    return false;
  if (!filter.base_class_package.empty() && filter.base_class_package != plugin_description.base_class_package)
    return false;
  if (!filter.base_class.empty() && filter.base_class != plugin_description.base_class)
    return false;

  return true;
}

std::vector<vigir_pluginlib_msgs::PluginDescription> filterDescriptionList(const std::vector<vigir_pluginlib_msgs::PluginDescription>& plugin_descriptions, const msgs::PluginDescription& filter, bool inverse)
{
  std::vector<vigir_pluginlib_msgs::PluginDescription> filter_list;
  filter_list.push_back(filter);
  return filterDescriptionList(plugin_descriptions, filter_list, inverse);
}

std::vector<vigir_pluginlib_msgs::PluginDescription> filterDescriptionList(const std::vector<msgs::PluginDescription>& plugin_descriptions, const std::vector<msgs::PluginDescription>& filter_list, bool inverse)
{
  std::vector<msgs::PluginDescription> result;

  for (const msgs::PluginDescription& plugin_description : plugin_descriptions)
  {
    bool is_matching = false;
    for (const msgs::PluginDescription& filter : filter_list)
    {
      if (isDescriptionMatching(plugin_description, filter))
      {
        is_matching = true;
        break;
      }
    }

    if (!inverse == is_matching)
        result.push_back(plugin_description);
  }

  return result;
}
}
