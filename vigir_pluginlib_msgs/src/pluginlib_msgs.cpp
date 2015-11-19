#include <vigir_pluginlib_msgs/pluginlib_msgs.h>

namespace vigir_pluginlib
{
bool isDescriptionMatching(const msgs::PluginDescription& plugin_description, const msgs::PluginDescription& filter)
{
  if (!filter.name.data.empty() && filter.name.data != plugin_description.name.data)
    return false;
  if (!filter.type_class_package.data.empty() && filter.type_class_package.data != plugin_description.type_class_package.data)
    return false;
  if (!filter.type_class.data.empty() && filter.type_class.data != plugin_description.type_class.data)
    return false;
  if (!filter.base_class_package.data.empty() && filter.base_class_package.data != plugin_description.base_class_package.data)
    return false;
  if (!filter.base_class.data.empty() && filter.base_class.data != plugin_description.base_class.data)
    return false;

  return true;
}

void filterDescriptionList(std::vector<vigir_pluginlib_msgs::PluginDescription>& filtered_plugin_descriptions, const std::vector<vigir_pluginlib_msgs::PluginDescription>& plugin_descriptions, const msgs::PluginDescription& filter, bool inverse)
{
  std::vector<vigir_pluginlib_msgs::PluginDescription> filter_list;
  filter_list.push_back(filter);
  filterDescriptionList(filtered_plugin_descriptions, plugin_descriptions, filter_list, inverse);
}

void filterDescriptionList(std::vector<msgs::PluginDescription>& filtered_plugin_descriptions, const std::vector<msgs::PluginDescription>& plugin_descriptions, const std::vector<msgs::PluginDescription>& filter_list, bool inverse)
{
  filtered_plugin_descriptions.clear();
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
        filtered_plugin_descriptions.push_back(plugin_description);
  }
}
}
