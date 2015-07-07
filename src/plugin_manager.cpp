#include <vigir_pluginlib/plugin_manager.h>
#include <ros/console.h>

namespace vigir_pluginlib
{
using namespace vigir_generic_params;

PluginManager::Ptr PluginManager::singelton = PluginManager::Ptr();

PluginManager::PluginManager()
{
}

PluginManager::~PluginManager()
{
  // prevents warning when ClassLoaders get destroyed
  plugins_by_name.clear();

  for (PluginLoaderBase* loader : Instance()->class_loader)
    delete loader;
  class_loader.clear();
}

PluginManager::Ptr& PluginManager::Instance()
{
   if (!singelton)
      singelton.reset(new PluginManager());
   return singelton;
}

void PluginManager::initTopics(ros::NodeHandle& nh)
{
  Instance()->nh = nh;
//  // subscribe topics
//  Instance()->update_parameter_set_sub = nh.subscribe("params/update_parameter_set", 1, &ParameterManager::updateParameterSet, Instance().get());

//  // start own services
//  Instance()->set_parameter_set_srv = nh.advertiseService("params/set_parameter_set", &ParameterManager::setParameterSetService, Instance().get());
//  Instance()->get_parameter_set_srv = nh.advertiseService("params/get_parameter_set", &ParameterManager::getParameterSetService, Instance().get());
//  Instance()->get_all_parameter_sets_srv = nh.advertiseService("params/get_all_parameter_sets", &ParameterManager::getAllParameterSetsService, Instance().get());
//  Instance()->get_parameter_set_names_srv = nh.advertiseService("params/get_parameter_set_names", &ParameterManager::getParameterSetNamesService, Instance().get());

//  // init action servers
//  Instance()->set_parameter_set_as = SimpleActionServer<msgs::SetParameterSetAction>::create(nh, "params/set_parameter_set", true, boost::bind(&ParameterManager::setParameterSetAction, Instance().get(), boost::ref(Instance()->set_parameter_set_as)));
//  Instance()->get_parameter_set_as = SimpleActionServer<msgs::GetParameterSetAction>::create(nh, "params/get_parameter_set", true, boost::bind(&ParameterManager::getParameterSetAction, Instance().get(), boost::ref(Instance()->get_parameter_set_as)));
//  Instance()->get_all_parameter_sets_as = SimpleActionServer<msgs::GetAllParameterSetsAction>::create(nh, "params/get_all_parameter_sets", true, boost::bind(&ParameterManager::getAllParameterSetsAction, Instance().get(), boost::ref(Instance()->get_all_parameter_sets_as)));
//  Instance()->get_parameter_set_names_as = SimpleActionServer<msgs::GetParameterSetNamesAction>::create(nh, "params/get_parameter_set_names", true, boost::bind(&ParameterManager::getParameterSetNamesAction, Instance().get(), boost::ref(Instance()->get_parameter_set_names_as)));
}

const PluginManager::PluginLoaderVector& PluginManager::getPluginClassLoader()
{
  return Instance()->class_loader;
}

bool PluginManager::addPlugin(const std::string& type, const std::string& base_class)
{
  boost::shared_ptr<Plugin> p;

  try
  {
    std::string _base_class = base_class;

    // search for appropriate ClassLoader
    for (PluginLoaderBase* loader : Instance()->class_loader)
    {
      if (loader->isClassAvailable(type) && (base_class.empty() || base_class == loader->getBaseClassType()))
      {
        if (!p)
        {
          _base_class = loader->getBaseClassType().c_str();
          p = loader->createPluginInstance(type);
        }
        else
          ROS_WARN("[PluginManager] Duplicate source for plugin '%s' found in ClassLoader '%s'!\nPlugin was already instanciated from ClassLoader '%s'", type.c_str(), loader->getBaseClassType().c_str(), _base_class.c_str());
      }
    }
    if (!p)
    {
      ROS_ERROR("[PluginManager] Plugin of type '%s' is unknown! Check if ClassLoader has been initialized!", type.c_str());
      return false;
    }
  }
  catch (pluginlib::PluginlibException& e)
  {
    ROS_ERROR("[PluginManager] Plugin of type '%s' failed to load for some reason. Error: %s", type.c_str(), e.what());
    return false;
  }

  PluginManager::addPlugin(p);
  return true;
}

void PluginManager::addPlugin(Plugin::Ptr plugin)
{
  if (!plugin)
  {
    ROS_ERROR("[PluginManager] Got NULL pointer as plugin. Fix it immediatly!");
    return;
  }

  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.find(plugin->getName());
  Plugin::Ptr unique_plugin;

  if (itr != Instance()->plugins_by_name.end()) // replace by name
    ROS_INFO("[PluginManager] Plugin '%s' with type_id '%s' is replaced by '%s' with type_id '%s'!", itr->second->getName().c_str(), itr->second->getTypeId().c_str(), plugin->getName().c_str(), plugin->getTypeId().c_str());
  else if (plugin->isUnique() && getUniquePluginByTypeId(plugin->getTypeId(), unique_plugin)) // replace by uniqueness
  {
    ROS_INFO("[PluginManager] Unique plugin '%s' with type_id '%s' is replaced by '%s'!", unique_plugin->getName().c_str(), unique_plugin->getTypeId().c_str(), plugin->getName().c_str());
    Instance()->plugins_by_name.erase(Instance()->plugins_by_name.find(unique_plugin->getName())); // prevent outputs by removePlugin call
  }
  else
    ROS_INFO("[PluginManager] Added new plugin '%s' with type_id '%s'", plugin->getName().c_str(), plugin->getTypeId().c_str());

  Instance()->plugins_by_name[plugin->getName()] = plugin;
}

void PluginManager::addPlugin(Plugin* plugin)
{
  Plugin::Ptr plugin_ptr(plugin);
  addPlugin(plugin_ptr);
}

bool PluginManager::getPluginByName(const std::string& name, Plugin::Ptr& plugin)
{
  plugin.reset();

  std::map<std::string, Plugin::Ptr>::const_iterator itr = Instance()->plugins_by_name.find(name);
  if (itr == Instance()->plugins_by_name.end())
    return false;

  plugin = itr->second;
  return true;
}

bool PluginManager::getPluginsByTypeId(const std::string& type_id, std::vector<Plugin::Ptr>& plugins)
{
  plugins.clear();

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (itr->second->getTypeId() == type_id)
      plugins.push_back(itr->second);
  }

  return !plugins.empty();
}

bool PluginManager::getUniquePluginByTypeId(const std::string& type_id, Plugin::Ptr& plugin)
{
  plugin.reset();

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (itr->second->isUnique() && itr->second->getTypeId() == type_id)
    {
      plugin = itr->second;
      return true;
    }
  }

  return false;
}

void PluginManager::removePlugin(Plugin::Ptr& plugin)
{
  removePluginByName(plugin->getName());
}

void PluginManager::removePluginByName(const std::string& name)
{
  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.find(name);
  if (itr == Instance()->plugins_by_name.end())
    return;

  ROS_INFO("[PluginManager] Removed plugin '%s' with type_id '%s'", itr->second->getName().c_str(), itr->second->getTypeId().c_str());
  Instance()->plugins_by_name.erase(itr);
}

void PluginManager::removePluginsByTypeId(const std::string& type_id)
{
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end();)
  {
    if (itr->second->getTypeId() == type_id)
    {
      ROS_INFO("[PluginManager] Removed plugin '%s' with type_id '%s'", itr->second->getName().c_str(), itr->second->getTypeId().c_str());
      Instance()->plugins_by_name.erase(itr++);
    }
    else
      itr++;
  }
}

bool PluginManager::hasPlugin(Plugin::Ptr& plugin)
{
  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.find(plugin->getName());
  return (itr != Instance()->plugins_by_name.end() && itr->second->getTypeId() == plugin->getTypeId());
}

bool PluginManager::hasPluginByName(const std::string& name)
{
  return Instance()->plugins_by_name.find(name) != Instance()->plugins_by_name.end();
}

bool PluginManager::hasPluginsByTypeId(const std::string& type_id)
{
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (itr->second->getTypeId() == type_id)
      return true;
  }
}

void PluginManager::loadParams(const ParameterSet& params)
{
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
    itr->second->loadParams(params);
}

bool PluginManager::initializePlugins(ros::NodeHandle& nh, const ParameterSet& params)
{
  bool result = true;

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (!itr->second->initialize(nh, params))
    {
      result = false;
      ROS_ERROR("[PluginManager] Failed to initialize plugin '%s' with type_id '%s'", itr->second->getName().c_str(), itr->second->getTypeId().c_str());
    }
  }

  return result;
}

bool PluginManager::initializePlugins(ros::NodeHandle& nh)
{
  return initializePlugins(nh, ParameterManager::getActive());
}
}
