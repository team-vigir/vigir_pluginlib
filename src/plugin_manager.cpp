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

  // subscribe topics
  Instance()->add_plugin_sub = nh.subscribe("plugin_manager/add_plugin", 1, &PluginManager::addPlugin, Instance().get());
  Instance()->remove_plugin_sub = nh.subscribe("plugin_manager/remove_plugin", 1, &PluginManager::removePlugin, Instance().get());

  // start own services
  Instance()->get_plugin_descriptions_srv = nh.advertiseService("plugin_manager/get_plugin_descriptions", &PluginManager::getPluginDescriptionsService, Instance().get());
  Instance()->get_plugin_status_srv = nh.advertiseService("plugin_manager/get_plugin_status", &PluginManager::getPluginStatusService, Instance().get());
  Instance()->add_plugin_srv = nh.advertiseService("plugin_manager/add_plugin", &PluginManager::addPluginService, Instance().get());
  Instance()->remove_plugin_srv = nh.advertiseService("plugin_manager/remove_plugin", &PluginManager::removePluginService, Instance().get());

  // init action servers
  Instance()->get_plugin_descriptions_as.reset(new GetPluginDescriptionsActionServer(nh, "plugin_manager/get_plugin_descriptions", boost::bind(&PluginManager::getPluginDescriptionsAction, Instance().get(), _1), false));
  Instance()->get_plugin_status_as.reset(new GetPluginStatusActionServer(nh, "plugin_manager/get_plugin_status", boost::bind(&PluginManager::getPluginStatusAction, Instance().get(), _1), false));
  Instance()->add_plugin_as.reset(new PluginManagementActionServer(nh, "plugin_manager/add_plugin", boost::bind(&PluginManager::addPluginAction, Instance().get(), _1), false));
  Instance()->remove_plugin_as.reset(new PluginManagementActionServer(nh, "plugin_manager/remove_plugin", boost::bind(&PluginManager::removePluginAction, Instance().get(), _1), false));

  // start action servers
  Instance()->get_plugin_descriptions_as->start();
  Instance()->get_plugin_status_as->start();
  Instance()->add_plugin_as->start();
  Instance()->remove_plugin_as->start();
}

const PluginManager::PluginLoaderVector& PluginManager::getPluginClassLoader()
{
  return Instance()->class_loader;
}

bool PluginManager::addPlugin(const std::string& type_class, const std::string& base_class)
{
  boost::shared_ptr<Plugin> p;

  try
  {
    std::string _base_class = base_class;

    // search for appropriate ClassLoader
    for (PluginLoaderBase* loader : Instance()->class_loader)
    {
      if (loader->isClassAvailable(type_class) && (base_class.empty() || base_class == loader->getBaseClassType()))
      {
        if (!p)
        {
          _base_class = loader->getBaseClassType().c_str();
          p = loader->createPluginInstance(type_class);

          PluginDescription d;
          d.base_class_package.data = loader->getBaseClassPackage();
          d.base_class.data = loader->getBaseClassType();
          d.type_class_package.data = loader->getClassPackage(type_class);
          d.type_class.data = type_class;
          p->setDescription(d);
        }
        else
          ROS_WARN("[PluginManager] Duplicate source for plugin '%s' found in ClassLoader '%s'!\nPlugin was already instanciated from ClassLoader '%s'", type_class.c_str(), loader->getBaseClassType().c_str(), _base_class.c_str());
      }
    }
    if (!p)
    {
      ROS_ERROR("[PluginManager] Plugin of type_class '%s' is unknown! Check if ClassLoader has been initialized!", type_class.c_str());
      return false;
    }
  }
  catch (pluginlib::PluginlibException& e)
  {
    ROS_ERROR("[PluginManager] Plugin of type_class '%s' failed to load for some reason. Error: %s", type_class.c_str(), e.what());
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

void PluginManager::getPluginDescriptions(std::vector<PluginDescription>& descriptions, PluginDescription filter)
{
  descriptions.clear();

  for (PluginLoaderBase* loader : Instance()->class_loader)
  {
    PluginDescription description;
    description.base_class_package.data = loader->getBaseClassPackage();
    description.base_class.data = loader->getBaseClassType();

    if (!filter.base_class_package.data.empty() && filter.base_class_package.data != description.base_class_package.data)
      continue;
    if (!filter.base_class.data.empty() && filter.base_class.data != description.base_class.data)
      continue;

    for (std::string type_class : loader->getDeclaredClasses())
    {
      description.type_class_package.data = loader->getClassPackage(type_class);
      description.type_class.data = type_class;

      if (!filter.type_class_package.data.empty() && filter.type_class_package.data != description.type_class_package.data)
        continue;
      if (!filter.type_class.data.empty() && filter.type_class.data != description.type_class.data)
        continue;

      descriptions.push_back(description);
    }
  }
}

void PluginManager::getPluginStatus(std::vector<PluginStatus>& status_list, PluginDescription filter)
{
  status_list.clear();

  for (std::map<std::string, Plugin::Ptr>::const_iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    const Plugin::Ptr& plugin = itr->second;

    PluginStatus status;

    status.header.stamp = ros::Time::now();
    status.name.data = plugin->getName();
    status.description = plugin->getDescription();

    if (!filter.base_class_package.data.empty() && filter.base_class_package.data != status.description.base_class_package.data)
      continue;
    if (!filter.base_class.data.empty() && filter.base_class.data != status.description.base_class.data)
      continue;
    if (!filter.type_class_package.data.empty() && filter.type_class_package.data != status.description.type_class_package.data)
      continue;
    if (!filter.type_class.data.empty() && filter.type_class.data != status.description.type_class.data)
      continue;

    status_list.push_back(status);
  }
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

void PluginManager::loadParams(const vigir_generic_params::ParameterSet& params)
{
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
    itr->second->loadParams(params);
}

bool PluginManager::initializePlugins(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
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

// --- Subscriber calls ---

void PluginManager::addPlugin(const PluginManagementConstPtr& plugin_management_msg)
{
  addPlugin(plugin_management_msg->plugin.type_class.data, plugin_management_msg->plugin.base_class.data);
}

void PluginManager::removePlugin(const PluginManagementConstPtr& plugin_management_msg)
{
  removePluginByName(plugin_management_msg->plugin.type_class.data);
}

// --- Service calls ---

bool PluginManager::getPluginDescriptionsService(GetPluginDescriptionsService::Request& req, GetPluginDescriptionsService::Response& resp)
{
  getPluginDescriptions(resp.plugin_descriptions, req.filter);
  return true;
}

bool PluginManager::getPluginStatusService(GetPluginStatusService::Request& req, GetPluginStatusService::Response& resp)
{
  getPluginStatus(resp.plugin_status, req.filter);
  return true;
}

bool PluginManager::addPluginService(PluginManagementService::Request& req, PluginManagementService::Response& resp)
{
  return addPlugin(req.plugin.type_class.data, req.plugin.base_class.data);
}

bool PluginManager::removePluginService(PluginManagementService::Request& req, PluginManagementService::Response& resp)
{
  removePluginByName(req.plugin.type_class.data);
  return true;
}

// --- Action Server calls ---

void PluginManager::getPluginDescriptionsAction(const GetPluginDescriptionsGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (get_plugin_descriptions_as->isPreemptRequested())
  {
    get_plugin_descriptions_as->setPreempted();
    return;
  }

  GetPluginDescriptionsResult result;
  getPluginDescriptions(result.plugin_descriptions, goal->filter);

  get_plugin_descriptions_as->setSucceeded(result);
}

void PluginManager::getPluginStatusAction(const GetPluginStatusGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (get_plugin_status_as->isPreemptRequested())
  {
    get_plugin_status_as->setPreempted();
    return;
  }

  GetPluginStatusResult result;
  getPluginStatus(result.plugin_status, goal->filter);

  get_plugin_status_as->setSucceeded(result);
}

void PluginManager::addPluginAction(const PluginManagementGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (add_plugin_as->isPreemptRequested())
  {
    add_plugin_as->setPreempted();
    return;
  }

  PluginManagementResult result;
  result.success.data = addPlugin(goal->plugin.type_class.data, goal->plugin.base_class.data);

  add_plugin_as->setSucceeded(result);
}

void PluginManager::removePluginAction(const PluginManagementGoalConstPtr& goal)
{
  // check if new goal was preempted in the meantime
  if (remove_plugin_as->isPreemptRequested())
  {
    remove_plugin_as->setPreempted();
    return;
  }

  PluginManagementResult result;
  result.success.data = true;
  removePluginByName(goal->plugin.type_class.data);

  remove_plugin_as->setSucceeded(result);
}
}
