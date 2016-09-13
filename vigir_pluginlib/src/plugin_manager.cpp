#include <vigir_pluginlib/plugin_manager.h>

#include <ros/console.h>



namespace vigir_pluginlib
{
using namespace vigir_generic_params;

PluginManager::Ptr PluginManager::singelton_ = PluginManager::Ptr();

PluginManager::PluginManager()
{
}

PluginManager::Ptr PluginManager::Instance()
{
   if (!singelton_)
      singelton_.reset(new PluginManager());
   return singelton_;
}

PluginManager::~PluginManager()
{
  boost::unique_lock<boost::shared_mutex> lock(plugins_mutex_);

  // prevents warning when ClassLoaders get destroyed
  plugins_by_name_.clear();

  for (PluginLoaderBase* loader : class_loader_)
    delete loader;
  class_loader_.clear();
}

void PluginManager::initialize(ros::NodeHandle& nh)
{
  Instance()->nh_ = nh;

  // subscribe topics
  Instance()->add_plugin_sub_ = nh.subscribe("plugin_manager/add_plugin", 1, &PluginManager::addPlugin, Instance().get());
  Instance()->load_plugin_set_sub_ = nh.subscribe("plugin_manager/load_plugin_set", 1, &PluginManager::loadPluginSet, Instance().get());
  Instance()->remove_plugin_sub_ = nh.subscribe("plugin_manager/remove_plugin", 1, &PluginManager::removePlugin, Instance().get());

  // publish topics
  Instance()->plugin_states_pub_ = nh.advertise<msgs::PluginStates>("plugin_manager/plugin_states_update", 1);

  // start own services
  Instance()->get_plugin_descriptions_srv_ = nh.advertiseService("plugin_manager/get_plugin_descriptions", &PluginManager::getPluginDescriptionsService, Instance().get());
  Instance()->get_plugin_states_srv_ = nh.advertiseService("plugin_manager/get_plugin_states", &PluginManager::getPluginStatesService, Instance().get());
  Instance()->add_plugin_srv_ = nh.advertiseService("plugin_manager/add_plugin", &PluginManager::addPluginService, Instance().get());
  Instance()->remove_plugin_srv_ = nh.advertiseService("plugin_manager/remove_plugin", &PluginManager::removePluginService, Instance().get());
  Instance()->load_plugin_set_srv_ = nh.advertiseService("plugin_manager/load_plugin_set", &PluginManager::loadPluginSetService, Instance().get());

  // init action servers
  Instance()->get_plugin_descriptions_as_.reset(new GetPluginDescriptionsActionServer(nh, "plugin_manager/get_plugin_descriptions", boost::bind(&PluginManager::getPluginDescriptionsAction, Instance().get(), _1), false));
  Instance()->get_plugin_states_as_.reset(new GetPluginStatesActionServer(nh, "plugin_manager/get_plugin_states", boost::bind(&PluginManager::getPluginStatesAction, Instance().get(), _1), false));
  Instance()->add_plugin_as_.reset(new PluginManagementActionServer(nh, "plugin_manager/add_plugin", boost::bind(&PluginManager::addPluginAction, Instance().get(), _1), false));
  Instance()->remove_plugin_as_.reset(new PluginManagementActionServer(nh, "plugin_manager/remove_plugin", boost::bind(&PluginManager::removePluginAction, Instance().get(), _1), false));
  Instance()->load_plugin_set_as_.reset(new PluginManagementActionServer(nh, "plugin_manager/load_plugin_set", boost::bind(&PluginManager::loadPluginSetAction, Instance().get(), _1), false));

  // start action servers
  Instance()->get_plugin_descriptions_as_->start();
  Instance()->get_plugin_states_as_->start();
  Instance()->add_plugin_as_->start();
  Instance()->remove_plugin_as_->start();
  Instance()->load_plugin_set_as_->start();
}

bool PluginManager::autocompletePluginDescriptionByName(const std::string& name, msgs::PluginDescription& plugin_description)
{
  try
  {
    ros::NodeHandle plugin_nh(Instance()->nh_, name);

    std::string type_class_package;
    if (!plugin_nh.getParam("type_class_package", type_class_package))
      return false;

    std::string type_class;
    if (!plugin_nh.getParam("type_class", type_class))
      return false;

    if ((plugin_description.type_class_package.data.empty() || plugin_description.type_class_package.data == type_class_package) &&
        (plugin_description.type_class.data.empty() || plugin_description.type_class.data == type_class))
    {
      plugin_description.type_class_package.data = type_class_package;
      plugin_description.type_class.data = type_class;

      if (plugin_description.base_class_package.data.empty())
        plugin_nh.param("base_class_package", plugin_description.base_class_package.data, std::string());
      if (plugin_description.base_class.data.empty())
        plugin_nh.param("base_class", plugin_description.base_class.data, std::string());

      if (plugin_description.public_param_ns.data.empty())
        plugin_description.public_param_ns.data = name;

      return true;
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Exception while extracting plugin description with name '%s':\n%s", name.c_str(), e.what());
    return false;
  }
}

bool PluginManager::addPlugins(const std::vector<msgs::PluginDescription>& plugin_descriptions, bool initialize)
{
  bool success = true;

  std::list<Plugin::Ptr> plugins;

  // add plugins
  for (const msgs::PluginDescription& description : plugin_descriptions)
  {
    if (addPlugin(description, false))
      plugins.push_back(getPluginByName(description.name.data));
    else
      success = false;
  }

  // (post) intialize plugins
  if (initialize)
  {
    for (Plugin::Ptr plugin : plugins)
      success &= plugin->initialize(ParameterManager::getActive());
    for (Plugin::Ptr plugin : plugins)
      success &= plugin->postInitialize(ParameterManager::getActive());
  }

  return success;
}

bool PluginManager::addPlugin(const msgs::PluginDescription& plugin_description, bool initialize)
{
  // best effort handling for incomplete plugin description
  msgs::PluginDescription description = plugin_description;
  if (description.type_class_package.data.empty() || description.type_class.data.empty())
  {
    if (description.name.data.empty())
    {
      ROS_ERROR("[PluginManager] addPlugin: Call without name (%s) or type class package (%s) and type class (%s)!", description.name.data.c_str(), description.type_class_package.data.c_str(), description.type_class.data.c_str());
      return false;
    }
    else if (!autocompletePluginDescriptionByName(description.name.data, description))
    {
      ROS_ERROR("[PluginManager] addPlugin: Can't autocomplete plugin description for (%s)!", description.name.data.c_str());
      return false;
    }
  }

  // create plugin
  Plugin::Ptr p;

  try
  {
    boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

    std::string _base_class = description.base_class.data;

    // search for appropriate ClassLoader
    for (PluginLoaderBase* loader : Instance()->class_loader_)
    {
      if (loader->isClassAvailable(description.type_class.data) && (description.base_class.data.empty() || description.base_class.data == loader->getBaseClassType()))
      {
        if (description.type_class_package.data != loader->getClassPackage(description.type_class.data))
          continue;
        if (!description.base_class_package.data.empty() && description.base_class_package.data != loader->getBaseClassPackage())
          continue;
        if (!description.base_class.data.empty() && description.base_class.data != loader->getBaseClassType())
          continue;

        if (!p)
        {
          _base_class = loader->getBaseClassType().c_str();
          p = loader->createPluginInstance(description.type_class.data);

          if (description.name.data.empty())
            description.name.data = p->getName();

          p->updateDescription(description);
        }
        else
          ROS_WARN("[PluginManager] Duplicate source for plugin '%s' found in ClassLoader '%s'!\nPlugin was already instanciated from ClassLoader '%s'", description.type_class.data.c_str(), loader->getBaseClassType().c_str(), _base_class.c_str());
      }
    }
    if (!p)
    {
      ROS_ERROR_STREAM("[PluginManager] Plugin with following description is unknown! Check if ClassLoader has been initialized and the plugin has been properly registered!\n" << description);
      return false;
    }
  }
  catch (pluginlib::PluginlibException& e)
  {
    ROS_ERROR("[PluginManager] Plugin (%s) of type_class '%s' failed to load for some reason. Error message: \n %s", description.name.data.c_str(), description.type_class.data.c_str(), e.what());
    return false;
  }

  PluginManager::addPlugin(p, initialize);
  return true;
}

bool PluginManager::addPluginByName(const std::string& name, bool initialize)
{
  msgs::PluginDescription description;
  description.name.data = name;
  return addPlugin(description, initialize);
}

void PluginManager::addPlugin(Plugin* plugin, bool initialize)
{
  Plugin::Ptr plugin_ptr(plugin);
  addPlugin(plugin_ptr, initialize);
}

void PluginManager::addPlugin(Plugin::Ptr plugin, bool initialize)
{
  if (!plugin)
  {
    ROS_ERROR("[PluginManager] addPlugin: Got NULL pointer as plugin. Fix it immediatly!");
    return;
  }

  boost::upgrade_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.find(plugin->getName());
  Plugin::Ptr unique_plugin;

  if (itr != Instance()->plugins_by_name_.end()) // replace by name
  {
    // skip duplicated plugin
    if (isDescriptionMatching(plugin->getDescription(), itr->second->getDescription()))
    {
      ROS_INFO("[PluginManager] addPlugin: Plugin '%s' with type_class '%s' is already added. Skipped.", plugin->getName().c_str(), plugin->getTypeClass().c_str());
      return;
    }
    else
      ROS_INFO("[PluginManager] addPlugin: Plugin '%s' with type_class '%s' is replaced by '%s' with type_class '%s'!", itr->second->getName().c_str(), itr->second->getTypeClass().c_str(), plugin->getName().c_str(), plugin->getTypeClass().c_str());
  }
  else if (plugin->isUnique() && getUniquePluginByTypeClass(plugin->getTypeClass(), unique_plugin)) // replace due to uniqueness
  {
    ROS_INFO("[PluginManager] addPlugin: Unique plugin '%s' with type_class '%s' is replaced by '%s'!", unique_plugin->getName().c_str(), unique_plugin->getTypeClass().c_str(), plugin->getName().c_str());
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    Instance()->plugins_by_name_.erase(Instance()->plugins_by_name_.find(unique_plugin->getName())); // prevent outputs by removePlugin call
  }
  else
    ROS_INFO("[PluginManager] addPlugin: Added new plugin '%s' with type_class '%s'", plugin->getName().c_str(), plugin->getTypeClass().c_str());

  {
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    Instance()->plugins_by_name_[plugin->getName()] = plugin;

    plugin->setup(Instance()->nh_, ParameterManager::getActive());

    if (initialize && !(plugin->initialize(ParameterManager::getActive()) && plugin->postInitialize(ParameterManager::getActive())))
      ROS_ERROR("[PluginManager] addPlugin: Initialization of Plugin '%s' with type_class '%s' failed!", plugin->getName().c_str(), plugin->getTypeClass().c_str());

    Instance()->loaded_plugin_set_.clear();
  }

  // publish update
  Instance()->publishPluginStateUpdate();
}

Plugin::Ptr PluginManager::getPluginByName(const std::string& name)
{
  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  std::map<std::string, Plugin::Ptr>::const_iterator itr = Instance()->plugins_by_name_.find(name);
  if (itr == Instance()->plugins_by_name_.end())
    return Plugin::Ptr();

  return itr->second;
}

bool PluginManager::getPluginByName(const std::string& name, Plugin::Ptr& plugin)
{
  plugin = getPluginByName(name);
  return plugin.get() != nullptr;
}

bool PluginManager::getPluginsByTypeClass(const std::string& type_class, std::vector<Plugin::Ptr>& plugins)
{
  plugins.clear();

  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
  {
    if (itr->second->getTypeClass() == type_class)
      plugins.push_back(itr->second);
  }

  return !plugins.empty();
}

bool PluginManager::getUniquePluginByTypeClass(const std::string& type_class, Plugin::Ptr& plugin)
{
  plugin.reset();

  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
  {
    if (itr->second->isUnique() && itr->second->getTypeClass() == type_class)
    {
      plugin = itr->second;
      return true;
    }
  }

  return false;
}

void PluginManager::getPluginDescriptions(std::vector<msgs::PluginDescription>& descriptions, msgs::PluginDescription filter)
{
  descriptions.clear();

  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  for (PluginLoaderBase* loader : Instance()->class_loader_)
  {
    msgs::PluginDescription description;
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

void PluginManager::getPluginStates(std::vector<msgs::PluginState>& plugin_states, msgs::PluginDescription filter)
{
  plugin_states.clear();

  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  for (std::map<std::string, Plugin::Ptr>::const_iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
  {
    const Plugin::Ptr& plugin = itr->second;

    msgs::PluginState state;

    state.header.stamp = ros::Time::now();
    state.description = plugin->getDescription();

    if (!vigir_pluginlib::isDescriptionMatching(state.description, filter))
      continue;

    plugin_states.push_back(state);
  }
}

bool PluginManager::removePlugins(const std::vector<msgs::PluginDescription>& plugin_descriptions)
{
  bool success = true;
  for (const msgs::PluginDescription& description : plugin_descriptions)
  {
    if (!removePlugin(description))
      success = false;
  }
  return success;
}

bool PluginManager::removePlugin(const msgs::PluginDescription& plugin_description)
{
  if (!plugin_description.name.data.empty())
    return removePluginByName(plugin_description.name.data);
//  else if(!plugin_description.type_class.data.empty())
//    addPlugin(plugin_description.type_class.data, plugin_description.base_class.data);
  else
    ROS_ERROR("[PluginManager] removePlugin: Call without name!");

  return false;
}

bool PluginManager::removePluginByName(const std::string& name)
{
  boost::upgrade_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.find(name);
  if (itr == Instance()->plugins_by_name_.end())
    return false;

  {
    ROS_INFO("[PluginManager] Removed plugin '%s' with type_class '%s'", itr->second->getName().c_str(), itr->second->getTypeClass().c_str());
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    Instance()->plugins_by_name_.erase(itr);

    Instance()->loaded_plugin_set_.clear();
  }

  // publish update
  Instance()->publishPluginStateUpdate();

  return true;
}

void PluginManager::removePlugin(Plugin::Ptr& plugin)
{
  removePluginByName(plugin->getName());
}

void PluginManager::removePluginsByTypeClass(const std::string& type_class)
{
  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end();)
  {
    if (itr->second->getTypeClass() == type_class)
    {
      lock.unlock();
      removePluginByName(itr++->first);
      lock.lock();
    }
    else
      itr++;
  }
}

bool PluginManager::loadPluginSet(const std::vector<msgs::PluginDescription>& plugin_descriptions)
{
  bool success = true;

  // get list of active plugins
  std::vector<msgs::PluginDescription> active_plugins;
  {
    boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);
    for (std::map<std::string, Plugin::Ptr>::const_iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
      active_plugins.push_back(itr->second->getDescription());
  }

  // remove all plugins which are not existing in list
  std::vector<msgs::PluginDescription> remove_plugin_list = filterDescriptionList(active_plugins, plugin_descriptions, true);
  if (!removePlugins(remove_plugin_list))
    success = false;

  // update param namespace of remaining plugins, if changed
  std::list<Plugin::Ptr> updated_plugins;
  std::vector<msgs::PluginDescription> updated_plugin_list = filterDescriptionList(plugin_descriptions, active_plugins);
  for (msgs::PluginDescription description : updated_plugin_list)
  {
    boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);
    Plugin::Ptr& plugin = Instance()->plugins_by_name_[description.name.data];

    plugin->updateDescription(description);
    plugin->setup(Instance()->nh_, ParameterManager::getActive()); // force reload parameters from param server
    updated_plugins.push_back(plugin);
  }

  // add new plugins
  std::vector<msgs::PluginDescription> add_plugin_list = filterDescriptionList(plugin_descriptions, active_plugins, true);
  if (!addPlugins(add_plugin_list))
    success = false;

  // reinitialize plugins which has been updated before
  for (Plugin::Ptr plugin : updated_plugins)
    success &= plugin->initialize(ParameterManager::getActive());
  for (Plugin::Ptr plugin : updated_plugins)
    success &= plugin->postInitialize(ParameterManager::getActive());

  return success;
}

bool PluginManager::loadPluginSet(const std::string& name)
{
  std::string prefix = "plugin_sets/" + name;

  if (!Instance()->nh_.hasParam(prefix))
  {
    ROS_ERROR("[PluginManager] loadPluginSet: Couldn't find plugin set '%s' at parameter server.", name.c_str());
    return false;
  }

  if (Instance()->loaded_plugin_set_ == name)
  {
    ROS_INFO("[PluginManager] loadPluginSet: Plugin set '%s' is already loaded.", name.c_str());
    return true;
  }

  Instance()->loaded_plugin_set_.clear();
  ROS_INFO("[PluginManager] loadPluginSet: Loading plugin set '%s'...", name.c_str());

  // grab all plugin descriptions in the subtree
  std::vector<msgs::PluginDescription> plugin_descriptions;
  XmlRpc::XmlRpcValue val;
  Instance()->nh_.getParam(prefix, val);

  if (val.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    return false;

  for (const auto& kv : val)
  {
    msgs::PluginDescription description;
    description.name.data = kv.first;

    XmlRpc::XmlRpcValue d = kv.second;
    if (d.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      if (d.hasMember("type_class_package"))
        description.type_class_package.data = static_cast<std::string>(d["type_class_package"]);
      if (d.hasMember("type_class"))
        description.type_class.data = static_cast<std::string>(d["type_class"]);
      if (d.hasMember("base_class_package"))
        description.base_class_package.data = static_cast<std::string>(d["base_class_package"]);
      if (d.hasMember("base_class"))
        description.base_class.data = static_cast<std::string>(d["base_class"]);
      description.private_param_ns.data = prefix + std::string("/") + description.name.data;

      std::string import_name;
      if (d.hasMember("import"))
      {
        import_name = static_cast<std::string>(d["import"]);
        // auto complete remaining empty description fields by global definition of 'imported_name' plugin if exists
        autocompletePluginDescriptionByName(import_name, description);
      }
    }
    // lookup remaining empty description fields by global definition if exists
    autocompletePluginDescriptionByName(description.name.data, description);

    plugin_descriptions.push_back(description);
  }

  if (loadPluginSet(plugin_descriptions))
  {
    Instance()->loaded_plugin_set_ = name;
    ROS_INFO("[PluginManager] loadPluginSet: Loaded plugin set '%s' successfully.", name.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("[PluginManager] loadPluginSet: Loaded plugin set '%s' failed!", name.c_str());
    return false;
  }
}

bool PluginManager::hasPlugin(Plugin::Ptr& plugin)
{
  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);
  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.find(plugin->getName());
  return (itr != Instance()->plugins_by_name_.end() && itr->second->getTypeClass() == plugin->getTypeClass());
}

bool PluginManager::hasPluginByName(const std::string& name)
{
  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);
  return Instance()->plugins_by_name_.find(name) != Instance()->plugins_by_name_.end();
}

bool PluginManager::hasPluginsByTypeClass(const std::string& type_class)
{
  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
  {
    if (itr->second->getTypeClass() == type_class)
      return true;
  }
}

void PluginManager::loadParams(const vigir_generic_params::ParameterSet& params)
{
  boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
    itr->second->loadParams(params);
}

bool PluginManager::getPluginDescription(const std::string& key, msgs::PluginDescription& description)
{
  description = msgs::PluginDescription();

  if (!Instance()->nh_.hasParam(key))
  {
    ROS_ERROR("[PluginManager] getPluginDescription: Couldn't retrieve plugin description at '%s' from parameter server.", key.c_str());
    return false;
  }

  Instance()->nh_.getParam(key + "/name", description.name.data);
  Instance()->nh_.getParam(key + "/type_class", description.type_class.data);
  Instance()->nh_.getParam(key + "/type_class_package", description.type_class_package.data);
  Instance()->nh_.getParam(key + "/base_class", description.base_class.data);
  Instance()->nh_.getParam(key + "/base_class_package", description.base_class_package.data);

  return !description.name.data.empty() || !description.type_class.data.empty();
}

void PluginManager::publishPluginStateUpdate()
{
  msgs::PluginStates plugin_states;
  getPluginStates(plugin_states.states);
  plugin_states_pub_.publish(plugin_states);
}

// --- Subscriber calls ---

void PluginManager::addPlugin(const msgs::PluginDescriptionConstPtr plugin_description)
{
  addPlugin(*plugin_description);
}

void PluginManager::loadPluginSet(const std_msgs::StringConstPtr name)
{
  PluginManager::loadPluginSet(name->data);
}

void PluginManager::removePlugin(const msgs::PluginDescriptionConstPtr plugin_description)
{
  removePlugin(*plugin_description);
}

// --- Service calls ---

bool PluginManager::getPluginDescriptionsService(msgs::GetPluginDescriptionsService::Request& req, msgs::GetPluginDescriptionsService::Response& resp)
{
  getPluginDescriptions(resp.descriptions, req.filter);
  return true;
}

bool PluginManager::getPluginStatesService(msgs::GetPluginStatesService::Request& req, msgs::GetPluginStatesService::Response& resp)
{
  getPluginStates(resp.states, req.filter);
  return true;
}

bool PluginManager::addPluginService(msgs::PluginManagementService::Request& req, msgs::PluginManagementService::Response& /*resp*/)
{
  return addPlugins(req.descriptions);
}

bool PluginManager::removePluginService(msgs::PluginManagementService::Request& req, msgs::PluginManagementService::Response& /*resp*/)
{
  return removePlugins(req.descriptions);
}

bool PluginManager::loadPluginSetService(msgs::PluginManagementService::Request& req, msgs::PluginManagementService::Response& /*resp*/)
{
  if (!req.name.data.empty())
    return loadPluginSet(req.name.data);
  else
    return loadPluginSet(req.descriptions);
}

// --- Action Server calls ---

void PluginManager::getPluginDescriptionsAction(const msgs::GetPluginDescriptionsGoalConstPtr goal)
{
  // check if new goal was preempted in the meantime
  if (get_plugin_descriptions_as_->isPreemptRequested())
  {
    get_plugin_descriptions_as_->setPreempted();
    return;
  }

  msgs::GetPluginDescriptionsResult result;
  getPluginDescriptions(result.descriptions, goal->filter);

  get_plugin_descriptions_as_->setSucceeded(result);
}

void PluginManager::getPluginStatesAction(const msgs::GetPluginStatesGoalConstPtr goal)
{
  // check if new goal was preempted in the meantime
  if (get_plugin_states_as_->isPreemptRequested())
  {
    get_plugin_states_as_->setPreempted();
    return;
  }

  msgs::GetPluginStatesResult result;
  getPluginStates(result.states, goal->filter);

  get_plugin_states_as_->setSucceeded(result);
}

void PluginManager::addPluginAction(const msgs::PluginManagementGoalConstPtr goal)
{
  // check if new goal was preempted in the meantime
  if (add_plugin_as_->isPreemptRequested())
  {
    add_plugin_as_->setPreempted();
    return;
  }

  msgs::PluginManagementResult result;
  result.success.data = addPlugins(goal->descriptions);

  if (result.success.data)
    add_plugin_as_->setSucceeded(result);
  else
    add_plugin_as_->setAborted(result);
}

void PluginManager::removePluginAction(const msgs::PluginManagementGoalConstPtr goal)
{
  // check if new goal was preempted in the meantime
  if (remove_plugin_as_->isPreemptRequested())
  {
    remove_plugin_as_->setPreempted();
    return;
  }

  msgs::PluginManagementResult result;
  result.success.data = removePlugins(goal->descriptions);

  if (result.success.data)
    remove_plugin_as_->setSucceeded(result);
  else
    remove_plugin_as_->setAborted(result);
}

void PluginManager::loadPluginSetAction(const msgs::PluginManagementGoalConstPtr goal)
{
  // check if new goal was preempted in the meantime
  if (load_plugin_set_as_->isPreemptRequested())
  {
    load_plugin_set_as_->setPreempted();
    return;
  }

  msgs::PluginManagementResult result;
  if (!goal->name.data.empty())
    result.success.data = loadPluginSet(goal->name.data);
  else
    result.success.data = loadPluginSet(goal->descriptions);

  if (result.success.data)
    load_plugin_set_as_->setSucceeded(result);
  else
    load_plugin_set_as_->setAborted(result);
}
} // namespace
