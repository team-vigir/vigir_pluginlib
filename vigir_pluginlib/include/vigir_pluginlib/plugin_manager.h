//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_PLUGINLIB_PLUGIN_MANAGER_H__
#define VIGIR_PLUGINLIB_PLUGIN_MANAGER_H__

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <boost/noncopyable.hpp>

#include <vigir_generic_params/parameter_manager.h>

#include <vigir_pluginlib/plugin_loader.h>
#include <vigir_pluginlib/plugin.h>

#include <vigir_pluginlib_msgs/pluginlib_msgs.h>



namespace vigir_pluginlib
{
class PluginManager
  : private boost::noncopyable
{
public:
  // typedefs
  typedef boost::shared_ptr<PluginManager> Ptr;
  typedef boost::shared_ptr<const PluginManager> ConstPtr;

  typedef std::vector<PluginLoaderBase*> PluginLoaderVector;

protected:
  typedef actionlib::SimpleActionServer<msgs::GetPluginDescriptionsAction>  GetPluginDescriptionsActionServer;
  typedef actionlib::SimpleActionServer<msgs::GetPluginStatesAction>        GetPluginStatesActionServer;
  typedef actionlib::SimpleActionServer<msgs::PluginManagementAction>       PluginManagementActionServer;

  static PluginManager::Ptr singelton_;

  PluginManager();

  static PluginManager::Ptr Instance();

public:
  ~PluginManager();

  /**
   * @brief Initialize all rostopic/-services within the namespace of the given nodehandle
   * @param nh The nodehandle which is used to setup all topics/services
   */
  static void initialize(ros::NodeHandle& nh);

  /**
   * @brief Tries to extract plugin description from rosparam server using the given name
   * @param name Name of the plugin
   * @param plugin_description Output of extracted plugin description
   * @param ns Namespace to lookup description from rosparam server
   * @return True if extraction was successful
   */
  static bool autocompletePluginDescriptionByName(const std::string& name, msgs::PluginDescription& plugin_description, const std::string& ns = std::string());

  /**
   * @brief Adds ClassLoader for a specific type of plugins
   * @param package The package containing the base class
   * @param base_class The type of the base class for classes to be loaded
   * @param attrib_name The attribute to search for in manifext.xml files, defaults to "plugin"
   * @param plugin_xml_paths The list of paths of plugin.xml files, defaults to be crawled via ros::package::getPlugins()
   * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
   */
  template<class PluginBaseClass>
  static bool addPluginClassLoader(const std::string& package, const std::string& base_class, const std::string& attrib_name = std::string("plugin"), const std::vector<std::string>& plugin_xml_paths = std::vector<std::string>())
  {
    boost::upgrade_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

    // check for duplicate
    for (PluginLoaderBase* loader : Instance()->class_loader_)
    {
      if (loader->getBaseClassType() == base_class)
      {
        ROS_DEBUG("[PluginManager] The ClassLoader for plugins of type '%s' has been already added!", base_class.c_str());
        return false;
      }
    }

    try
    {
      boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
      PluginLoader<PluginBaseClass>* loader = new PluginLoader<PluginBaseClass>(package, base_class, attrib_name, plugin_xml_paths);
      Instance()->class_loader_.push_back(loader);
      ROS_INFO("[PluginManager] Added ClassLoader for plugins of type '%s'.", base_class.c_str());
      ROS_DEBUG("  Declared classes:");
      for (const std::string s : loader->getDeclaredClasses())
        ROS_DEBUG("    %s", s.c_str());
      ROS_DEBUG("  Registered libraries:");
      for (const std::string s : loader->getRegisteredLibraries())
        ROS_DEBUG("    %s", s.c_str());
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      ROS_ERROR("[PluginManager] The ClassLoader for plugin '%s' of package '%s' failed to load for some reason. Error:\n%s", base_class.c_str(), package.c_str(), e.what());
      return false;
    }

    return true;
  }

  /**
   * @brief Instantiation of plugin using ClassLoader
   * @param plugin_description description which plugin to load
   * @param name name of plugin to load
   * @param initialize if true then the plugin's initialize will be called
   * after instantiation
   * @param auto_completion try to auto complete plugin description
   * @return false, if instantiation has failed, otherwise true
   */
  static bool addPlugins(const std::vector<msgs::PluginDescription>& plugin_descriptions, bool initialize = true, bool auto_completion = true);
  static bool addPlugin(const msgs::PluginDescription& plugin_description, bool initialize = true, bool auto_completion = true);
  static bool addPluginByName(const std::string& name, bool initialize = true);

  template<typename PluginDerivedClass>
  static void addPlugin(bool initialize = true)
  {
    addPlugin(new PluginDerivedClass());
  }
  static void addPlugin(Plugin* plugin, bool initialize = true); // this function takes over pointer and will free memory automatically, when plugin is removed
  static void addPlugin(Plugin::Ptr plugin, bool initialize = true);

  /**
   * Returns first found plugin matching typename T. If specific element should be returned, do set name.
   */
  template<typename T>
  static boost::shared_ptr<T> getPlugin(const std::string& name = std::string())
  {
    // name specific search
    if (!name.empty())
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(getPluginByName(name));
      if (plugin)
        return plugin;

      ROS_ERROR("[PluginManager] Couldn't find any matching plugin named '%s' of type '%s'!", name.c_str(), Plugin::getTypeClass<T>().c_str());
    }
    // type specific search
    else
    {
      boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

      for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
      {
        boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
        if (plugin)
          return plugin;
      }

      ROS_ERROR("[PluginManager] Couldn't find any matching plugin of type '%s'!", Plugin::getTypeClass<T>().c_str());
    }

    return boost::shared_ptr<T>();
  }

  /**
   * Returns first found plugin matching typename T. If specific element should be returned, do set name.
   */
  template<typename T>
  static bool getPlugin(boost::shared_ptr<T>& plugin, const std::string& name = std::string())
  {
    plugin = getPlugin<T>(name);
    return plugin != nullptr;
  }

  template<typename T>
  inline static bool getPluginByName(const std::string& name, boost::shared_ptr<T>& plugin)
  {
    return getPlugin(plugin, name);
  }
  static Plugin::Ptr getPluginByName(const std::string& name);
  static bool getPluginByName(const std::string& name, Plugin::Ptr& plugin);

  /// returns all plugins derived by class T in alphabetical order (name)
  template<typename T>
  static bool getPlugins(std::vector<boost::shared_ptr<T> >& plugins)
  {
    return getPluginsByType<T>(plugins);
  }

  template<typename T>
  static bool getPluginsByType(std::vector<boost::shared_ptr<T> >& plugins)
  {
    plugins.clear();

    boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

    for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
      if (plugin)
        plugins.push_back(plugin);
    }

    return !plugins.empty();
  }
  /// returns all plugins derived by class T as map
  template<typename T>
  static bool getPluginsByType(std::map<std::string, boost::shared_ptr<T> >& plugins)
  {
    plugins.clear();

    boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

    for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
      if (plugin)
        plugins[itr->first] = plugin;
    }

    return !plugins.empty();
  }
  static bool getPluginsByTypeClass(const std::string& type_class, std::vector<Plugin::Ptr>& plugins);

  /// returns a plugin marked as unique of specific type id
  static bool getUniquePluginByTypeClass(const std::string& type_class, Plugin::Ptr& plugin);

  static void getPluginDescriptions(std::vector<msgs::PluginDescription>& descriptions, msgs::PluginDescription filter = msgs::PluginDescription());
  static void getPluginStates(std::vector<msgs::PluginState>& plugin_states, msgs::PluginDescription filter = msgs::PluginDescription());

  static bool removePlugins(const std::vector<msgs::PluginDescription>& plugin_descriptions);
  static bool removePlugin(const msgs::PluginDescription& plugin_description);
  static bool removePluginByName(const std::string& name);

  static void removePlugin(Plugin::Ptr& plugin);

  template<typename T>
  static void removePluginsByType()
  {
    boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

    for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end();)
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
      if (plugin)
      {
        lock.unlock();
        removePluginByName(itr++->first);
        lock.lock();
      }
      else
        itr++;
    }
  }
  static void removePluginsByTypeClass(const std::string& type_class);

  static bool loadPluginSet(const std::vector<msgs::PluginDescription>& plugin_descriptions, bool auto_completion = true);
  static bool loadPluginSet(const std::string& name);

  static bool hasPlugin(Plugin::Ptr& plugin);

  static bool hasPluginByName(const std::string& name);

  template<typename T>
  static bool hasPluginsByType()
  {
    boost::shared_lock<boost::shared_mutex> lock(Instance()->plugins_mutex_);

    for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name_.begin(); itr != Instance()->plugins_by_name_.end(); itr++)
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
      if (plugin)
        return true;
    }

    return false;
  }
  static bool hasPluginsByTypeClass(const std::string& type_class);

  static void loadParams(const vigir_generic_params::ParameterSet& params);

protected:
  // helper
  bool getPluginDescription(const std::string& key, msgs::PluginDescription& description);
  void publishPluginStateUpdate();

  // mutex to ensure thread safeness
  mutable boost::shared_mutex plugins_mutex_;

  // nodehandle (namespace) to be used
  ros::NodeHandle nh_;

  // class loader
  PluginLoaderVector class_loader_;

  // instantiated plugins
  std::string loaded_plugin_set_;
  std::map<std::string, Plugin::Ptr> plugins_by_name_;

  /// ROS API

  // subscriber
  void addPlugin(const msgs::PluginDescriptionConstPtr plugin_description);
  void loadPluginSet(const std_msgs::StringConstPtr name);
  void removePlugin(const msgs::PluginDescriptionConstPtr plugin_description);

  // service calls
  bool getPluginDescriptionsService(msgs::GetPluginDescriptionsService::Request& req, msgs::GetPluginDescriptionsService::Response& resp);
  bool getPluginStatesService(msgs::GetPluginStatesService::Request& req, msgs::GetPluginStatesService::Response& resp);
  bool addPluginService(msgs::PluginManagementService::Request& req, msgs::PluginManagementService::Response& resp);
  bool removePluginService(msgs::PluginManagementService::Request& req, msgs::PluginManagementService::Response& resp);
  bool loadPluginSetService(msgs::PluginManagementService::Request& req, msgs::PluginManagementService::Response& resp);

  // action server calls
  void getPluginDescriptionsAction(const msgs::GetPluginDescriptionsGoalConstPtr goal);
  void getPluginStatesAction(const msgs::GetPluginStatesGoalConstPtr goal);
  void addPluginAction(const msgs::PluginManagementGoalConstPtr goal);
  void removePluginAction(const msgs::PluginManagementGoalConstPtr goal);
  void loadPluginSetAction(const msgs::PluginManagementGoalConstPtr goal);

  // subscriber
  ros::Subscriber add_plugin_sub_;
  ros::Subscriber load_plugin_set_sub_;
  ros::Subscriber remove_plugin_sub_;

  // publisher
  ros::Publisher plugin_states_pub_;

  // service servers
  ros::ServiceServer get_plugin_descriptions_srv_;
  ros::ServiceServer get_plugin_states_srv_;
  ros::ServiceServer add_plugin_srv_;
  ros::ServiceServer remove_plugin_srv_;
  ros::ServiceServer load_plugin_set_srv_;

  // action servers
  boost::shared_ptr<GetPluginDescriptionsActionServer> get_plugin_descriptions_as_;
  boost::shared_ptr<GetPluginStatesActionServer> get_plugin_states_as_;
  boost::shared_ptr<PluginManagementActionServer> add_plugin_as_;
  boost::shared_ptr<PluginManagementActionServer> remove_plugin_as_;
  boost::shared_ptr<PluginManagementActionServer> load_plugin_set_as_;
};
}

#endif
