//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_PLUGINLIB_PLUGIN_H__
#define VIGIR_PLUGINLIB_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_generic_params/parameter_manager.h>

#include <vigir_pluginlib_msgs/pluginlib_msgs.h>



namespace vigir_pluginlib
{
class Plugin
{
public:
  // typedefs
  typedef boost::shared_ptr<Plugin> Ptr;
  typedef boost::shared_ptr<const Plugin> ConstPtr;
  typedef boost::weak_ptr<Plugin> WeakPtr;
  typedef boost::weak_ptr<const Plugin> ConstWeakPtr;

  Plugin(const std::string& name = std::string(), const std::string& type_class_package = std::string(), const std::string& base_class_package = std::string(), const std::string& base_class = std::string());
  virtual ~Plugin();

private:
  /**
   * @brief Internal initialization of plugin itself.
   * @param nh Nodehandle the plugin
   * @param params active parameter set taken from ParameterManager
   * @return true, if setup was successful
   */
  bool setup(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet());

public:
  /**
   * @brief Loads parameters from parameter set and rosparam server.
   * This method will be automatically called before Plugin::initialize(...). And can be called each time parameters are
   * changed. Use this method for dynamic parameters.
   * @param params active global parameter set (!= plugin's own parameter set) taken from ParameterManager
   * @return true, if loading parameters was successful
   */
  virtual bool loadParams(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) { return true; }

  /**
   * @brief Initialization of plugin specific features. This method is intented to be called only
   * during initialization of the plugin. It is not supposed to be called again during life cycle.
   * @param params active global parameter set (!= plugin's own parameter set) taken from ParameterManager
   * @return true when initialization was successful
   */
  virtual bool initialize(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) { return true; }

  /**
   * @brief Called after initialization of this and other plugins has been completed.
   * At this point other plugins can be used safely.
   * @param params active global parameter set (!= plugin's own parameter set) taken from ParameterManager
   * @return true, if post initialization was successful
   */
  virtual bool postInitialize(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) { return true; }

 /**
  * @brief Returns class type string of plugin. If custom class type string should be used, then partial specialization
  * of TypeClass::get(...) is required (see type_class_traits.h).
  * @return determined type class as string
  */
  const std::string& getTypeClass() const;

  const msgs::PluginDescription& getDescription() const;
  const std::string& getName() const;
  const std::string& getTypeClassPackage() const;
  const std::string& getBaseClassPackage() const;
  const std::string& getBaseClass() const;

  /**
   * Unique plugins (default: true) can only exist once per type_class.
   * The plugin manager will replace any plugin of same type_class with the
   * new one. If a plugin can live in coexistence with others plugins
   * with the the same type_class, override this method to return false.
   **/
  virtual bool isUnique() const { return true; }

  /**
   * @brief Updates parameter from plugin's dedicated parameter set
   * @param key key of parameter
   * @param val new value for parametr
   * @param default_val default value
   */
  template<typename T>
  void updateParam(const std::string& key, const T& val)
  {
    params_.setParam(key, val);
  }

  friend class PluginManager;

protected:
  /**
   * @brief Updates plugin description
   * @param description new description
   */
  void updateDescription(const msgs::PluginDescription& description);

  /**
   * @brief Returns parameter set of plugin
   * @return parameter set of plugin
   */
  inline const vigir_generic_params::ParameterSet& getParams() const { return params_; }

  /**
   * @brief Determines if param with given key exists in set.
   * @param key parameter name
   * @return true if parameter with given key exists
   */
  inline bool hasParam(const std::string& key) const
  {
    return params_.hasParam(key);
  }

  /**
   * @brief Retrieves parameter from plugin's dedicated parameter set
   * @param key key of parameter
   * @param p [out] return variable for parameter
   * @param default_val default value
   * @param ignore_warnings (default = false) When set to true, no warnings will be printed if param is not available
   * @return true when parameter was found
   */
  template<typename T>
  inline bool getParam(const std::string& key, T& p, const T& default_val, bool ignore_warnings = false) const
  {
    return params_.getParam(key, p, default_val, ignore_warnings);
  }

  /**
   * @brief Retrieves parameter from plugin's dedicated parameter set
   * @param key key of parameter
   * @param p [out] return variable for parameter
   * @param default_val default value
   * @param ignore_warnings (default = false) When set to true, no warnings will be printed if param is not available
   * @return true when parameter was found
   */
  template<typename T>
  inline bool param(const std::string& key, T& p, const T& default_val, bool ignore_warnings = false) const
  {
    return params_.param(key, p, default_val, ignore_warnings);
  }

  /**
   * @brief Retrieves parameter from plugin's dedicated parameter set
   * @param key key of parameter
   * @param default_val default value
   * @param ignore_warnings (default = false) When set to true, no warnings will be printed if param is not available
   * @return retrieved parameter if available, otherwise given default value
   */
  template<typename T>
  inline T param(const std::string& key, const T& default_val, bool ignore_warnings = false) const
  {
    return params_.param(key, default_val, ignore_warnings);
  }

  ros::NodeHandle nh_;

private:
  void setNodehandle(const ros::NodeHandle& nh);
  void setParams(const vigir_generic_params::ParameterSet& params);

  vigir_generic_params::ParameterSet params_;

  msgs::PluginDescription description_;
};

/**
 * Upcast method in order to change current type to another within the inheritance hierachy.
 * @param T target class type
 * @param in input shared pointer that should be casted
 * @return shared pointer to upcast object, nullptr when invalid cast
 */
template<typename T1, typename T2>
inline static boost::shared_ptr<T1> cast(boost::shared_ptr<T2> in) { return boost::dynamic_pointer_cast<T1>(in); }
}

#endif
