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

#ifndef VIGIR_PLUGINLIB_PLUGIN_H__
#define VIGIR_PLUGINLIB_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_generic_params/parameter_manager.h>

#include <vigir_pluginlib_msgs/pluginlib_msgs.h>



namespace vigir_pluginlib
{
std::string demangle(const char* name);

class Plugin
{
public:
  // typedefs
  typedef boost::shared_ptr<Plugin> Ptr;
  typedef boost::shared_ptr<const Plugin> ConstPtr;
  typedef boost::weak_ptr<Plugin> WeakPtr;
  typedef boost::weak_ptr<const Plugin> ConstWeakPtr;

  Plugin(const std::string& name, const std::string& type_class_package = std::string(), const std::string& base_class_package = std::string(), const std::string& base_class = std::string());
  virtual ~Plugin();

private:
  /**
   * @brief Internal initialization of plugin itself, e.g. setting parameter namespaces.
   * @param nh Nodehandle the plugin
   * @param global_params global parameter set
   * @return true, if setup was successful
   */
  bool setup(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet());

public:
  /**
   * @brief Initialization of plugin specific features.
   * @param global_params global parameter set
   * @return true when initialization was successful
   */
  virtual bool initialize(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) { return loadParams(global_params); }

  /**
   * @brief Called after initialization of this and other plugins has been completed.
   * At this point other plugins can be used safely.
   * @param global_params global parameter set
   * @return true, if post initialization was successful
   */
  virtual bool postInitialize(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) { return true; }

  /**
   * @brief Loads parameters from parameter set and rosparam server (!= plugin's dedicated parameters).
   * This method will be automatically called by Plugin::initialize(...).
   * @param global_params global parameter set
   * @return true, if loading parameters was successful
   */
  virtual bool loadParams(const vigir_generic_params::ParameterSet& /*global_params*/) { return true; }

  /**
   * Used for automatically generate type ids for data types. Override _typeClass()
   * function to use custom type ids for derived data types!
   * Usage: vigir_pluginlib::Plugin::getTypeClass<MyClass>()
   * DO NOT OVERRIDE THIS METHOD! This wrapper prevents wrong usage, e.g. A::_typeClass<B>().
   */
  template <typename T>
  inline static std::string getTypeClass() { return T::template _typeClass<T>(); }
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

  friend class PluginManager;

protected:
  /**
   * Used internally for automatically generate type ids for data types.
   */
  template <typename T>
  inline static std::string _typeClass(T* t = nullptr) { return demangle(t ? typeid(*t).name() : typeid(T).name()); }

  /**
   * @brief Updates plugin description
   * @param description new description
   */
  void updateDescription(const msgs::PluginDescription& description);

  /**
   * @brief Returns parameter set of plugin
   * @return parameter set of pluign
   */
  inline const vigir_generic_params::ParameterSet& getParams() const { return params_; }

  /**
   * @brief Retrieves parameter from plugin's dedicated parameter set
   * @param name name of parameter
   * @param val [out] return variable for parameter
   * @param def default value
   * @param ignore_warnings if true no warnings will be printed out when param was not present
   * @return true when parameter was found at rosparam
   */
  template<typename T>
  bool getParam(const std::string& name, T& val, const T& def = T(), bool ignore_warnings = false) const
  {
    if (ignore_warnings && !params_.hasParam(name))
      return false;

    return params_.getParam(name, val, def);
  }

  /**
   * @brief Retrieves parameter from plugin's dedicated parameter set
   * @param name name of parameter
   * @param def default value
   * @param ignore_warnings When true no warnings will be printed out when param was not present
   * @return retrieved parameter if available, otherwise given default value
   */
  template<typename T>
  T param(const std::string& name, const T& def = T(), bool ignore_warnings = false) const
  {
    T result;
    getParam(name, result, def, ignore_warnings);
    return result;
  }

  ros::NodeHandle nh_;

private:
  void setNodehandle(const ros::NodeHandle& nh);
  void setParams(const vigir_generic_params::ParameterSet& params);

  vigir_generic_params::ParameterSet params_;

  msgs::PluginDescription description_;
};
}

#endif
