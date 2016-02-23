//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
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
  Plugin(const msgs::PluginDescription& description);
  Plugin(const std::string& name, const std::string& type_class);
  virtual ~Plugin();

  /**
   * @brief Initialization of plugin specific features.
   * @return true when initialization was successful
   */
  virtual bool initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params);

  virtual void loadParams(const vigir_generic_params::ParameterSet& params);

  const msgs::PluginDescription& getDescription() const;
  const std::string& getName() const;
  const std::string& getTypeClassPackage() const;
  const std::string& getTypeClass() const;
  const std::string& getBaseClassPackage() const;
  const std::string& getBaseClass() const;

  /**
   * Unique plugins (default: true) can only exist once per type_class.
   * The plugin manager will replace any plugin of same type_class with the
   * new one. If a plugin can live in coexistence with others plugins
   * with the the same type_class, override this method to return false.
   **/
  virtual bool isUnique() const { return true; }

  // typedefs
  typedef boost::shared_ptr<Plugin> Ptr;
  typedef boost::shared_ptr<const Plugin> ConstPtr;

  friend class PluginManager;

protected:
  template<typename T>
  bool getPluginParam(const std::string& name, T& val, const T& def = T(), bool ignore_warnings = false) const
  {
    std::string name_ns = description.param_namespace.data + std::string("/") + name;
    if (root_nh.hasParam(name_ns) && root_nh.getParam(name_ns, val))
      return true;
    else
    {
      val = def;
      if (!ignore_warnings)
      {
        if (description.param_namespace.data.empty())
          ROS_WARN("[%s]: No plugin parameters defined!", this->getName().c_str());
        else
          ROS_WARN("[%s]: '%s' plugin parameter is missing in namespace '%s'!", this->getName().c_str(), name.c_str(), description.param_namespace.data.c_str());
      }
      return false;
    }
  }

  template<typename T>
  T pluginParam(const std::string& name, const T& def = T(), bool ignore_warnings = false) const
  {
    T result;
    getPluginParam(name, result, def, ignore_warnings);
    return result;
  }

  ros::NodeHandle root_nh;
  ros::NodeHandle plugin_nh;

private:
  msgs::PluginDescription description;
};
}

#endif
