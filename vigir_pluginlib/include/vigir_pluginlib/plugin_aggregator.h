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

#ifndef VIGIR_PLUGINLIB_PLUGIN_AGGREGATOR_H__
#define VIGIR_PLUGINLIB_PLUGIN_AGGREGATOR_H__

#include <functional>

#include <ros/ros.h>

#include <vigir_generic_params/parameter_manager.h>

#include <vigir_pluginlib/plugin_manager.h>



namespace vigir_pluginlib
{
template<class PluginClass>
class PluginAggregator
{
public:
  // typedefs
  typedef boost::shared_ptr<PluginAggregator<PluginClass>> Ptr;
  typedef boost::shared_ptr<const PluginAggregator<PluginClass>> ConstPtr;
  typedef boost::weak_ptr<PluginAggregator<PluginClass>> WeakPtr;
  typedef boost::weak_ptr<const PluginAggregator<PluginClass>> ConstWeakPtr;

  PluginAggregator(const std::string& name = std::string("PluginAggregator"))
    : name_(name) {}

  /**
   * @brief Clears internal collection of all plugins
   */
  virtual void clear()
  {
    plugins_.clear();
  }

  /**
   * @brief Collects all plugins from PluginManager matching the type T.
   */
  virtual void loadPlugins()
  {
    // get plugins
    PluginManager::getPlugins(plugins_);

    if (plugins_.empty())
      ROS_WARN("[%s] loadPlugins: Couldn't find any plugin of type '%s'.", name_.c_str(), vigir_pluginlib::TypeClass::get<PluginClass>().c_str());
    else
    {
      ROS_INFO("[%s] Plugins loaded:", name_.c_str());
      for (const boost::shared_ptr<PluginClass> plugin : plugins_)
      {
        if (plugin)
          ROS_INFO("    %s", plugin->getName().c_str());
      }
    }
  }

  /**
   * @brief Returns a list of all known plugins of type T.
   * @return List of all known plugins of type T
   */
  std::vector<boost::shared_ptr<PluginClass>> getPlugins() const
  {
    return plugins_;
  }

  /**
   * @brief Returns the number of known plugins of type T.
   * @return Number of known plugins of type T
   */
  size_t size() const
  {
    return plugins_.size();
  }

  /**
   * @brief Loads parameters on all known plugins.
   * @param params Parameter set to load
   * @return false when error occurs, otherwise true
   */
  virtual bool loadParams(const vigir_generic_params::ParameterSet& params)
  {
    bool result = true;

    for (const boost::shared_ptr<PluginClass> plugin : plugins_)
    {
      if (plugin)
        result &= plugin->loadParams(params);
    }

    return result;
  }

  void call(std::function<void()> fun)
  {
    for (const boost::shared_ptr<PluginClass> plugin : plugins_)
    {
      if (plugin)
        fun(plugin.get());
    }
  }

protected:
  std::string name_;
  std::vector<boost::shared_ptr<PluginClass>> plugins_;
};
}

#endif
