//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
// Based on pluginlib (http://wiki.ros.org/pluginlib)
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

#ifndef VIGIR_PLUGINLIB_PLUGIN_LOADER_BASE_H__
#define VIGIR_PLUGINLIB_PLUGIN_LOADER_BASE_H__

#include <ros/ros.h>

#include <pluginlib/class_loader_base.h>

#include <vigir_pluginlib/plugin.h>



namespace vigir_pluginlib
{
class PluginLoaderBase
  : public virtual pluginlib::ClassLoaderBase
{
public:
  /**
   * @brief  Constructor for a ClassLoader
   * @param package The package containing the base class
   * @param base_class The type of the base class for classes to be loaded
   * @param attrib_name The attribute to search for in manifext.xml files, defaults to "plugin"
   * @param plugin_xml_paths The list of paths of plugin.xml files, defaults to be crawled via ros::package::getPlugins()
   * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
   */
  PluginLoaderBase(std::string package, std::string base_class, std::string attrib_name = std::string("plugin"), std::vector<std::string> plugin_xml_paths = std::vector<std::string>());

  /**
   * @brief  Creates an instance of a plugin which is derived from vigir_pluginlib::Plugin base class (which implicitly calls loadLibraryForClass() to increment the library counter). Deleting the instance and calling unloadLibraryForClass() is automatically handled by the shared pointer.
   * @param  lookup_name The name of the class to load
   * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
   * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
   * @return An instance of the class
   */
  virtual Plugin::Ptr createPluginInstance(const std::string& lookup_name) = 0;

  std::string getBaseClassPackage() const;

protected:
  std::string base_class_package_;
};
}

#endif
