//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_PLUGINLIB_PLUGIN_LOADER_H__
#define VIGIR_PLUGINLIB_PLUGIN_LOADER_H__

#include <ros/ros.h>

#include <pluginlib/class_loader.h>

#include <vigir_pluginlib/plugin_loader_base.h>
#include <vigir_pluginlib/plugin.h>



namespace vigir_pluginlib
{
template <class T>
class PluginLoader
  : public pluginlib::ClassLoader<T>
  , public PluginLoaderBase
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
  PluginLoader(const std::string& package, const std::string& base_class, const std::string& attrib_name = std::string("plugin"), const std::vector<std::string>& plugin_xml_paths = std::vector<std::string>())
    : pluginlib::ClassLoader<T>(package, base_class, attrib_name, plugin_xml_paths)
    , PluginLoaderBase(package, base_class, attrib_name, plugin_xml_paths)
  {
  }

  /**
   * @brief  Returns a list of all available plugin manifest paths for this ClassLoader's base class type
   * @return A vector of strings corresponding to the paths of all available plugin manifests
   */
  std::vector<std::string> getPluginXmlPaths() override
  {
    return pluginlib::ClassLoader<T>::getPluginXmlPaths();
  }

  /**
   * @brief  Returns a list of all available classes for this ClassLoader's base class type
   * @return A vector of strings corresponding to the names of all available classes
   */
  std::vector<std::string> getDeclaredClasses() override
  {
    return pluginlib::ClassLoader<T>::getDeclaredClasses();
  }

  /**
   * @brief  Refreshs the list of all available classes for this ClassLoader's base class type
   * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
   */
  void refreshDeclaredClasses() override
  {
    return pluginlib::ClassLoader<T>::refreshDeclaredClasses();
  }

  /**
   * @brief  Strips the package name off of a lookup name
   * @param lookup_name The name of the plugin
   * @return The name of the plugin stripped of the package name
   */
  std::string getName(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::getName(lookup_name);
  }

  /**
   * @brief  Checks if the class associated with a plugin name is available to be loaded
   * @param lookup_name The name of the plugin
   * @return True if the plugin is available, false otherwise
   */
  bool isClassAvailable(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::isClassAvailable(lookup_name);
  }

  /**
   * @brief  Given the lookup name of a class, returns the type of the derived class associated with it
   * @param lookup_name The name of the class
   * @return The name of the associated derived class
   */
  std::string getClassType(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::getClassType(lookup_name);
  }

  /**
   * @brief  Given the lookup name of a class, returns its description
   * @param lookup_name The lookup name of the class
   * @return The description of the class
   */
  std::string getClassDescription(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::getClassDescription(lookup_name);
  }

  /**
   * @brief  Given the lookup name of a class, returns the type of the associated base class
   * @return The type of the associated base class
   */
  std::string getBaseClassType() const override
  {
    return pluginlib::ClassLoader<T>::getBaseClassType();
  }

  /**
   * @brief  Given the name of a class, returns name of the containing package
   * @param lookup_name The name of the class
   * @return The name of the containing package
   */
  std::string getClassPackage(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::getClassPackage(lookup_name);
  }

  /**
   * @brief  Given the name of a class, returns the path of the associated plugin manifest
   * @param lookup_name The name of the class
   * @return The path of the associated plugin manifest
   */
  std::string getPluginManifestPath(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::getPluginManifestPath(lookup_name);
  }

  /**
   * @brief Checks if a given class is currently loaded
   * @param  lookup_name The lookup name of the class to query
   * @return True if the class is loaded, false otherwise
   */
  bool isClassLoaded(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::isClassLoaded(lookup_name);
  }

  /**
   * @brief  Attempts to load a class with a given name
   * @param lookup_name The lookup name of the class to load
   * @exception pluginlib::LibraryLoadException Thrown if the library for the class cannot be loaded
   */
  void loadLibraryForClass(const std::string & lookup_name) override
  {
    return pluginlib::ClassLoader<T>::loadLibraryForClass(lookup_name);
  }

  /**
   * @brief  Attempts to unload a class with a given name
   * @param lookup_name The lookup name of the class to unload
   * @exception pluginlib::LibraryUnloadException Thrown if the library for the class cannot be unloaded
   * @return The number of pending unloads until the library is removed from memory
   */
  int unloadLibraryForClass(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::unloadLibraryForClass(lookup_name);
  }

  /**
   * @brief  Returns the libraries that are registered and can be loaded
   * @return A vector of strings corresponding to the names of registered libraries
   */
  std::vector<std::string> getRegisteredLibraries() override
  {
    return pluginlib::ClassLoader<T>::getRegisteredLibraries();
  }

  /**
   * @brief  Given the name of a class, returns the path to its associated library
   * @param lookup_name The name of the class
   * @return The path to the associated library
   */
  std::string getClassLibraryPath(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::getClassLibraryPath(lookup_name);
  }

  /**
   * @brief  Creates an instance of a plugin which is derived from vigir_pluginlib::Plugin base class (which implicitly calls loadLibraryForClass() to increment the library counter). Deleting the instance and calling unloadLibraryForClass() is automatically handled by the shared pointer.
   * @param  lookup_name The name of the class to load
   * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
   * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
   * @return An instance of the class
   */
  Plugin::Ptr createPluginInstance(const std::string& lookup_name) override
  {
    return pluginlib::ClassLoader<T>::createInstance(lookup_name);
  }
};
}

#endif
