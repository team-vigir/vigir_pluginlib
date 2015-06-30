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



namespace vigir_pluginlib
{
using namespace vigir_generic_params;

class Plugin
{
public:
  Plugin(const std::string& name, const std::string& type_id, const ParameterSet& params);
  Plugin(const std::string& name, const std::string& type_id);
  virtual ~Plugin();

  virtual void loadParams(const ParameterSet& params);

  virtual bool initialize(ros::NodeHandle& nh, const ParameterSet& params);

  const std::string& getName() const;
  const std::string& getTypeId() const;

  /**
   * Unique plugins (default: true) can only exist once per type_id.
   * The plugin manager will replace any plugin of same type_id with the
   * new one. If a plugin can live in coexistence with others plugins
   * with the the same type_id, override this method to return false.
   **/
  virtual bool isUnique() const;

  // typedefs
  typedef boost::shared_ptr<Plugin> Ptr;
  typedef boost::shared_ptr<const Plugin> ConstPtr;

private:
  const std::string type_id;
  const std::string name;
};
}

#endif
