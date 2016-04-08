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

#ifndef VIGIR_PLUGINLIB_MSGS_H__
#define VIGIR_PLUGINLIB_MSGS_H__

#include <ros/ros.h>

// messages
#include <vigir_pluginlib_msgs/PluginDescription.h>
#include <vigir_pluginlib_msgs/PluginState.h>
#include <vigir_pluginlib_msgs/PluginStates.h>

// services
#include <vigir_pluginlib_msgs/GetPluginDescriptionsService.h>
#include <vigir_pluginlib_msgs/GetPluginStatesService.h>
#include <vigir_pluginlib_msgs/PluginManagementService.h>

// actions
#include <vigir_pluginlib_msgs/GetPluginDescriptionsAction.h>
#include <vigir_pluginlib_msgs/GetPluginStatesAction.h>
#include <vigir_pluginlib_msgs/PluginManagementAction.h>



namespace vigir_pluginlib
{
namespace msgs
{
using namespace vigir_pluginlib_msgs;
}

bool isDescriptionMatching(const msgs::PluginDescription& plugin_description, const msgs::PluginDescription& filter);

/**
 * @brief Filters a plugin description list using a given filter criteria.
 * @param plugin_descriptions original list
 * @param filter description to match
 * @param inverse if true, the inverse result will be returned
 * @return resulting filtered list
 */
std::vector<msgs::PluginDescription> filterDescriptionList(const std::vector<msgs::PluginDescription>& plugin_descriptions, const msgs::PluginDescription& filter, bool inverse = false);

/**
 * @brief Filters a plugin description list using a given filter criteria.
 * @param plugin_descriptions original list
 * @param filter_list descriptions to match; Matching at least on element of this list is sufficient for positiv match
 * @param inverse if true, the inverse result will be returned
 * @return resulting filtered list
 */
std::vector<msgs::PluginDescription> filterDescriptionList(const std::vector<msgs::PluginDescription>& plugin_descriptions, const std::vector<msgs::PluginDescription>& filter_list, bool inverse = false);
}

#endif
