#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, Slot, QObject, QAbstractItemModel
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout

from vigir_plugin_manager.plugin_tree_model import *
from vigir_pluginlib.msg import PluginState, GetPluginStatesAction, GetPluginStatesGoal, GetPluginStatesResult, PluginManagementAction, PluginManagementGoal, PluginManagementResult


# UI initialization
class PluginManagerDialog(Plugin):

    def __init__(self, context):
        super(PluginManagerDialog, self).__init__(context)
        self.setObjectName('PluginManagerDialog')

        self._parent = QWidget()
        self._widget = PluginManagerWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


# Plugin Manager Widget
class PluginManagerWidget(QObject):

    plugin_states_signal = Signal(list)

    def __init__(self, context):
        super(PluginManagerWidget, self).__init__()

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.plugin_manager_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('vigir_plugin_manager'), 'resource', 'plugin_manager.ui')
        loadUi(ui_file, self.plugin_manager_widget, {'QWidget': QWidget})
        vbox.addWidget(self.plugin_manager_widget)

        # init table view
        self.plugin_tree_model = PluginTreeModel()
        #self.plugin_manager_widget.plugin_tree_view.setModel(self.plugin_tree_model)

        # connect to signals
        #self.plugin_manager_widget.send_torque_mode.clicked[bool].connect(self._handle_send_torque_mode_clicked)

        # Qt signals
        self.plugin_states_signal.connect(self.update_plugin_tree_view)
        #self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

        # init subscribers/action clients
        self.get_plugin_states_client = actionlib.SimpleActionClient("/vigir/footstep_planning/plugin_manager/get_plugin_states", GetPluginStatesAction)

        # start timer
        self.update_timer = rospy.Timer(rospy.Duration(3), self._update_plugin_states)

    def shutdown_plugin(self):
        print "Shutting down ..."
        self.update_timer.shutdown()
        print "Done!"

    def _update_plugin_states(self, event):
        if (self.get_plugin_states_client.wait_for_server(rospy.Duration(0.5))):
            self.get_plugin_states_client.send_goal(GetPluginStatesGoal())
            if (self.get_plugin_states_client.wait_for_result(rospy.Duration(5.0))):
                self.plugin_states_signal.emit(self.get_plugin_states_client.get_result().states)

    @Slot(list)
    def update_plugin_tree_view(self, states):
        #self.plugin_tree_model = PluginTreeModel() # TODO: otherwise crash!
        self.plugin_tree_model.setData(states)
        self.plugin_manager_widget.plugin_tree_view.setModel(self.plugin_tree_model)

    def _control_mode_callback(self, control_mode):
        self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), str(control_mode.data))
