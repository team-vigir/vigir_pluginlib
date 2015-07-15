#!/usr/bin/env python

import os
import QtCore

import rospy
import rospkg
import actionlib

import std_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject, QAbstractTableModel
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout

from vigir_pluginlib.msg import GetPluginDescriptionsAction, GetPluginDescriptionsGoal, GetPluginDescriptionsResult, PluginManagementAction, PluginManagementGoal, PluginManagementResult

class PluginManagerDialog(Plugin):

    def __init__(self, context):
        super(PluginManagerDialog, self).__init__(context)
        self.setObjectName('PluginManagerDialog')

        self._parent = QWidget()
        self._widget = PluginManagerWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

class PluginManagerWidget(QObject):

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
        self.plugin_table_model = PluginTableModel()
        #self.plugin_manager_widget.plugin_table_view.setModel(self.plugin_table_model)

        # connect to signals
        #self.plugin_manager_widget.send_torque_mode.clicked[bool].connect(self._handle_send_torque_mode_clicked)

        # Qt signals
        #self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

        # init subscribers/action clients
        self.get_plugin_descriptions_client = actionlib.SimpleActionClient("/vigir/footstep_planning/plugin_manager/get_plugin_descriptions", GetPluginDescriptionsAction)

        # start timer
        self.update_timer = rospy.Timer(rospy.Duration(3), self._update_plugin_descriptions)

    def shutdown_plugin(self):
        print "Shutting down ..."
        self.update_timer.shutdown()        
        print "Done!"

    def _update_plugin_descriptions(self, event):
        if (self.get_plugin_descriptions_client.wait_for_server(rospy.Duration(0.5))):
            self.get_plugin_descriptions_client.send_goal(GetPluginDescriptionsGoal())
            if (self.get_plugin_descriptions_client.wait_for_result(rospy.Duration(5.0))):
                self.plugin_table_model.update(self.get_plugin_descriptions_client.get_result().plugin_descriptions)
                self.plugin_manager_widget.plugin_table_view.setModel(self.plugin_table_model)

    def _control_mode_callback(self, control_mode):
        self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), str(control_mode.data))

class PluginTableModel(QtCore.QAbstractTableModel):

    def __init__(self, parent=None, *args):
        super(PluginTableModel, self).__init__()
        self.plugin_descriptions = []

    def update(self, plugin_descriptions):
        self.plugin_descriptions = plugin_descriptions

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.plugin_descriptions)

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 3

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return
        elif role == QtCore.Qt.DisplayRole:
            if index.column() == 0:
                return self.plugin_descriptions[index.row()].type_class.data
            if index.column() == 1:
                return self.plugin_descriptions[index.row()].base_class.data
            if index.column() == 2:
                return self.plugin_descriptions[index.row()].type_class_package.data

    def flags(self, index):
        return QtCore.Qt.ItemIsEnabled

