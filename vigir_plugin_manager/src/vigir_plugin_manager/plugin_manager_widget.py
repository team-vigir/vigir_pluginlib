#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, Slot, QObject, QAbstractItemModel
from python_qt_binding.QtGui import QAbstractItemView, QWidget, QMenu, QAction, QHBoxLayout, QVBoxLayout

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

        # init tree view
        tree_view = self.plugin_manager_widget.plugin_tree_view
        tree_view.setSelectionMode(QAbstractItemView.ExtendedSelection)

        tree_view.setContextMenuPolicy(Qt.CustomContextMenu)
        tree_view.customContextMenuRequested.connect(self.open_context_menu)

        self.plugin_tree_model = PluginTreeModel()
        tree_view.setModel(self.plugin_tree_model)

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

    def open_context_menu(self, position):
        indexes = self.plugin_manager_widget.plugin_tree_view.selectedIndexes()
        level = -1
        if len(indexes) > 0:
            level = 0
            index = indexes[0]
            while index.parent().isValid():
                index = index.parent()
                level += 1

        menu = QMenu()
        if level == 0:
            expand_action = QAction(self.tr("Expand"), None)
            expand_action.triggered.connect(self.plugin_manager_widget.plugin_tree_view.expandAll)
            menu.addAction(expand_action)
        if level == 0 or level == 1:
            remove_action = QAction(self.tr("Remove"), None)
            remove_action.triggered.connect(self._remove_plugin)
            menu.addAction(remove_action)

        menu.exec_(self.plugin_manager_widget.plugin_tree_view.viewport().mapToGlobal(position))

    def _update_plugin_states(self, event):
        if (self.get_plugin_states_client.wait_for_server(rospy.Duration(0.5))):
            self.get_plugin_states_client.send_goal(GetPluginStatesGoal())
            if (self.get_plugin_states_client.wait_for_result(rospy.Duration(5.0))):
                self.plugin_states_signal.emit(self.get_plugin_states_client.get_result().states)

    @Slot(list)
    def update_plugin_tree_view(self, states):
        #self.plugin_tree_model = PluginTreeModel() # TODO: otherwise crash!
        self.plugin_tree_model.updateData(states)
        self.plugin_manager_widget.plugin_tree_view.setModel(self.plugin_tree_model)
        #for column in range(0, self.plugin_tree_model.columnCount()):
        #    self.plugin_tree_model.resizeColumnToContents(column)
        #self.plugin_manager_widget.plugin_tree_view.expandAll()

    @Slot()
    def _remove_plugin(self):
        model = self.plugin_manager_widget.plugin_tree_view.model()

        indexes = self.plugin_manager_widget.plugin_tree_view.selectionModel().selectedIndexes()
        indexes = filter(lambda index: index.column() == 0, indexes)

        rows = []
        for index in indexes:
            rows.append((index.row(), index.parent()))

        rows.sort(reverse=True)

        for row in rows:
            model.removeRow(row[0], row[1])
            # TODO: action server call

    def _control_mode_callback(self, control_mode):
        self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), str(control_mode.data))
