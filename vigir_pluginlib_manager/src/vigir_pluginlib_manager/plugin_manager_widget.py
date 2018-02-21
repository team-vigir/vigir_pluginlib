#!/usr/bin/env python

import os

import rospy
import rospkg
import rosparam
import actionlib

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, Slot, QSignalMapper, QObject, QAbstractItemModel
from python_qt_binding.QtWidgets import QAbstractItemView, QWidget, QMenu, QAction, QComboBox

from vigir_pluginlib_manager.plugin_tree_model import *
from vigir_pluginlib_msgs.msg import PluginStates, GetPluginDescriptionsAction, GetPluginDescriptionsGoal, GetPluginStatesAction, GetPluginStatesGoal, GetPluginStatesResult, PluginManagementAction, PluginManagementGoal, PluginManagementResult


# UI initialization
class PluginManagerDialog(Plugin):

    def __init__(self, context):
        super(PluginManagerDialog, self).__init__(context)
        self_context = context

        self._widget = PluginManagerWidget(self, context)

        if self_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self_context.serial_number()))

        self_context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


# Plugin Manager Widget
class PluginManagerWidget(QWidget):

    _NUM_DESC_ATTRIBUTES = 5

    plugin_states_updated_signal = Signal(list)

    def __init__(self, parent, context):
        super(PluginManagerWidget, self).__init__()

        self.namespace = '/'
        self.plugin_states_update_sub = None
        self.load_plugin_set_client = None
        self.get_plugin_descriptions_client = None
        self.get_plugin_states_client = None
        self.add_plugin_client = None
        self.remove_plugin_client = None
        self.plugin_descriptions = []
        self.add_plugin_selection_filter = PluginDescription()
        
        # load from ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('vigir_pluginlib_manager'), 'resource', 'plugin_manager.ui')
        loadUi(ui_file, self, {'QWidget': QWidget})
        self.setObjectName('PluginManagerUi')

        # init tree view
        tree_view = self.plugin_tree_view
        tree_view.setSelectionMode(QAbstractItemView.ExtendedSelection)

        tree_view.setContextMenuPolicy(Qt.CustomContextMenu)
        tree_view.customContextMenuRequested.connect(self._open_context_menu)

        self.plugin_tree_model = PluginTreeModel()
        tree_view.setModel(self.plugin_tree_model)

        # set up combo boxes
        self.pluginNameComboBox.setInsertPolicy(QComboBox.NoInsert)

        # references to combo boxes
        self.plugin_cb = []
        self.plugin_cb.append(self.pluginNameComboBox)
        self.plugin_cb.append(self.pluginTypeClassComboBox)
        self.plugin_cb.append(self.pluginTypePackageComboBox)
        self.plugin_cb.append(self.pluginBaseClassComboBox)
        self.plugin_cb.append(self.pluginBasePackageComboBox)

        # init signal mapper
        self.plugin_cb_mapper = QSignalMapper(self)
        self.plugin_cb_mapper.mapped.connect(self.add_plugin_selection_changed)

        # connect to signals
        for i in range(len(self.plugin_cb)):
            self.plugin_cb_mapper.setMapping(self.plugin_cb[i], i)
            self.plugin_cb[i].currentIndexChanged.connect(self.plugin_cb_mapper.map)
        self.namespaceComboBox.currentIndexChanged[str].connect(self.set_namespace)
        self.refreshAllPushButton.clicked[bool].connect(self.search_namespace)
        self.refreshAllPushButton.clicked[bool].connect(self.refresh_plugin_descriptions)
        self.refreshAllPushButton.clicked[bool].connect(self.refresh_plugin_states)
        self.loadPluginSetPushButton.clicked[bool].connect(self.load_plugin_set)
        self.refreshPluginStatesPushButton.clicked[bool].connect(self.refresh_plugin_sets)
        self.refreshPluginStatesPushButton.clicked[bool].connect(self.refresh_plugin_descriptions)
        self.refreshPluginStatesPushButton.clicked[bool].connect(self.refresh_plugin_states)
        self.addPluginPushButton.clicked[bool].connect(self.add_plugin)
        self.clearAddPluginSelectionPushButton.clicked[bool].connect(self.clear_add_plugin_selection)
        self.removePluginsPushButton.clicked[bool].connect(self.remove_plugins)

        # Qt signals
        self.plugin_states_updated_signal.connect(self.update_plugin_tree_view)
        #self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)

        # init plugin tree view
        self.search_namespace()

    def shutdown_plugin(self):
        print 'Shutting down ...'
        self.plugin_states_update_sub.unregister()
        print 'Done!'

    def _open_context_menu(self, position):
        indexes = self.plugin_tree_view.selectedIndexes()
        level = -1
        if len(indexes) > 0:
            level = 0
            index = indexes[0]
            while index.parent().isValid():
                index = index.parent()
                level += 1

        menu = QMenu()
        if level == 0:
            expand_action = QAction(self.tr('Expand'), None)
            expand_action.triggered.connect(self.plugin_tree_view.expandAll)
            menu.addAction(expand_action)
        if level == 0 or level == 1:
            remove_action = QAction(self.tr('Remove'), None)
            remove_action.triggered.connect(self.remove_plugins)
            menu.addAction(remove_action)

        menu.exec_(self.plugin_tree_view.viewport().mapToGlobal(position))

    def init_topics(self, namespace):
        # init subscribers
        self.plugin_states_update_sub = rospy.Subscriber(namespace + 'plugin_manager/plugin_states_update', PluginStates, self.plugin_states_update)

        # init action clients
        self.load_plugin_set_client = actionlib.SimpleActionClient(namespace + 'plugin_manager/load_plugin_set', PluginManagementAction)
        self.get_plugin_descriptions_client = actionlib.SimpleActionClient(namespace + 'plugin_manager/get_plugin_descriptions', GetPluginDescriptionsAction)
        self.get_plugin_states_client = actionlib.SimpleActionClient(namespace + 'plugin_manager/get_plugin_states', GetPluginStatesAction)
        self.add_plugin_client = actionlib.SimpleActionClient(namespace + 'plugin_manager/add_plugin', PluginManagementAction)
        self.remove_plugin_client = actionlib.SimpleActionClient(namespace + 'plugin_manager/remove_plugin', PluginManagementAction)

        print("Switched to namespace '" + namespace + "'")

    def _set_data_in_description(self, description, index, data):
        if index == 0:
            description.name = data
        if index == 1:
            description.type_class = data
        if index == 2:
            description.type_class_package = data
        if index == 3:
            description.base_class = data
        if index == 4:
            description.base_class_package = data
        return description

    def _get_data_from_description(self, description, index):
        if index == 0:
            return description.name
        if index == 1:
            return description.type_class
        if index == 2:
            return description.type_class_package
        if index == 3:
            return description.base_class
        if index == 4:
            return description.base_class_package

    def filter_descriptions(self, filtered_list, description_filter):
        result = filtered_list
        for i in range(self._NUM_DESC_ATTRIBUTES):
            if not self._get_data_from_description(description_filter, i):
                continue
            result = filter(lambda d: self._get_data_from_description(description_filter, i) == self._get_data_from_description(d, i), result)
        return result

    @Slot()
    def search_namespace(self):
        cb = self.namespaceComboBox
        cb.blockSignals(True)
        cb.setEnabled(False)
        cb.clear()
        cb.addItem('Updating...')

        # get topic list
        _, _, topic_type = rospy.get_master().getTopicTypes()
        topic_dict = dict(topic_type)
        # filter list
        topic_dict_filtered = dict()
        for k, v in topic_dict.items():
            if v == 'vigir_pluginlib_msgs/GetPluginStatesActionGoal':
                topic_dict_filtered[k] = v

        # update combo box with found namespaces
        cb.clear()

        namespaces = [ns[:-37] for ns in sorted(topic_dict_filtered.keys())]
        cb.addItems(namespaces)

        if cb.count() > 0:
            self.set_namespace(cb.currentText())
            cb.setEnabled(True)
            cb.blockSignals(False)
        else:
            cb.addItem('No topics available!')

    @Slot(str)
    def set_namespace(self, namespace):
        self.namespace = namespace
        self.init_topics(namespace)
        self.refresh_plugin_sets()
        self.refresh_plugin_descriptions()
        self.refresh_plugin_states()

    @Slot()
    def refresh_plugin_sets(self):
        plugin_sets = []
        if rospy.has_param(self.namespace+'/plugin_sets'):
            plugin_sets = rospy.get_param(self.namespace+'/plugin_sets').keys()

        cb = self.loadPluginSetComboBox
        cb.clear()

        if plugin_sets:
            cb.addItems(plugin_sets)
            cb.setEnabled(True)
            self.loadPluginSetPushButton.setEnabled(True)
        else:
            cb.setEnabled(False)
            self.loadPluginSetPushButton.setEnabled(False)

    @Slot()
    def load_plugin_set(self):
        if self.load_plugin_set_client.wait_for_server(rospy.Duration(1.0)):
            # send request to server
            goal = PluginManagementGoal()
            goal.name = self.loadPluginSetComboBox.currentText()
            self.load_plugin_set_client.send_goal(goal)

    @Slot(int)
    def add_plugin_selection_changed(self, index):
        # update filter mask
        self._set_data_in_description(self.add_plugin_selection_filter, index, self.plugin_cb[index].currentText())

        # block signals of combo boxes
        for cb in self.plugin_cb:
            cb.blockSignals(True)

        # filter elements in combo boxes
        filtered_descriptions = self.filter_descriptions(self.plugin_descriptions, self.add_plugin_selection_filter)

        for cb_index in range(self._NUM_DESC_ATTRIBUTES):
            if cb_index != index:
                rows_enabled = 0
                last_enabled_row_index = 0

                data = [self._get_data_from_description(d, cb_index) for d in filtered_descriptions]
                item_texts = [self.plugin_cb[cb_index].itemText(i) for i in range(self.plugin_cb[cb_index].count())]
                for row in range(1, len(item_texts)):
                    if not item_texts[row] or item_texts[row] in data:
                        self.plugin_cb[cb_index].setItemData(row, 33, Qt.UserRole - 1)  # enable item
                        rows_enabled += 1
                        last_enabled_row_index = row
                    else:
                        self.plugin_cb[cb_index].setItemData(row, 0, Qt.UserRole - 1)   # disable item

                # if only one element is left, then auto select it
                if rows_enabled == 1:
                    self.plugin_cb[cb_index].setCurrentIndex(last_enabled_row_index)

        # unblock signals of combo boxes
        for cb in self.plugin_cb:
            cb.blockSignals(False)

        self.addPluginPushButton.setEnabled(True)

    @Slot()
    def clear_add_plugin_selection(self):
        # block signals of combo boxes
        for cb in self.plugin_cb:
            cb.blockSignals(True)

        self.plugin_cb[0].clearEditText()
        for cb in self.plugin_cb:
            cb.setCurrentIndex(0)

        # reset selection filter
        self.add_plugin_selection_filter = PluginDescription()
        for cb in self.plugin_cb:
            for row in range(cb.count()):
                cb.setItemData(row, 33, Qt.UserRole - 1)    # enable item

        # unblock signals of combo boxes
        for cb in self.plugin_cb:
            cb.blockSignals(False)

        self.addPluginPushButton.setEnabled(False)

    @Slot()
    def refresh_plugin_descriptions(self):
        # clear old status
        self.plugin_descriptions = []

        for cb in self.plugin_cb:
            cb.blockSignals(True)
            cb.clear()

        self.addPluginPushButton.setEnabled(False)

        # collect all plugin descriptions from manager
        if self.get_plugin_descriptions_client.wait_for_server(rospy.Duration(1.0)):
            self.get_plugin_descriptions_client.send_goal(GetPluginDescriptionsGoal())
            if self.get_plugin_descriptions_client.wait_for_result(rospy.Duration(5.0)):
                self.plugin_descriptions = self.get_plugin_descriptions_client.get_result().descriptions

        # collect all plugins loaded into param server
        all_params = rosparam.list_params(self.namespace)
        for pname in all_params:
            # remove the plugin manager namespace
            if self.namespace == '/':
                pname_sub = pname
            else:
                pname_sub = pname[len(self.namespace):]
            psplit = pname_sub.split('/')

            # get plugin description from param server
            if len(psplit) >= 2 and psplit[1] == 'type_class':
                description = PluginDescription()
                description.name = psplit[0]
                description.type_class = rospy.get_param(pname)
                if rospy.has_param(self.namespace+psplit[0]+'/type_class_package'):
                    description.type_class_package = rospy.get_param(self.namespace+psplit[0]+'/type_class_package')
                if rospy.has_param(self.namespace+psplit[0]+'/base_class'):
                    description.base_class = rospy.get_param(self.namespace+psplit[0]+'/base_class')
                if rospy.has_param(self.namespace+psplit[0]+'/base_class_package'):
                    description.base_class_package = rospy.get_param(self.namespace+psplit[0]+'/base_class_package')
                self.plugin_descriptions.append(description)

        # prepare combo box item texts
        description = [[''] for i in range(self._NUM_DESC_ATTRIBUTES)]
        for pd in self.plugin_descriptions:
            for i in range(self._NUM_DESC_ATTRIBUTES):
                description[i].append(self._get_data_from_description(pd, i))

        # update combo boxes
        for i in range(len(self.plugin_cb)):
            description[i] = sorted(list(set(description[i])))
            self.plugin_cb[i].addItems(description[i])

        for cb in self.plugin_cb:
            cb.blockSignals(False)

    @Slot()
    def refresh_plugin_states(self):
        if self.get_plugin_states_client.wait_for_server(rospy.Duration(1.0)):
            self.get_plugin_states_client.send_goal(GetPluginStatesGoal())
            if self.get_plugin_states_client.wait_for_result(rospy.Duration(5.0)):
                self.plugin_states_updated_signal.emit(self.get_plugin_states_client.get_result().states)

    @Slot(PluginStates)
    def plugin_states_update(self, plugin_states_msg):
        self.plugin_states_updated_signal.emit(plugin_states_msg.states)

    @Slot(list)
    def update_plugin_tree_view(self, states):
        self.plugin_tree_model.updateData(states)
        #self.plugin_tree_view.setModel(self.plugin_tree_model)
        #for column in range(0, self.plugin_tree_model.columnCount()):
        #    self.plugin_tree_model.resizeColumnToContents(column)
        #self.plugin_tree_view.expandAll()

    @Slot()
    def add_plugin(self):
        if self.add_plugin_client.wait_for_server(rospy.Duration(1.0)):
            # generate plugin description
            description = PluginDescription()
            for i in range(self._NUM_DESC_ATTRIBUTES):
                self._set_data_in_description(description, i, self.plugin_cb[i].currentText())

            # send request to server
            goal = PluginManagementGoal()
            goal.descriptions.append(description)
            self.add_plugin_client.send_goal(goal)

    @Slot()
    def remove_plugins(self):
        indexes = self.plugin_tree_view.selectionModel().selectedIndexes()
        indexes = filter(lambda index: index.column() == 0, indexes)

        # extract plugin descriptions from selection
        descriptions = []
        for index in indexes:
            descriptions.append(index.internalPointer().getPluginState().description)

        # send request to server
        if self.remove_plugin_client.wait_for_server(rospy.Duration(1.0)):
            goal = PluginManagementGoal()
            goal.descriptions = descriptions
            self.remove_plugin_client.send_goal(goal)
