#!/usr/bin/env python

import QtCore

import rospy
import rospkg

from python_qt_binding.QtCore import Qt, QObject, QAbstractItemModel

from vigir_pluginlib.msg import PluginState


# Tree Model for Plugins
class PluginTreeModel(QtCore.QAbstractItemModel):

    def __init__(self, parent=None, *args):
        super(PluginTreeModel, self).__init__(parent)
        self._root_item = PluginTreeItem()
        self._plugin_descriptions = []

    def clear(self):
        self._root_item.clear()

    def setData(self, data):
        self.clear()
        for state in data:
            self._root_item.appendChild(PluginTreeItem(state, self._root_item))

    def columnCount(self, parent=QtCore.QModelIndex()):
        if parent.isValid():
            return parent.internalPointer().columnCount()
        else:
            return self._root_item.columnCount()

    def rowCount(self, parent=QtCore.QModelIndex()):
        if parent.isValid():
            return parent.internalPointer().childCount()
        else:
            return self._root_item.childCount()

    def headerData(self, section, orientation, role):
        if orientation == QtCore.Qt.Horizontal and role == QtCore.Qt.DisplayRole:
            if section == 0:
                return "Name"
            elif section == 1:
                return "Base Class"
            elif section == 2:
                return "Type Class"
        return None

    def data(self, index, role):
        if not index.isValid():
            return None
        elif role == QtCore.Qt.DisplayRole:
            return index.internalPointer().data(index.column())
        else:
            return None

    def flags(self, index=QtCore.QModelIndex()):
        if index.isValid():
            return super(PluginTreeModel, self).flags(index) # QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |
        else:
            return QtCore.Qt.NoItemFlags

    def index(self, row, column, parent=QtCore.QModelIndex()):
        if not self.hasIndex(row, column, parent):
            return QtCore.QModelIndex()

        if parent.isValid():
            parent_item = parent.internalPointer()
        else:
            parent_item = self._root_item

        child_item = parent_item.child(row)
        if child_item is not None:
            return self.createIndex(row, column, child_item)
        else:
            return QtCore.QModelIndex()

    def parent(self, index=QtCore.QModelIndex()):
        if not index.isValid():
            return QtCore.QModelIndex()

        child_item = index.internalPointer()
        parent_item = child_item.parentItem()

        if parent_item == self._root_item:
            return QtCore.QModelIndex()
        else:
            return self.createIndex(parent_item.row(), 0, parent_item)


# Tree Item for Plugins
class PluginTreeItem:

    def __init__(self, data=PluginState(), parent=None):
        self._parent_item = parent
        self._child_items = []
        self._plugin_description = None
        self.setData(data)

    def clear(self):
        for child in self._child_items:
            child.clear()
        self._child_items = []
        self._plugin_description = []

    def setData(self, data):
        self._plugin_description = data.description

    def appendChild(self, item):
        item._parent_item = self
        self._child_items.append(item)

    def childCount(self):
        return len(self._child_items)

    def child(self, row):
        if row < self.childCount():
            return self._child_items[row]
        else:
            return None

    def columnCount(self):
        return 3

    def data(self, column):
        if column == 0:
            return self._plugin_description.name.data
        elif column == 1:
            return self._plugin_description.type_class.data
        elif column == 2:
            return self._plugin_description.base_class.data
        else:
            return None

    def parentItem(self):
        return self._parent_item

    def row(self):
        if self._parent_item is not None:
            return self._parent_item._childItems.index(self)
        else:
            return 0
