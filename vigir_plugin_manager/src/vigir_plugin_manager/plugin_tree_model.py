#!/usr/bin/env python

import QtCore

import rospy
import rospkg

from python_qt_binding.QtCore import Qt, QObject, QAbstractItemModel

from vigir_pluginlib.msg import PluginState, PluginDescription


# Tree Model for Plugins
class PluginTreeModel(QtCore.QAbstractItemModel):

    def __init__(self, parent=None, *args):
        super(PluginTreeModel, self).__init__(parent)
        self._root_item = PluginTreeItem()
        self._plugin_states = []

    def clear(self):
        self._root_item.clear()

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
                return "Class"
            elif section == 1:
                return "Name"
        return None

    def insertRows(self, position, count, parent=QtCore.QModelIndex()):
        parent_item = self.getItem(parent)

        self.beginInsertRows(parent, position, position + count - 1)
        success = parent_item.insertChildren(position, count)
        self.endInsertRows()

        return success

    def removeRows(self, position, count, parent=QtCore.QModelIndex()):
        parent_item = self.getItem(parent)

        self.beginRemoveRows(parent, position, position + count - 1)
        success = parent_item.removeChildren(position, count)
        self.endRemoveRows()

        # remove empty branch
        if success and parent_item.parentItem() and not parent_item.childCount():
            return self.removeRows(parent_item.childNumber(), 1)

        return success

    def addBranch(self, base_class):
        state = PluginState()
        state.description.base_class.data = base_class

        position = self._root_item.childCount()
        if self.insertRows(position, 1):
            branch_item = self._root_item.child(position)
            branch_item.setData(state)
            return self.index(self._root_item.childCount()-1, 0)

        return QtCore.QModelIndex()

    def addItem(self, state):
        if not state.description.base_class.data:
            state.description.base_class.data = "Unknown"
        if not state.description.type_class.data:
            state.description.type_class.data = "Unknown"

        # Search for branch with type_class
        branch = self.findBranch(state.description.base_class.data)

        # Otherwise create branch with type class
        if not branch.isValid():
            branch = self.addBranch(state.description.base_class.data)

        branch_item = self.getItem(branch)

        # check if child already does exist
        child = self.findChild(state.description, branch)
        if child.isValid():
            return self.getItem(child)

        # Add new item to branch
        position = branch_item.childCount()
        if self.insertRows(position, 1, branch):
            child_item = branch_item.child(position)
            child_item.setData(state)
            return self.index(child_item.childNumber(), 0, branch)

        return QtCore.QModelIndex()

    def getItem(self, index=QtCore.QModelIndex()):
        if index.isValid():
            return index.internalPointer()
        else:
            return self._root_item

    def setData(self, data):
        self.clear()
        for state in data:
            self.addItem(state)

    def updateData(self, data):
        # removing old entries
        # TODO
        # updating entries
        for state in data:
            self.addItem(state)

    def data(self, index, role):
        if not index.isValid():
            return None
        elif role == QtCore.Qt.DisplayRole:
            return index.internalPointer().data(index.column())
        else:
            return None

    def flags(self, index=QtCore.QModelIndex()):
        if index.isValid():
            return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable | super(PluginTreeModel, self).flags(index) #QtCore.Qt.ItemIsEditable
        else:
            return QtCore.Qt.NoItemFlags

    def index(self, row, column, parent=QtCore.QModelIndex()):
        if not self.hasIndex(row, column, parent):
            return QtCore.QModelIndex()

        if parent.isValid() and parent.column() != 0:
            return QtCore.QModelIndex()

        parent_item = self.getItem(parent)

        child_item = parent_item.child(row)
        if child_item:
            return self.createIndex(row, column, child_item)
        else:
            return QtCore.QModelIndex()

    def parent(self, index=QtCore.QModelIndex()):
        if not index.isValid():
            return QtCore.QModelIndex()

        child_item = self.getItem(index)
        parent_item = child_item.parentItem()

        if parent_item == self._root_item:
            return QtCore.QModelIndex()

        return self.createIndex(parent_item.childNumber(), 0, parent_item)

    def findChild(self, description, parent=QtCore.QModelIndex()):
        parent_item = self.getItem(parent)

        child_item = parent_item.findChild(description)
        if child_item:
            return self.index(child_item.childNumber(), 0, parent)
        else:
            return QtCore.QModelIndex()

    def findBranch(self, base_class, parent=QtCore.QModelIndex()):
        parent_item = self.getItem(parent)

        child_item = parent_item.findBranch(base_class)
        if child_item:
            return self.index(child_item.childNumber(), 0, parent)
        else:
            return QtCore.QModelIndex()

    def expandChildren(self, index, view):
        if not index.isValid():
            return

        for i in range(0, index.model().rowCount(index)):
            child = index.child(i, 0)
            self.expandChildren(child, view)

        if not view.expanded(index):
            view.expand(index)

    def expandAll(self):
        self.expandChildren(self.createIndex(0, 0, self._root_item))


# Tree Item for Plugins
class PluginTreeItem:

    def __init__(self, state=PluginState(), parent=None):
        self._parent_item = parent
        self._child_items = []
        self._plugin_state = PluginState()
        self.setData(state)

    def clear(self):
        for child in self._child_items:
            child.clear()
        self._child_items = []
        self._plugin_state = PluginState()

    def child(self, row):
        if row < self.childCount():
            return self._child_items[row]
        else:
            return None

    def childCount(self):
        return len(self._child_items)

    def childNumber(self):
        if self._parent_item is not None:
            return self._parent_item._child_items.index(self)
        return 0

    def insertChildren(self, position, count):
        if position < 0 or position > self.childCount():
            return False

        for row in range(0, count):
            self._child_items.insert(position, PluginTreeItem(parent=self))

        return True

    def removeChildren(self, position, count):
        if position < 0 or position > self.childCount():
            return False

        del self._child_items[position:position+count]

        return True

    def columnCount(self):
        return 2

    def data(self, column):
        if column == 0:
            if self.childCount() > 0:
                return self._plugin_state.description.base_class.data
            else:
                return self._plugin_state.description.type_class.data
        elif column == 1:
            if self.childCount() > 0:
                return ""
            else:
                return self._plugin_state.description.name.data
        else:
            return None

    def setData(self, state):
        self._plugin_state = state

    def getPluginState(self):
        return self._plugin_state

    def findChild(self, description):
        child = filter(lambda child: child.getPluginState().description == description, self._child_items)
        if not child:
            return None
        else:
            return child[0]

    def findBranch(self, base_class):
        branch = filter(lambda child: child.getPluginState().description.base_class.data == base_class, self._child_items)
        if not branch:
            return None
        else:
            return branch[0]

    def parentItem(self):
        return self._parent_item
