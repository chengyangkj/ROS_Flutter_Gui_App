import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';

abstract class MapEditCommand {
  void execute();
  void undo();
}

class DrawObstacleCommand extends MapEditCommand {
  final OccupancyMap map;
  final List<MapEntry<int, int>> modifiedCells;
  final List<int> originalValues;

  DrawObstacleCommand(this.map, this.modifiedCells) 
      : originalValues = modifiedCells.map((cell) => map.data[cell.key][cell.value]).toList();

  @override
  void execute() {
    for (var cell in modifiedCells) {
      final row = cell.key;
      final col = cell.value;
      if (row >= 0 && row < map.Rows() && col >= 0 && col < map.Cols()) {
        map.data[row][col] = 100;
      }
    }
  }

  @override
  void undo() {
    for (int i = 0; i < modifiedCells.length; i++) {
      final cell = modifiedCells[i];
      final row = cell.key;
      final col = cell.value;
      if (row >= 0 && row < map.Rows() && col >= 0 && col < map.Cols()) {
        map.data[row][col] = originalValues[i];
      }
    }
  }
}

class EraseObstacleCommand extends MapEditCommand {
  final OccupancyMap map;
  final List<MapEntry<int, int>> modifiedCells;
  final List<int> originalValues;

  EraseObstacleCommand(this.map, this.modifiedCells)
      : originalValues = modifiedCells.map((cell) => map.data[cell.key][cell.value]).toList();

  @override
  void execute() {
    for (var cell in modifiedCells) {
      final row = cell.key;
      final col = cell.value;
      if (row >= 0 && row < map.Rows() && col >= 0 && col < map.Cols()) {
        map.data[row][col] = 0;
      }
    }
  }

  @override
  void undo() {
    for (int i = 0; i < modifiedCells.length; i++) {
      final cell = modifiedCells[i];
      final row = cell.key;
      final col = cell.value;
      if (row >= 0 && row < map.Rows() && col >= 0 && col < map.Cols()) {
        map.data[row][col] = originalValues[i];
      }
    }
  }
}

class AddRouteCommand extends MapEditCommand {
  final TopologyMap topologyMap;
  final TopologyRoute route;
  final VoidCallback onUpdate;

  AddRouteCommand(this.topologyMap, this.route, this.onUpdate);

  @override
  void execute() {
    topologyMap.routes.add(route);
    onUpdate();
  }

  @override
  void undo() {
    topologyMap.routes.remove(route);
    onUpdate();
  }
}

class DeleteRouteCommand extends MapEditCommand {
  final TopologyMap topologyMap;
  final TopologyRoute route;
  final VoidCallback onUpdate;

  DeleteRouteCommand(this.topologyMap, this.route, this.onUpdate);

  @override
  void execute() {
    topologyMap.routes.remove(route);
    onUpdate();
  }

  @override
  void undo() {
    topologyMap.routes.add(route);
    onUpdate();
  }
}

class ModifyRouteCommand extends MapEditCommand {
  final TopologyMap topologyMap;
  final TopologyRoute oldRoute;
  final TopologyRoute newRoute;
  final VoidCallback onUpdate;

  ModifyRouteCommand(this.topologyMap, this.oldRoute, this.newRoute, this.onUpdate);

  @override
  void execute() {
    final index = topologyMap.routes.indexOf(oldRoute);
    if (index != -1) {
      topologyMap.routes[index] = newRoute;
      onUpdate();
    }
  }

  @override
  void undo() {
    final index = topologyMap.routes.indexOf(newRoute);
    if (index != -1) {
      topologyMap.routes[index] = oldRoute;
      onUpdate();
    }
  }
}

class ModifyPointCommand extends MapEditCommand {
  final List<NavPoint> points;
  final NavPoint oldPoint;
  final NavPoint newPoint;
  final VoidCallback onUpdate;

  ModifyPointCommand(this.points, this.oldPoint, this.newPoint, this.onUpdate);

  @override
  void execute() {
    final index = points.indexWhere((p) => p.name == oldPoint.name);
    if (index != -1) {
      points[index] = newPoint;
      onUpdate();
    }
  }

  @override
  void undo() {
    final index = points.indexWhere((p) => p.name == newPoint.name);
    if (index != -1) {
      points[index] = oldPoint;
      onUpdate();
    }
  }
}

class CommandManager {
  final List<MapEditCommand> _undoStack = [];
  final List<MapEditCommand> _redoStack = [];
  final int maxHistorySize = 50;

  void executeCommand(MapEditCommand command) {
    command.execute();
    _undoStack.add(command);
    if (_undoStack.length > maxHistorySize) {
      _undoStack.removeAt(0);
    }
    _redoStack.clear();
  }

  void undo() {
    if (_undoStack.isEmpty) return;
    final command = _undoStack.removeLast();
    command.undo();
    _redoStack.add(command);
  }

  void redo() {
    if (_redoStack.isEmpty) return;
    final command = _redoStack.removeLast();
    command.execute();
    _undoStack.add(command);
  }

  bool canUndo() => _undoStack.isNotEmpty;
  bool canRedo() => _redoStack.isNotEmpty;

  void clear() {
    _undoStack.clear();
    _redoStack.clear();
  }
}

