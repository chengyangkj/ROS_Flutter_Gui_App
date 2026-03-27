import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/display/tile_map.dart';

abstract class MapEditCommand {
  void execute();
  void undo();
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

class AddPointCommand extends MapEditCommand {
  final TopologyMap topologyMap;
  final NavPoint point;
  final VoidCallback onUpdate;

  AddPointCommand(this.topologyMap, this.point, this.onUpdate);

  @override
  void execute() {
    topologyMap.points.add(point);
    onUpdate();
  }

  @override
  void undo() {
    topologyMap.points.removeWhere((p) => p.name == point.name);
    onUpdate();
  }
}

class DeletePointCommand extends MapEditCommand {
  final TopologyMap topologyMap;
  final NavPoint point;
  final VoidCallback onUpdate;

  DeletePointCommand(this.topologyMap, this.point, this.onUpdate);

  @override
  void execute() {
    topologyMap.points.removeWhere((p) => p.name == point.name);
    onUpdate();
  }

  @override
  void undo() {
    topologyMap.points.add(point);
    onUpdate();
  }
}

class RenamePointCommand extends MapEditCommand {
  final TopologyMap topologyMap;
  final NavPoint oldPoint;
  final NavPoint newPoint;
  final VoidCallback onUpdate;

  RenamePointCommand(this.topologyMap, this.oldPoint, this.newPoint, this.onUpdate);

  void _apply() {
    final index = topologyMap.points.indexWhere((p) => p.name == oldPoint.name);
    if (index != -1) {
      topologyMap.points[index] = newPoint;
    }

    for (final route in topologyMap.routes) {
      if (route.fromPoint == oldPoint.name) {
        route.fromPoint = newPoint.name;
      }
      if (route.toPoint == oldPoint.name) {
        route.toPoint = newPoint.name;
      }
    }
  }

  void _applyUndo() {
    final index = topologyMap.points.indexWhere((p) => p.name == newPoint.name);
    if (index != -1) {
      topologyMap.points[index] = oldPoint;
    }

    for (final route in topologyMap.routes) {
      if (route.fromPoint == newPoint.name) {
        route.fromPoint = oldPoint.name;
      }
      if (route.toPoint == newPoint.name) {
        route.toPoint = oldPoint.name;
      }
    }
  }

  @override
  void execute() {
    _apply();
    onUpdate();
  }

  @override
  void undo() {
    _applyUndo();
    onUpdate();
  }
}

class ModifyPointCommand extends MapEditCommand {
  final TopologyMap topologyMap;
  final NavPoint oldPoint;
  final NavPoint newPoint;
  final VoidCallback onUpdate;

  ModifyPointCommand(this.topologyMap, this.oldPoint, this.newPoint, this.onUpdate);

  @override
  void execute() {
    final index = topologyMap.points.indexWhere((p) => p.name == oldPoint.name);
    if (index != -1) {
      topologyMap.points[index] = newPoint;
      onUpdate();
    }
  }

  @override
  void undo() {
    final index = topologyMap.points.indexWhere((p) => p.name == newPoint.name);
    if (index != -1) {
      topologyMap.points[index] = oldPoint;
      onUpdate();
    }
  }
}

class CommandManager {
  final List<MapEditCommand> _undoStack = [];
  final int maxHistorySize = 50;

  void executeCommand(MapEditCommand command) {
    command.execute();
    _undoStack.add(command);
    if (_undoStack.length > maxHistorySize) {
      _undoStack.removeAt(0);
    }
  }

  // 只记录命令，不执行（用于已经执行过的操作）
  void recordCommand(MapEditCommand command) {
    _undoStack.add(command);
    if (_undoStack.length > maxHistorySize) {
      _undoStack.removeAt(0);
    }
  }

  void undo() {
    if (_undoStack.isEmpty) return;
    final command = _undoStack.removeLast();
    command.undo();
  }

  bool canUndo() => _undoStack.isNotEmpty;

  void clear() {
    _undoStack.clear();
  }
}

class ObstacleEditCommand extends MapEditCommand {
  final GlobalKey<TileMapState> tileMapKey;
  final Map<int, int> oldEdits;
  final Map<int, int> newEdits;

  ObstacleEditCommand(this.tileMapKey, this.oldEdits, this.newEdits);

  @override
  void execute() {
    tileMapKey.currentState?.applyObstacleEdits(newEdits);
  }

  @override
  void undo() {
    tileMapKey.currentState?.applyObstacleEdits(oldEdits);
  }
}

