import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/display/map.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'package:ros_flutter_gui_app/display/pose.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/provider/them_provider.dart';
import 'package:vector_math/vector_math_64.dart' as vm;
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/page/map_edit_page.dart';
import 'package:ros_flutter_gui_app/display/topology_line.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/page/map_edit_command.dart';
import 'dart:math' as math;

// 编辑操作状态枚举
enum EditOperationState {
  none,           // 无操作
  drawingObstacle, // 绘制障碍物
  erasingObstacle, // 擦除障碍物
  draggingPoint,   // 拖动点位
  rotatingPoint,   // 旋转点位
}

// 专门的地图编辑Flame组件
class MapEditFlame extends FlameGame {
  late MapComponent _displayMap;
  late GridComponent _displayGrid;
  final RosChannel? rosChannel;
  final ThemeProvider? themeProvider;
  
  final double minScale = 0.01;
  final double maxScale = 10.0;

  RobotPose currentRobotPose = RobotPose.zero();
  
  // 地图变换参数
  double mapScale = 1.0;

  // 当前选中的编辑工具
  EditToolType? selectedTool;
  
  // 回调函数，用于通知外部添加导航点
  Future<NavPoint?> Function(double x, double y, {double theta})? onAddNavPoint;
  
  // 回调函数，用于通知外部导航点选择状态变化
  VoidCallback? onWayPointSelectionChanged;
  
  // 回调函数，用于通知外部当前选中点位动态更新
  VoidCallback? currentSelectPointUpdate;
  
  // 导航点组件列表（合并在线和离线导航点）
  final List<PoseComponent> wayPoints = [];
  
  PoseComponent? currentSelectedWayPoint;
  
  // 拓扑相关组件
  late TopologyLine _topologyLineComponent;
  List<NavPoint> offLineNavPoints = [];
  
  // 手势相关变量
  double _baseScale = 1.0;
  Vector2? _lastFocalPoint;
  
  // 障碍物编辑相关
  CommandManager commandManager = CommandManager();
  EditOperationState _operationState = EditOperationState.none;
  Vector2? _lastObstaclePosition;
  List<GridCellChange> _currentGridChanges = [];
  Map<String, int> _initialGridValues = {}; // 保存首次修改时的原始值
  double _obstacleBrushSize = 0.05;
  
  // 移动工具相关
  PoseComponent? _draggedPoint;
  Vector2? _dragStartPos;
  NavPoint? _dragStartPointData;
  
  // 拓扑连线相关
  String? _routeStartPoint;
  
  // 直线绘制相关
  Vector2? _lineStartPoint;
  
  // 设置画笔大小
  void setBrushSize(double size) {
    _obstacleBrushSize = size;
  }
  
  // 获取画笔大小
  double getBrushSize() => _obstacleBrushSize;
  
  MapEditFlame({
    this.rosChannel, 
    this.themeProvider,
    this.onAddNavPoint, 
    this.onWayPointSelectionChanged,
    this.currentSelectPointUpdate,
  }) {
    // 初始化拓扑线组件
    _topologyLineComponent = TopologyLine(
      points: [],
      routes: [],
      rosChannel: rosChannel,
    );
  }
  
  @override
  Color backgroundColor() =>  Colors.white;

  
  // 设置当前选中的工具
  void setSelectedTool(EditToolType? tool) {
    selectedTool = tool;
  }
  
  Future<void> addNavPointAtRobotPosition() async {
    if (selectedTool != EditToolType.addNavPoint) return;
    
    final navPoint = await onAddNavPoint!(currentRobotPose.x, currentRobotPose.y, theta: currentRobotPose.theta);
    if (navPoint != null && rosChannel != null) {
      final topologyMap = rosChannel!.mapManager.topologyMap.value;
      final command = AddPointCommand(topologyMap, navPoint, () {
        _updateTopologyLayers();
        rosChannel!.updateTopologyMap(topologyMap);
      });
      commandManager.executeCommand(command);
    }
  }
  
  // 获取当前选中的导航点
  PoseComponent? get selectedWayPoint {
    return currentSelectedWayPoint;
  }
  
  // 获取当前选中导航点的信息
  NavPoint? getSelectedWayPointInfo() {
    final wayPoint = selectedWayPoint;
    if (wayPoint == null) return null;
    var x = wayPoint.position.x;
    var y = wayPoint.position.y;
    var direction = -wayPoint.direction;
    double mapx = 0;
    double mapy = 0;
    if(rosChannel != null) {
      vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(x, y));
      mapx = mapPose.x;
      mapy = mapPose.y;
    }

    var pointInfo = wayPoint.getPointInfo();
    if (pointInfo != null) {
      pointInfo.x=mapx;
      pointInfo.y=mapy;
      pointInfo.theta=direction;
    }

    return pointInfo;
  }
  
  List<NavPoint> getAllWayPoint() {
    List<NavPoint> allWayPoints = [];
    
    for (int i = 0; i < wayPoints.length; i++) {
      final wayPoint = wayPoints[i];
      var x = wayPoint.position.x;
      var y = wayPoint.position.y;
      var direction = -wayPoint.direction;
      double mapx = 0;
      double mapy = 0;
      
      if (rosChannel != null) {
        vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(x, y));
        mapx = mapPose.x;
        mapy = mapPose.y;
      }

      var pointInfo = wayPoint.getPointInfo();
      if (pointInfo != null) {
        pointInfo.x=mapx;
        pointInfo.y=mapy;
        pointInfo.theta=direction;
      }
      
      allWayPoints.add(pointInfo!);
    }
    
    return allWayPoints;
  }
  
  @override
  Future<void> onLoad() async {
    super.onLoad();
    
    // 添加地图组件
    _displayMap = MapComponent(rosChannel: rosChannel);
    world.add(_displayMap);
    _displayMap.updateThemeMode(false);
    
    // 添加网格组件
    _displayGrid = GridComponent(
      size: size,
      rosChannel: rosChannel,
    );
    _displayGrid.updateThemeMode(false);
    world.add(_displayGrid);
    
    // 设置ROS监听器
    _setupRosListeners();
    
    // 初始化拓扑图层
    _updateTopologyLayers();
  }
  
  void _setupRosListeners() {
    if (rosChannel != null) {
      
      rosChannel!.robotPoseMap.addListener(() {
        currentRobotPose = rosChannel!.robotPoseMap.value;
      });
      
          // 监听拓扑地图数据
      rosChannel!.topologyMap_.addListener(() {
        _updateTopologyLayers();
      });
      // 立即更新地图数据
      _displayMap.updateMapData(rosChannel!.map_.value);
    }
  }
  
  // 处理单击事件
  Future<bool> onTapDown(Vector2 position) async {
    final worldPoint = camera.globalToLocal(position);
    
    if (selectedTool == EditToolType.move) {
      // 移动工具：选中点位
      final clickedWayPoint = _findWayPointAtPosition(worldPoint);
      if (clickedWayPoint != null) {
        _selectWayPoint(clickedWayPoint);
        currentSelectPointUpdate?.call();
        return true;
      }
      return false;
    } else if (selectedTool == EditToolType.addNavPoint) {
      final clickedWayPoint = _findWayPointAtPosition(worldPoint);
      
      if (clickedWayPoint != null) {
        _selectWayPoint(clickedWayPoint);
        currentSelectPointUpdate?.call();
        return true;
      }
      
      double mapX = 0;
      double mapY = 0;
      if (rosChannel != null) {
        vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(worldPoint.x, worldPoint.y));
        mapX = mapPose.x;
        mapY = mapPose.y;
      }

      final navPoint = await onAddNavPoint!(mapX, mapY);
      if (navPoint != null && rosChannel != null) {
        final topologyMap = rosChannel!.mapManager.topologyMap.value;
        final command = AddPointCommand(topologyMap, navPoint, () {
          _updateTopologyLayers();
        });
        commandManager.executeCommand(command);
      }
      return true;
    } else if (selectedTool == EditToolType.addRoute) {
      final clickedWayPoint = _findWayPointAtPosition(worldPoint);
      if (clickedWayPoint != null && clickedWayPoint.navPoint != null) {
        if (_routeStartPoint == null) {
          _routeStartPoint = clickedWayPoint.navPoint!.name;
          _selectWayPoint(clickedWayPoint);
          currentSelectPointUpdate?.call();
        } else if (_routeStartPoint != clickedWayPoint.navPoint!.name) {
          _createRoute(_routeStartPoint!, clickedWayPoint.navPoint!.name);
          _routeStartPoint = null;
        }
        return true;
      } else {
        _routeStartPoint = null;
        return false;
      }
    } else if (selectedTool == EditToolType.drawLine) {
      double mapX = 0;
      double mapY = 0;
      if (rosChannel != null) {
        vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(worldPoint.x, worldPoint.y));
        mapX = mapPose.x;
        mapY = mapPose.y;
      }
      
      if (_lineStartPoint == null) {
        _lineStartPoint = Vector2(mapX, mapY);
        return true;
      } else {
        // 绘制直线
        _drawLine(_lineStartPoint!, Vector2(mapX, mapY));
        _lineStartPoint = null;
        return true;
      }
    } else if (selectedTool == EditToolType.drawObstacle) {
      _operationState = EditOperationState.drawingObstacle;
      _currentGridChanges.clear();
      _initialGridValues.clear();
      _lastObstaclePosition = worldPoint;
      _drawObstacleAtPosition(worldPoint);
      return true;
    } else if (selectedTool == EditToolType.eraseObstacle) {
      _operationState = EditOperationState.erasingObstacle;
      _currentGridChanges.clear();
      _initialGridValues.clear();
      _lastObstaclePosition = worldPoint;
      _eraseObstacleAtPosition(worldPoint);
      return true;
    }
    return false;
  }
  
  // 处理鼠标按下（用于移动工具的左键/右键）
  bool onPointerDown(Vector2 position, int buttons) {
    if (selectedTool == EditToolType.move) {
      final worldPoint = camera.globalToLocal(position);
      final clickedWayPoint = _findWayPointAtPosition(worldPoint);
      
      if (clickedWayPoint != null && clickedWayPoint.navPoint != null) {
        final pointData = clickedWayPoint.navPoint!;
        
        if (buttons == 1) {
          // 左键：开始拖动移动
          _operationState = EditOperationState.draggingPoint;
          _draggedPoint = clickedWayPoint;
          _dragStartPos = worldPoint;
          _dragStartPointData = NavPoint(
            name: pointData.name,
            x: pointData.x,
            y: pointData.y,
            theta: pointData.theta,
            type: pointData.type,
          );
          _selectWayPoint(clickedWayPoint);
          return true;
        } else if (buttons == 2) {
          // 右键：开始拖动旋转
          _operationState = EditOperationState.rotatingPoint;
          _draggedPoint = clickedWayPoint;
          _dragStartPos = worldPoint;
          _dragStartPointData = NavPoint(
            name: pointData.name,
            x: pointData.x,
            y: pointData.y,
            theta: pointData.theta,
            type: pointData.type,
          );
          _selectWayPoint(clickedWayPoint);
          return true;
        }
      }
    }
    return false;
  }
  
  // 处理拖动更新
  bool onPanUpdate(Vector2 position) {
    final worldPoint = camera.globalToLocal(position);
    
    switch (_operationState) {
      case EditOperationState.draggingPoint:
        if (_draggedPoint != null && _dragStartPos != null) {
          double mapX = 0;
          double mapY = 0;
          if (rosChannel != null) {
            vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(worldPoint.x, worldPoint.y));
            mapX = mapPose.x;
            mapY = mapPose.y;
          }
          
          if (_draggedPoint!.navPoint != null) {
            final updatedNavPoint = NavPoint(
              name: _draggedPoint!.navPoint!.name,
              x: mapX,
              y: mapY,
              theta: _draggedPoint!.navPoint!.theta,
              type: _draggedPoint!.navPoint!.type,
            );
            _draggedPoint!.updatePose(RobotPose(mapX, mapY, _draggedPoint!.navPoint!.theta));
            _draggedPoint!.navPoint = updatedNavPoint;
            currentSelectPointUpdate?.call();
          }
          return true;
        }
        break;
        
      case EditOperationState.rotatingPoint:
        if (_draggedPoint != null && _dragStartPos != null) {
          double mapX = 0;
          double mapY = 0;
          if (rosChannel != null) {
            vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(worldPoint.x, worldPoint.y));
            mapX = mapPose.x;
            mapY = mapPose.y;
          }
          
          if (_draggedPoint!.navPoint != null) {
            final pointX = _draggedPoint!.navPoint!.x;
            final pointY = _draggedPoint!.navPoint!.y;
            final dx = mapX - pointX;
            final dy = mapY - pointY;
            final theta = -math.atan2(dy, dx);
            
            final updatedNavPoint = NavPoint(
              name: _draggedPoint!.navPoint!.name,
              x: pointX,
              y: pointY,
              theta: theta,
              type: _draggedPoint!.navPoint!.type,
            );
            _draggedPoint!.updatePose(RobotPose(pointX, pointY, theta));
            _draggedPoint!.navPoint = updatedNavPoint;
            currentSelectPointUpdate?.call();
          }
          return true;
        }
        break;
        
      case EditOperationState.drawingObstacle:
      case EditOperationState.erasingObstacle:
        if (_lastObstaclePosition != null && rosChannel != null) {
          final distance = (worldPoint - _lastObstaclePosition!).length;
          // 计算两点之间的插值步数，确保连续绘制
          final stepSize = _obstacleBrushSize / 4.0;
          final steps = math.max(1, (distance / stepSize).ceil());
          
          // 批量收集所有要修改的单元格
          final List<MapEntry<int, int>> allCells = [];
          
          for (int i = 0; i <= steps; i++) {
            final t = i / steps;
            final interpolatedPoint = Vector2(
              _lastObstaclePosition!.x + (worldPoint.x - _lastObstaclePosition!.x) * t,
              _lastObstaclePosition!.y + (worldPoint.y - _lastObstaclePosition!.y) * t,
            );
            
            final cells = _getCellsForPosition(interpolatedPoint);
            allCells.addAll(cells);
          }
          
          // 去重
          final uniqueCells = <MapEntry<int, int>>{};
          for (var cell in allCells) {
            uniqueCells.add(cell);
          }
          
          final value = _operationState == EditOperationState.drawingObstacle ? 100 : 0;
          if (uniqueCells.isNotEmpty) {
            final changes = _displayMap.modifyCells(uniqueCells.toList(), value, initialValues: _initialGridValues);
            // 收集变化并更新初始值映射
            for (var change in changes) {
              final key = '${change.row},${change.col}';
              if (!_initialGridValues.containsKey(key)) {
                _initialGridValues[key] = change.oldValue;
              }
              // 更新或添加到当前变化列表
              final existingIndex = _currentGridChanges.indexWhere((c) => c.row == change.row && c.col == change.col);
              if (existingIndex == -1) {
                _currentGridChanges.add(change);
              } else {
                // 更新 newValue，保留原始的 oldValue
                _currentGridChanges[existingIndex] = GridCellChange(
                  row: change.row,
                  col: change.col,
                  oldValue: _currentGridChanges[existingIndex].oldValue,
                  newValue: change.newValue,
                );
              }
            }
          }
          
          _lastObstaclePosition = worldPoint;
        }
        return true;
        
      case EditOperationState.none:
        break;
    }
    return false;
  }
  
  // 处理拖动结束
  bool onPanEnd() {
    switch (_operationState) {
      case EditOperationState.draggingPoint:
      case EditOperationState.rotatingPoint:
        if (_draggedPoint != null && _dragStartPointData != null && _draggedPoint!.navPoint != null && rosChannel != null) {
          final currentPoint = _draggedPoint!.navPoint!;
          if (currentPoint.name == _dragStartPointData!.name) {
            final topologyMap = rosChannel!.mapManager.topologyMap.value;
            final command = ModifyPointCommand(topologyMap, _dragStartPointData!, currentPoint, () {
              _updateTopologyLayers();
              rosChannel!.updateTopologyMap(topologyMap);
            });
            commandManager.executeCommand(command);
          }
        }
        _operationState = EditOperationState.none;
        _draggedPoint = null;
        _dragStartPos = null;
        _dragStartPointData = null;
        return true;
        
      case EditOperationState.drawingObstacle:
      case EditOperationState.erasingObstacle:
        if (_currentGridChanges.isNotEmpty) {
          final command = ModifyGridCommand(
            _displayMap,
            List.from(_currentGridChanges),
            () {},
          );
          // 使用 recordCommand 因为数据已经在拖动过程中被修改了
          commandManager.recordCommand(command);
          _currentGridChanges.clear();
          _initialGridValues.clear();
        }
        _operationState = EditOperationState.none;
        _lastObstaclePosition = null;
        return true;
        
      case EditOperationState.none:
        break;
    }
    return false;
  }
  
  void _createRoute(String fromPoint, String toPoint) {
    if (rosChannel == null) return;
    
    final mapManager = rosChannel!.mapManager;
    
    final newRoute = TopologyRoute(
      fromPoint: fromPoint,
      toPoint: toPoint,
      routeInfo: RouteInfo(controller: 'FollowPath'),
    );
    
    final topologyMap = mapManager.topologyMap.value;
    final command = AddRouteCommand(topologyMap, newRoute, () {
      _updateTopologyLayers();
      rosChannel!.updateTopologyMap(topologyMap);
    });
    
    commandManager.executeCommand(command);
    print('已创建路线: $fromPoint -> $toPoint');
  }
  
  void _drawLine(Vector2 start, Vector2 end) {
    if (rosChannel == null) return;
    
    final map = rosChannel!.map_.value;
    final startIdx = map.xy2idx(vm.Vector2(start.x, start.y));
    final endIdx = map.xy2idx(vm.Vector2(end.x, end.y));
    
    final dx = endIdx.x - startIdx.x;
    final dy = endIdx.y - startIdx.y;
    final steps = math.max(dx.abs().round(), dy.abs().round());
    
    final lineCells = <MapEntry<int, int>>[];
    
    for (int i = 0; i <= steps; i++) {
      final t = i / steps;
      final x = startIdx.x + dx * t;
      final y = startIdx.y + dy * t;
      final row = y.round();
      final col = x.round();
      
      if (row >= 0 && row < map.Rows() && col >= 0 && col < map.Cols()) {
        bool exists = lineCells.any((c) => c.key == row && c.value == col);
        if (!exists) {
          lineCells.add(MapEntry(row, col));
        }
      }
    }
    
    // 应用绘制并获取变化
    if (lineCells.isNotEmpty) {
      final changes = _displayMap.modifyCells(lineCells, OBSTACLE_COLOR);
      if (changes.isNotEmpty) {
        final command = ModifyGridCommand(_displayMap, changes, () {});
        commandManager.recordCommand(command);
      }
    }
  }
  
  List<MapEntry<int, int>> _getCellsForPosition(Vector2 worldPoint) {
    if (rosChannel == null) return [];
    
    final map = rosChannel!.map_.value;
    // 参考 AddPoint 工具：worldPoint 是 Flame 世界坐标（像素），需要先转换为地图世界坐标（米）
    vm.Vector2 mapPose = map.idx2xy(vm.Vector2(worldPoint.x, worldPoint.y));
    final idx = map.xy2idx(mapPose);
    final centerRow = idx.y.round();
    final centerCol = idx.x.round();
    
    final brushRadiusInCells = (_obstacleBrushSize / map.mapConfig.resolution);
    final brushRadius = brushRadiusInCells.ceil();
    
    final List<MapEntry<int, int>> cells = [];
    
    for (int row = centerRow - brushRadius; row <= centerRow + brushRadius; row++) {
      for (int col = centerCol - brushRadius; col <= centerCol + brushRadius; col++) {
        if (row >= 0 && row < map.Rows() && col >= 0 && col < map.Cols()) {
          final distance = math.sqrt(
            math.pow(row - centerRow, 2) + math.pow(col - centerCol, 2)
          );
          if (distance <= brushRadiusInCells) {
            cells.add(MapEntry(row, col));
          }
        }
      }
    }
    
    return cells;
  }
  
  // 在指定位置绘制障碍物
  void _drawObstacleAtPosition(Vector2 worldPoint) {
    final cells = _getCellsForPosition(worldPoint);
    if (cells.isNotEmpty) {
      final changes = _displayMap.modifyCells(cells, OBSTACLE_COLOR, initialValues: _initialGridValues);
      for (var change in changes) {
        final key = '${change.row},${change.col}';
        if (!_initialGridValues.containsKey(key)) {
          _initialGridValues[key] = change.oldValue;
        }
        _currentGridChanges.add(change);
      }
    }
  }
  
  // 在指定位置擦除障碍物
  void _eraseObstacleAtPosition(Vector2 worldPoint) {
    final cells = _getCellsForPosition(worldPoint);
    if (cells.isNotEmpty) {
      final changes = _displayMap.modifyCells(cells, FREE_COLOR, initialValues: _initialGridValues);
      for (var change in changes) {
        final key = '${change.row},${change.col}';
        if (!_initialGridValues.containsKey(key)) {
          _initialGridValues[key] = change.oldValue;
        }
        _currentGridChanges.add(change);
      }
    }
  }
  
  void undo() {
    commandManager.undo();
  }
  
  bool canUndo() => commandManager.canUndo();
  
  // 检测屏幕位置是否在某个点位上
  bool isPointAtScreenPosition(Vector2 screenPosition) {
    final worldPoint = camera.globalToLocal(screenPosition);
    return _findWayPointAtPosition(worldPoint) != null;
  }
  
  // 地图拖拽改由 onScaleStart/Update/End 处理
  
  // 处理缩放开始
  bool onScaleStart(Vector2 position) {
    _baseScale = mapScale;
    _lastFocalPoint = position;
    return true;
  }
  
  // 处理缩放更新
  bool onScaleUpdate(double scale, Vector2 position) {
    if (_lastFocalPoint == null) return false;
    
    // 计算新的缩放值
    final newScale = (_baseScale * scale).clamp(minScale, maxScale);
    
    // 应用缩放
    mapScale = newScale;
    camera.viewfinder.zoom = mapScale;
    
    // 计算焦点偏移
    final focalPointDelta = position - _lastFocalPoint!;
    camera.viewfinder.position -= focalPointDelta / camera.viewfinder.zoom;
    _lastFocalPoint = position;
    return true;
  }
  
  // 处理缩放结束
  bool onScaleEnd() {
    _lastFocalPoint = null;
    return true;
  }
  
  // 处理滚轮缩放
  bool onScroll(double delta, Vector2 position) {
    const zoomSensitivity = 0.5;
    double zoomChange = -delta.sign * zoomSensitivity;
    final newZoom = (camera.viewfinder.zoom + zoomChange).clamp(minScale, maxScale);
    
    // 获取鼠标位置在世界坐标中的位置
    final worldPoint = camera.globalToLocal(position);
    
    // 应用缩放
    camera.viewfinder.zoom = newZoom;
    mapScale = newZoom;
    
    // 调整相机位置以保持鼠标位置不变
    final newScreenPoint = camera.localToGlobal(worldPoint);
    final offset = position - newScreenPoint;
    camera.viewfinder.position -= offset / camera.viewfinder.zoom;
    
    return true;
  }
  
  // 设置离线导航点
  void setOfflineNavPoints(List<NavPoint> navPoints) {
    offLineNavPoints = List<NavPoint>.from(navPoints);
    _updateTopologyLayers();
  }
  
  // 创建导航点
  PoseComponent addWayPoint(NavPoint navPoint) {
    final wayPoint = PoseComponent(
      PoseComponentSize: globalSetting.robotSize,
      color: Colors.green,
      count: 2,
      isEditMode: false,
      direction: navPoint.theta,
      onPoseChanged: (RobotPose pose) {
        // 调用拖拽更新回调
        currentSelectPointUpdate?.call();
      },
      navPoint: navPoint,
      rosChannel: rosChannel,
      poseType: PoseType.waypoint,
    );
    
    wayPoint.priority = 1000; // 确保在最上层
    wayPoint.updatePose(RobotPose(navPoint.x, navPoint.y, navPoint.theta));

    // 添加到WayPoint列表和世界中
    wayPoints.add(wayPoint);
    world.add(wayPoint);
    
    // 新增点位默认选中并显示编辑环
    _selectWayPoint(wayPoint);
    wayPoint.setEditMode(false);
    return wayPoint;
  }
  
  
  // 删除选中的导航点
  String deleteSelectedWayPoint() {
    if (selectedWayPoint != null) {
      var name = selectedWayPoint?.navPoint?.name;
      wayPoints.remove(selectedWayPoint);
      selectedWayPoint?.removeFromParent();
      
      // 通知选择状态变化
      onWayPointSelectionChanged?.call();
      return name!;
    }
    return "";
  }
  
  // 获取导航点数量
  int get wayPointCount => wayPoints.length;
  
  // 查找指定位置的导航点
  PoseComponent? _findWayPointAtPosition(Vector2 position) {
    for (final wayPoint in wayPoints) {
      // 使用containsPoint方法检测点击，与MainFlame的实现保持一致
      if (wayPoint.containsPoint(position)) {
        return wayPoint;
      }
    }
    currentSelectedWayPoint?.setEditMode(false);
    currentSelectedWayPoint = null;
    return null;
  }
  
  // 选中导航点
  void _selectWayPoint(PoseComponent wayPoint) {
    currentSelectedWayPoint = wayPoint;
    // 取消其他导航点的选中状态
    for (final wp in wayPoints) {
      if (wp.navPoint?.name != wayPoint.navPoint?.name) {
        wp.setEditMode(false);
      }
    }
    wayPoint.setEditMode(true);
    
    // 通知外部导航点选择状态变化
    onWayPointSelectionChanged?.call();
    
    // 通知外部当前选中点位动态更新
    currentSelectPointUpdate?.call();
  }
  
  // 修改选中的点位
  void modifySelectedPoint(NavPoint updatedPoint) {
    if (currentSelectedWayPoint == null || currentSelectedWayPoint!.navPoint == null) return;
    
    final oldPoint = currentSelectedWayPoint!.navPoint!;
    
    currentSelectedWayPoint!.navPoint = updatedPoint;
    
    if (rosChannel != null) {
      currentSelectedWayPoint!.updatePose(RobotPose(updatedPoint.x, updatedPoint.y, updatedPoint.theta));
      final mapManager = rosChannel!.mapManager;
      mapManager.updateNavPoint(oldPoint.name, updatedPoint);
      rosChannel!.updateTopologyMap(mapManager.topologyMap.value);
    }
    
    _updateTopologyLayers();
    currentSelectPointUpdate?.call();
  }
  
  // 更新拓扑图层
  void _updateTopologyLayers() {
    // 清除现有的导航点组件
    for (final waypoint in wayPoints) {
      waypoint.removeFromParent();
    }
    wayPoints.clear();
    
    // 确定使用哪个导航点数据源
    List<NavPoint> navPoints = offLineNavPoints;
    List<TopologyRoute> routes = [];
    
    if (rosChannel?.topologyMap_.value != null) {
      final topologyMap = rosChannel!.topologyMap_.value;
      if (topologyMap.points.isNotEmpty) {
        navPoints = List<NavPoint>.from(topologyMap.points);
        routes = topologyMap.routes;
        print('使用在线拓扑点位: ${navPoints.length} 个点, ${routes.length} 条路径');
      } else {
        print('在线拓扑点位为空，使用离线导航点: ${offLineNavPoints.length} 个');
      }
    } else {
      print('拓扑地图数据为空，使用离线导航点: ${offLineNavPoints.length} 个');
    }

    // 更新拓扑线组件数据
    _topologyLineComponent.removeFromParent();
    _topologyLineComponent = TopologyLine(
      points: navPoints,
      routes: routes,
      rosChannel: rosChannel,
    );
    
    // 创建导航点组件并添加到wayPoints
    for (final point in navPoints) {
      final waypoint = PoseComponent(
        PoseComponentSize: globalSetting.robotSize,
        color: Colors.green,
        count: 2,
        isEditMode: false,
        direction: point.theta,
        navPoint: point,
        rosChannel: rosChannel,
        poseType: PoseType.waypoint,
      );

      // 设置路径点位置（使用地图索引坐标）
      waypoint.updatePose(RobotPose(point.x, point.y, point.theta));
      wayPoints.add(waypoint);
    }
    
    // 地图编辑模式下，拓扑地图永远可见
    // 添加拓扑线组件
    if (!world.contains(_topologyLineComponent)) {
      world.add(_topologyLineComponent);
    }
    
    // 添加所有导航点组件到world
    for (final waypoint in wayPoints) {
      if (!world.contains(waypoint)) {
        world.add(waypoint);
      }
    }
    
    print('拓扑图层已更新，显示 ${wayPoints.length} 个导航点');
  }
}



