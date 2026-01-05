import 'package:flame/game.dart';
import 'package:flame/components.dart';
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
  Future<NavPoint?> Function(double x, double y)? onAddNavPoint;
  
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
  List<MapEntry<int, int>> _currentEditCells = [];
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
  
  // 使用当前机器人位置添加导航点
  Future<void> addNavPointAtRobotPosition() async {
    if (selectedTool != EditToolType.addNavPoint) return;
    
    // 使用当前机器人位置添加导航点
    final result = await onAddNavPoint!(currentRobotPose.x, currentRobotPose.y);
    if (result != null) {
      print('使用机器人位置添加导航点: $result x: ${currentRobotPose.x} y: ${currentRobotPose.y}');
      // 创建新的导航点，使用机器人当前朝向
      final navPointWithRobotPose = NavPoint(
        name: result.name,
        x: currentRobotPose.x,
        y: currentRobotPose.y,
        theta: currentRobotPose.theta,
        type: result.type,
      );
      addWayPoint(navPointWithRobotPose);
    } else {
      print('用户取消了使用机器人位置添加导航点');
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
    if(rosChannel != null && rosChannel!.map_.value != null){
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
  
  // 获取所有导航点的信息
  List<NavPoint> getAllWayPoint() {
    List<NavPoint> allWayPoints = [];
    
    for (int i = 0; i < wayPoints.length; i++) {
      final wayPoint = wayPoints[i];
      var x = wayPoint.position.x;
      var y = wayPoint.position.y;
      var direction = -wayPoint.direction;
      double mapx = 0;
      double mapy = 0;
      
      if (rosChannel != null && rosChannel!.map_.value != null) {
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
      if (rosChannel != null && rosChannel!.map_.value != null) {
        vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(worldPoint.x, worldPoint.y));
        mapX = mapPose.x;
        mapY = mapPose.y;
      }

      final result = await onAddNavPoint!(mapX, mapY);
      if (result != null) {
        addWayPoint(result);
      }
      return true;
    } else if (selectedTool == EditToolType.addRoute) {
      // 拓扑连线工具：点击点位进行连线
      final clickedWayPoint = _findWayPointAtPosition(worldPoint);
      if (clickedWayPoint != null && clickedWayPoint.navPoint != null) {
        if (_routeStartPoint == null) {
          _routeStartPoint = clickedWayPoint.navPoint!.name;
          _selectWayPoint(clickedWayPoint);
          currentSelectPointUpdate?.call();
        } else if (_routeStartPoint != clickedWayPoint.navPoint!.name) {
          // 创建路线
          _createRoute(_routeStartPoint!, clickedWayPoint.navPoint!.name);
          _routeStartPoint = null;
        }
        return true;
      } else {
        // 点击空白区域，取消连线
        _routeStartPoint = null;
        return false;
      }
    } else if (selectedTool == EditToolType.drawLine) {
      // 直线绘制工具：点击设置起始点和结束点
      double mapX = 0;
      double mapY = 0;
      if (rosChannel != null && rosChannel!.map_.value != null) {
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
      _currentEditCells.clear();
      _lastObstaclePosition = worldPoint;
      _drawObstacleAtPosition(worldPoint);
      return true;
    } else if (selectedTool == EditToolType.eraseObstacle) {
      _operationState = EditOperationState.erasingObstacle;
      _currentEditCells.clear();
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
          // 移动点位
          double mapX = 0;
          double mapY = 0;
          if (rosChannel != null && rosChannel!.map_.value != null) {
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
          // 旋转点位方向
          double mapX = 0;
          double mapY = 0;
          if (rosChannel != null && rosChannel!.map_.value != null) {
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
        if (_lastObstaclePosition != null) {
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
            
            if (_operationState == EditOperationState.drawingObstacle) {
              final cells = _getCellsForPosition(interpolatedPoint);
              allCells.addAll(cells);
            } else {
              final cells = _getCellsForPosition(interpolatedPoint);
              allCells.addAll(cells);
            }
          }
          
          // 去重并批量更新
          final uniqueCells = <MapEntry<int, int>>{};
          for (var cell in allCells) {
            uniqueCells.add(cell);
          }
          
          final value = _operationState == EditOperationState.drawingObstacle ? 100 : 0;
          if (uniqueCells.isNotEmpty) {
            _displayMap.modifyCells(uniqueCells.toList(), value);
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
        // 完成点位移动/旋转操作，创建命令
        if (_draggedPoint != null && _dragStartPointData != null && _draggedPoint!.navPoint != null) {
          final currentPoint = _draggedPoint!.navPoint!;
          if (currentPoint.name == _dragStartPointData!.name) {
            // 创建修改点位命令
            // 注意：这里需要实现 ModifyPointCommand
            // 暂时直接更新，后续可以添加到命令系统
          }
        }
        _operationState = EditOperationState.none;
        _draggedPoint = null;
        _dragStartPos = null;
        _dragStartPointData = null;
        return true;
        
      case EditOperationState.drawingObstacle:
      case EditOperationState.erasingObstacle:
        if (rosChannel != null && rosChannel!.map_.value != null) {
          if (_currentEditCells.isNotEmpty) {
            final map = rosChannel!.map_.value!;
            if (_operationState == EditOperationState.drawingObstacle) {
              final command = DrawObstacleCommand(
                map,
                List.from(_currentEditCells),
              );
              commandManager.executeCommand(command);
            } else {
              final command = EraseObstacleCommand(
                map,
                List.from(_currentEditCells),
              );
              commandManager.executeCommand(command);
            }
            _currentEditCells.clear();
            _displayMap.updateMapData(map);
          }
        }
        _operationState = EditOperationState.none;
        _lastObstaclePosition = null;
        return true;
        
      case EditOperationState.none:
        break;
    }
    return false;
  }
  
  // 创建路线
  void _createRoute(String fromPoint, String toPoint) {
    if (rosChannel == null || rosChannel!.topologyMap_.value == null) return;
    
    final topologyMap = rosChannel!.topologyMap_.value!;
    
    // 检查路线是否已存在
    final exists = topologyMap.routes.any(
      (r) => r.fromPoint == fromPoint && r.toPoint == toPoint
    );
    
    if (exists) {
      print('路线已存在: $fromPoint -> $toPoint');
      return;
    }
    
    // 创建新路线
    final newRoute = TopologyRoute(
      fromPoint: fromPoint,
      toPoint: toPoint,
      routeInfo: RouteInfo(
        controller: 'FollowPath',
        goalChecker: 'general_goal_checker',
        speedLimit: 1.0,
      ),
    );
    
    final command = AddRouteCommand(topologyMap, newRoute, () {
      _updateTopologyLayers();
      if (rosChannel != null) {
        rosChannel!.updateTopologyMap(topologyMap);
      }
    });
    
    commandManager.executeCommand(command);
    print('已创建路线: $fromPoint -> $toPoint');
  }
  
  // 绘制直线
  void _drawLine(Vector2 start, Vector2 end) {
    if (rosChannel == null || rosChannel!.map_.value == null) return;
    
    final map = rosChannel!.map_.value!;
    final startIdx = map.xy2idx(vm.Vector2(start.x, start.y));
    final endIdx = map.xy2idx(vm.Vector2(end.x, end.y));
    
    final dx = endIdx.x - startIdx.x;
    final dy = endIdx.y - startIdx.y;
    final steps = math.max(dx.abs().round(), dy.abs().round());
    
    _currentEditCells.clear();
    
    for (int i = 0; i <= steps; i++) {
      final t = i / steps;
      final x = startIdx.x + dx * t;
      final y = startIdx.y + dy * t;
      final row = y.round();
      final col = x.round();
      
      if (row >= 0 && row < map.Rows() && col >= 0 && col < map.Cols()) {
        final cellKey = MapEntry(row, col);
        bool exists = false;
        for (var existing in _currentEditCells) {
          if (existing.key == row && existing.value == col) {
            exists = true;
            break;
          }
        }
        if (!exists) {
          _currentEditCells.add(cellKey);
        }
      }
    }
    
    // 应用绘制
    for (var cell in _currentEditCells) {
      if (cell.key >= 0 && cell.key < map.Rows() && 
          cell.value >= 0 && cell.value < map.Cols()) {
        map.data[cell.key][cell.value] = 100;
      }
    }
    
    // 创建命令
    if (_currentEditCells.isNotEmpty) {
      final command = DrawObstacleCommand(map, List.from(_currentEditCells));
      commandManager.executeCommand(command);
      _currentEditCells.clear();
      _displayMap.updateMapData(map);
    }
  }
  
  // 获取指定位置的所有单元格（用于批量处理）
  List<MapEntry<int, int>> _getCellsForPosition(Vector2 worldPoint) {
    if (rosChannel == null || rosChannel!.map_.value == null) return [];
    
    final map = rosChannel!.map_.value!;
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
      _displayMap.modifyCells(cells, OBSTACLE_COLOR);
    }
  }
  
  // 在指定位置擦除障碍物
  void _eraseObstacleAtPosition(Vector2 worldPoint) {
    final cells = _getCellsForPosition(worldPoint);
    if (cells.isNotEmpty) {
      _displayMap.modifyCells(cells, FREE_COLOR);
    }
  }
  
  // 撤销操作
  void undo() {
    commandManager.undo();
    if (rosChannel != null && rosChannel!.map_.value != null) {
      _displayMap.updateMapData(rosChannel!.map_.value);
    }
  }
  
  // 重做操作
  void redo() {
    commandManager.redo();
    if (rosChannel != null && rosChannel!.map_.value != null) {
      _displayMap.updateMapData(rosChannel!.map_.value);
    }
  }
  
  // 检查是否可以撤销
  bool canUndo() => commandManager.canUndo();
  
  // 检查是否可以重做
  bool canRedo() => commandManager.canRedo();
  
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
    
    // 更新点位数据
    currentSelectedWayPoint!.navPoint = updatedPoint;
    
    // 更新位置
    if (rosChannel != null && rosChannel!.map_.value != null) {
      final map = rosChannel!.map_.value!;
      final idx = map.xy2idx(vm.Vector2(updatedPoint.x, updatedPoint.y));
      currentSelectedWayPoint!.updatePose(RobotPose(updatedPoint.x, updatedPoint.y, updatedPoint.theta));
    }
    
    // 更新拓扑地图
    if (rosChannel != null && rosChannel!.topologyMap_.value != null) {
      final topologyMap = rosChannel!.topologyMap_.value!;
      final index = topologyMap.points.indexWhere((p) => p.name == oldPoint.name);
      if (index != -1) {
        topologyMap.points[index] = updatedPoint;
        rosChannel!.updateTopologyMap(topologyMap);
      }
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

