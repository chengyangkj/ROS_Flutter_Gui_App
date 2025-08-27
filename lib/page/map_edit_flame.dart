import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/display/map.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'package:ros_flutter_gui_app/display/waypoint.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

// 专门的地图编辑Flame组件
class MapEditFlame extends FlameGame {
  late MapComponent _displayMap;
  late GridComponent _displayGrid;
  final RosChannel? rosChannel;
  
  final double minScale = 0.1;
  final double maxScale = 10.0;
  
  // 地图变换参数
  double mapScale = 1.0;
  
  // 当前选中的编辑工具
  String? selectedTool;
  
  // 回调函数，用于通知外部添加导航点
  Function(double x, double y)? onAddNavPoint;
  
  // 回调函数，用于通知外部导航点选择状态变化
  VoidCallback? onWayPointSelectionChanged;
  
  // 回调函数，用于通知外部当前选中点位动态更新
  VoidCallback? currentSelectPointUpdate;
  
  // 导航点组件列表
  final List<WayPoint> wayPoints = [];
  
  // 双击检测
  DateTime? _lastTapTime;
  Vector2? _lastTapPosition;
  
  // 手势相关变量
  double _baseScale = 1.0;
  Vector2? _lastFocalPoint;
  bool _isDragging = false;
  Vector2? _dragStartPosition;

  OccupancyMap? _occupancyMap;
  
  MapEditFlame({
    this.rosChannel, 
    this.onAddNavPoint, 
    this.onWayPointSelectionChanged,
    this.currentSelectPointUpdate,
  });
  
  // 设置当前选中的工具
  void setSelectedTool(String? tool) {
    selectedTool = tool;
  }
  
  // 获取当前选中的导航点
  WayPoint? get selectedWayPoint {
    return wayPoints.where((wp) => wp.isSelected).firstOrNull;
  }
  
  // 获取当前选中导航点的信息
  Map<String, dynamic>? getSelectedWayPointInfo() {
    final wayPoint = selectedWayPoint;
    if (wayPoint == null) return null;
    var x = wayPoint.position.x;
    var y = wayPoint.position.y;
    var direction = -wayPoint.directionAngle;
    double mapx = 0;
    double mapy = 0;
    if(_occupancyMap != null){
      vm.Vector2 mapPose = _occupancyMap!.idx2xy(vm.Vector2(x, y));
      mapx = mapPose.x;
      mapy = mapPose.y;
    }


    return {
      'x': mapx.toStringAsFixed(2),
      'y': mapy.toStringAsFixed(2),
      'direction': direction.toStringAsFixed(1),
      'name': '导航点 ${wayPoints.indexOf(wayPoint) + 1}',
    };
  }
  
  // 获取所有导航点的信息
  List<Map<String, dynamic>> getAllWayPoint() {
    List<Map<String, dynamic>> allWayPoints = [];
    
    for (int i = 0; i < wayPoints.length; i++) {
      final wayPoint = wayPoints[i];
      var x = wayPoint.position.x;
      var y = wayPoint.position.y;
      var direction = -wayPoint.directionAngle;
      double mapx = 0;
      double mapy = 0;
      
      if (_occupancyMap != null) {
        vm.Vector2 mapPose = _occupancyMap!.idx2xy(vm.Vector2(x, y));
        mapx = mapPose.x;
        mapy = mapPose.y;
      }
      
      allWayPoints.add({
        'x': mapx.toStringAsFixed(2),
        'y': mapy.toStringAsFixed(2),
        'direction': direction.toStringAsFixed(1),
        'name': '导航点 ${i + 1}',
        'isSelected': wayPoint.isSelected,
        'index': i,
      });
    }
    
    return allWayPoints;
  }
  
  @override
  Future<void> onLoad() async {
    super.onLoad();
    
    // 添加地图组件
    _displayMap = MapComponent(rosChannel: rosChannel);
    world.add(_displayMap);
    _displayMap.priority = 900;
    
    // 添加网格组件
    _displayGrid = GridComponent(
      size: size,
      rosChannel: rosChannel,
    );
    _displayGrid.priority = 910;
    
    // 设置ROS监听器
    _setupRosListeners();
  }
  
  void _setupRosListeners() {
    if (rosChannel != null) {
      // 监听地图数据
      rosChannel!.map_.addListener(() {
        _displayMap.updateMapData(rosChannel!.map_.value);
        _occupancyMap=rosChannel!.map_.value;
      });
      
      // 立即更新地图数据
      _displayMap.updateMapData(rosChannel!.map_.value);
      _occupancyMap=rosChannel!.map_.value;
    }
  }
  
  // 处理单击事件，实现双击检测
  bool onTapDown(Vector2 position) {
    if (selectedTool == 'addNavPoint') {
      final worldPoint = camera.globalToLocal(position);
      final clickedWayPoint = _findWayPointAtPosition(worldPoint);
      
      if (clickedWayPoint != null) {
        // 选中导航点
        _selectWayPoint(clickedWayPoint);
        return true;
      }
      
      // 双击检测
      final now = DateTime.now();
      if (_lastTapTime != null && 
          _lastTapPosition != null &&
          now.difference(_lastTapTime!).inMilliseconds < 300 && // 300ms内
          (_lastTapPosition! - worldPoint).length < 20) { // 20像素内
        
        // 双击：创建新的导航点
        _createWayPoint(worldPoint.x, worldPoint.y);
        
        // 调用回调函数添加导航点
        if (onAddNavPoint != null) {
          onAddNavPoint!(worldPoint.x, worldPoint.y);
        }
        
        // 重置双击检测
        _lastTapTime = null;
        _lastTapPosition = null;
        return true;
      }
      
      // 记录单击信息
      _lastTapTime = now;
      _lastTapPosition = worldPoint;
    }
    return false;
  }
  
  // 处理拖拽开始
  bool onDragStart(Vector2 position) {
    _isDragging = true;
    _dragStartPosition = position;
    _lastFocalPoint = position;
    return true;
  }
  
  // 处理拖拽更新
  bool onDragUpdate(Vector2 position) {
    if (!_isDragging || _lastFocalPoint == null) return false;
    
    final delta = position - _lastFocalPoint!;
    camera.viewfinder.position -= delta / camera.viewfinder.zoom;
    _lastFocalPoint = position;
    return true;
  }
  
  // 处理拖拽结束
  bool onDragEnd() {
    _isDragging = false;
    _dragStartPosition = null;
    _lastFocalPoint = null;
    return true;
  }
  
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
  
  // 创建导航点
  WayPoint _createWayPoint(double x, double y) {
    final wayPoint = WayPoint(
      waypointSize:globalSetting.robotSize,
      color: const Color(0xFF0080ff),
      count: 2,
      isEditMode: true,
      directionAngle: 0.0,
      onDragUpdate: () {
        // 调用拖拽更新回调
        currentSelectPointUpdate?.call();
      },
    );
    
    wayPoint.position = Vector2(x, y);
    wayPoint.priority = 1000; // 确保在最上层
    
    // 添加到WayPoint列表和世界中
    wayPoints.add(wayPoint);
    world.add(wayPoint);
    
    return wayPoint;
  }
  
  // 添加导航点
  void _addNavPoint(double x, double y) {
    final wayPoint = _createWayPoint(x, y);
    
    // 选中新创建的WayPoint
    _selectWayPoint(wayPoint);
    
    // 通知外部添加了新的导航点
    onAddNavPoint?.call(x, y);
  }
  
  // 删除选中的导航点
  void deleteSelectedWayPoint() {
    final selectedWayPoint = wayPoints.where((wp) => wp.isSelected).firstOrNull;
    if (selectedWayPoint != null) {
      wayPoints.remove(selectedWayPoint);
      selectedWayPoint.removeFromParent();
      
      // 通知选择状态变化
      onWayPointSelectionChanged?.call();
    }
  }
  
  // 获取导航点数量
  int get wayPointCount => wayPoints.length;
  
  // 查找指定位置的导航点
  WayPoint? _findWayPointAtPosition(Vector2 position) {
    for (final wayPoint in wayPoints) {
      final distance = (wayPoint.position - position).length;
      if (distance <= wayPoint.waypointSize / 2) {
        return wayPoint;
      }
    }
    return null;
  }
  
  // 选中导航点
  void _selectWayPoint(WayPoint wayPoint) {
    // 取消其他导航点的选中状态
    for (final wp in wayPoints) {
      wp.isSelected = false;
    }
    wayPoint.isSelected = true;
    
    // 通知外部导航点选择状态变化
    onWayPointSelectionChanged?.call();
    
    // 通知外部当前选中点位动态更新
    currentSelectPointUpdate?.call();
  }
}
