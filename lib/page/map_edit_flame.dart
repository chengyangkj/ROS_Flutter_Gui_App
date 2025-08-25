import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/display/map.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'package:ros_flutter_gui_app/display/waypoint.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

// 专门的地图编辑Flame组件
class MapEditFlame extends FlameGame with ScrollDetector, ScaleDetector, TapDetector {
  late MapComponent _displayMap;
  late GridComponent _displayGrid;
  final RosChannel? rosChannel;
  
  final double minScale = 0.1;
  final double maxScale = 10.0;
  
  // 地图变换参数
  double mapScale = 1.0;
  Vector2 mapOffset = Vector2.zero();
  
  // 当前选中的编辑工具
  String? selectedTool;
  
  // 回调函数，用于通知外部添加导航点
  Function(double x, double y)? onAddNavPoint;
  
  // 导航点组件列表
  final List<WayPoint> wayPoints = [];
  
  // 双击检测
  DateTime? _lastTapTime;
  Vector2? _lastTapPosition;
  
  // 当前拖拽的组件
  WayPoint? _draggedWayPoint;
  Vector2? _dragStartPosition;
  
  // 当前旋转的组件
  WayPoint? _rotatingWayPoint;
  double? _rotationStartAngle;
  
  // 长按检测
  Timer? _longPressTimer;
  
  MapEditFlame({this.rosChannel, this.onAddNavPoint});
  
  // 设置当前选中的工具
  void setSelectedTool(String? tool) {
    selectedTool = tool;
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
      });
      
      // 立即更新地图数据
      _displayMap.updateMapData(rosChannel!.map_.value);
    }
  }
  
  // 处理单击事件，实现双击检测
  @override
  bool onTapDown(TapDownInfo info) {
    if (selectedTool == 'addNavPoint') {
      final worldPoint = camera.globalToLocal(info.eventPosition.global);
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
  
  // 简化的拖拽和旋转处理（通过onTapDown实现）
  // 在实际使用中，可以通过手势检测器来实现更复杂的拖拽
  
  // 创建导航点
  void _createWayPoint(double x, double y) {
    final wayPoint = WayPoint(
      waypointSize: 5.0,
      color: Colors.blue,
      count: 2,
      isEditMode: true, // 在编辑模式下显示小红点
      directionAngle: 0.0, // 初始方向角度
    );
    wayPoint.position = Vector2(x, y);
    wayPoint.priority = 1000; // 确保在最上层
    
    wayPoints.add(wayPoint);
    world.add(wayPoint);
  }
  
  // 删除选中的导航点
  void deleteSelectedWayPoint() {
    final selectedWayPoint = wayPoints.where((wp) => wp.isSelected).firstOrNull;
    if (selectedWayPoint != null) {
      wayPoints.remove(selectedWayPoint);
      selectedWayPoint.removeFromParent();
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
  }
  
  // 检查是否点击了旋转控制点
  bool _isOnRotationControl(WayPoint wayPoint, Vector2 position) {
    // 计算圆环上的控制点位置
    final radius = wayPoint.waypointSize / 2 + 2;
    final controlPoint = wayPoint.position + Vector2(radius, 0);
    final distance = (controlPoint - position).length;
    return distance <= 5; // 控制点半径
  }
  
  // 计算角度
  double _calculateAngle(Vector2 center, Vector2 point) {
    return (point - center).angleToSigned(Vector2(1, 0));
  }
  
  // 手势和滚轮缩放支持
  double _baseScale = 1.0;
  Vector2? _lastFocalPoint;
  
  @override
  bool onScroll(PointerScrollInfo info) {
    // 滚轮缩放
    const zoomSensitivity = 0.5;
    double zoomChange = -info.scrollDelta.global.y.sign * zoomSensitivity;
    final newZoom = (camera.viewfinder.zoom + zoomChange).clamp(minScale, maxScale);
    
    // 获取鼠标位置在世界坐标中的位置
    final mousePosition = info.eventPosition.global;
    final worldPoint = camera.globalToLocal(mousePosition);
    
    // 应用缩放
    camera.viewfinder.zoom = newZoom;
    mapScale = newZoom;
    
    // 调整相机位置以保持鼠标位置不变
    final newScreenPoint = camera.localToGlobal(worldPoint);
    final offset = mousePosition - newScreenPoint;
    camera.viewfinder.position -= offset / camera.viewfinder.zoom;
    
    return true;
  }
  
  @override
  bool onScaleStart(ScaleStartInfo info) {
    _baseScale = mapScale;
    _lastFocalPoint = info.eventPosition.global;
    return true;
  }
  
  @override
  bool onScaleUpdate(ScaleUpdateInfo info) {
    if (_lastFocalPoint == null) return false;
    
    // 计算新的缩放值
    final newScale = (_baseScale * info.scale.global.x).clamp(minScale, maxScale);
    
    // 应用缩放
    mapScale = newScale;
    camera.viewfinder.zoom = mapScale;
    
    // 计算焦点偏移
    final focalPointDelta = info.eventPosition.global - _lastFocalPoint!;
    camera.viewfinder.position -= focalPointDelta / camera.viewfinder.zoom;
    
    _lastFocalPoint = info.eventPosition.global;
    return true;
  }
  
  @override
  bool onScaleEnd(ScaleEndInfo info) {
    _lastFocalPoint = null;
    return true;
  }
  
  // 缩放控制方法
  void zoomIn() {
    mapScale = (mapScale * 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  void zoomOut() {
    mapScale = (mapScale / 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  void resetZoom() {
    mapScale = 1.0;
    camera.viewfinder.zoom = mapScale;
    camera.viewfinder.position = Vector2.zero();
  }
}
