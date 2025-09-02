import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/display/map.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'package:ros_flutter_gui_app/display/pose.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:vector_math/vector_math_64.dart' as vm;
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';

// 专门的地图编辑Flame组件
class MapEditFlame extends FlameGame {
  late MapComponent _displayMap;
  late GridComponent _displayGrid;
  final RosChannel? rosChannel;
  
  final double minScale = 0.01;
  final double maxScale = 10.0;
  
  // 地图变换参数
  double mapScale = 1.0;
  
  // 当前选中的编辑工具
  String? selectedTool;
  
  // 回调函数，用于通知外部添加导航点
  Future<NavPoint?> Function(double x, double y)? onAddNavPoint;
  
  // 回调函数，用于通知外部导航点选择状态变化
  VoidCallback? onWayPointSelectionChanged;
  
  // 回调函数，用于通知外部当前选中点位动态更新
  VoidCallback? currentSelectPointUpdate;
  
  // 导航点组件列表
  final List<PoseComponent> wayPoints = [];
  
  PoseComponent? currentSelectedWayPoint;
  
  // 手势相关变量
  double _baseScale = 1.0;
  Vector2? _lastFocalPoint;

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
  Future<bool> onTapDown(Vector2 position) async {
    if (selectedTool == 'addNavPoint') {
      // position 为 GestureDetector.localPosition
      final worldPoint = camera.globalToLocal(position);
      final clickedWayPoint = _findWayPointAtPosition(worldPoint);
      
      if (clickedWayPoint != null) {
        print('clickedWayPoint: ${clickedWayPoint.navPoint?.name}');
        // 选中导航点
        _selectWayPoint(clickedWayPoint);
        currentSelectPointUpdate?.call();
        return true;
      }
        double mapX=0;
        double mapY=0;
        if(rosChannel != null && rosChannel!.map_.value != null){
          vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(worldPoint.x, worldPoint.y));
          mapX = mapPose.x;
          mapY = mapPose.y;
        }

        final result = await onAddNavPoint!(mapX, mapY);
        if (result != null) {
          print('导航点添加结果: $result mapX: $mapX mapY: $mapY');
          // 用户确定了名称，创建新的导航点
          addWayPoint(result);
        } else {
          print('用户取消了导航点添加');
        }
      
        return true;  
  
    }
    return false;
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
}
