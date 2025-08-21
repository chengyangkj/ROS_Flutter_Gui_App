import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:flame_svg/flame_svg.dart';
import 'package:flame_svg/svg_component.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/display/map.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'dart:math';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';

class MainFlame extends FlameGame
    with ScrollDetector, ScaleDetector {
  late DisplayMap _displayMap;
  late DisplayGrid _displayGrid;
  final RosChannel? rosChannel;
  late SvgComponent _displayRobot;
  final double minScale = 0.1;
  final double maxScale = 10.0;
  final robotSize = 10.0;

  // 地图变换参数
  double mapScale = 1.0;
  Vector2 mapOffset = Vector2.zero();
  
  MainFlame({this.rosChannel});
  
  @override
  Future<void> onLoad() async {
    super.onLoad();
    
     _displayGrid = DisplayGrid(
      size: size,
      rosChannel: rosChannel,
    );
    world.add(_displayGrid);

    // 添加地图组件，传递RosChannel
    _displayMap = DisplayMap(rosChannel: rosChannel);
    world.add(_displayMap);
    
    // 添加机器人组件

    var svg = await loadSvg('icons/robot/robot2.svg');
    _displayRobot= SvgComponent(
      svg: svg,
      size: Vector2.all(robotSize),
      anchor: Anchor.center,
    );
    world.add(_displayRobot);

    rosChannel!.robotPoseScene.addListener(() {
      _displayRobot.position = Vector2(
        rosChannel!.robotPoseScene.value.x,
        rosChannel!.robotPoseScene.value.y,
      );
      _displayRobot.angle=-deg2rad(90)- rosChannel!.robotPoseScene.value.theta;
    });
    
  }

  
  // 手势和滚轮缩放支持 - 使用Flame内置事件
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
    final scale = info.scale.global.x; // 使用x分量作为统一缩放值
    
    if (scale != 1.0) {
      // 处理缩放
      final newScale = (_baseScale * scale).clamp(minScale, maxScale);
      
      if (_lastFocalPoint != null) {
        // 获取焦点在世界坐标中的位置（缩放前）
        final focalPoint = info.eventPosition.global;
        final worldPoint = camera.globalToLocal(focalPoint);
        
        // 应用新的缩放
        camera.viewfinder.zoom = newScale;
        mapScale = newScale;
        
        // 重新计算焦点在屏幕上的位置（缩放后）
        final newScreenPoint = camera.localToGlobal(worldPoint);
        
        // 调整相机位置以保持焦点位置不变
        final offset = focalPoint - newScreenPoint;
        camera.viewfinder.position -= offset / camera.viewfinder.zoom;
      } else {
        camera.viewfinder.zoom = newScale;
        mapScale = newScale;
      }
    } else {
      // 处理平移
      if (_lastFocalPoint != null) {
        final currentFocalPoint = info.eventPosition.global;
        final delta = currentFocalPoint - _lastFocalPoint!;
        camera.viewfinder.position -= delta / camera.viewfinder.zoom;
        _lastFocalPoint = currentFocalPoint;
      }
    }
    return true;
  }
  
  @override
  bool onScaleEnd(ScaleEndInfo info) {
    _lastFocalPoint = null;
    return true;
  }

  
  void centerOnRobot() {
    // 将相机重置到默认位置和缩放
    camera.viewfinder.position = Vector2.zero();
    camera.viewfinder.zoom = 1.0;
    mapScale = 1.0;
  }
  
  void zoomIn() {
    mapScale = (mapScale * 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  void zoomOut() {
    mapScale = (mapScale / 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  // 简化实现，只保留基本的点击效果

  @override
  void update(double dt) {
    super.update(dt);
    // 这里可以添加游戏逻辑更新
  }
}