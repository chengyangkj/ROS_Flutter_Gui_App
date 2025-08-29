import 'dart:math';
import 'dart:ui';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

enum PoseType{
  robot,
  waypoint,
}

class PoseComponent extends PositionComponent with HasGameRef {
  late double PoseComponentSize;
  late Color color;
  int count;
  late Timer animationTimer;
  double animationValue = 0.0;

  // 添加编辑模式控制
  bool isEditMode = false;
  
  // 添加方向角度（弧度）
  double direction = 0.0;
  
  // 添加姿态变化回调
  Function(RobotPose)? onPoseChanged;

  // 存储导航点信息
  NavPoint? navPoint;
  
  // 存储占用地图信息
  OccupancyMap? occMap;
  
  // 添加RosChannel引用
  RosChannel? rosChannel;
  PoseType poseType;
  
  // 组件内处理手势，无需对外状态
  PoseComponent({
    required this.PoseComponentSize,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
    this.isEditMode = false,
    this.direction = 0.0,
    this.onPoseChanged,
    this.navPoint,
    this.occMap,
    this.rosChannel,
    this.poseType = PoseType.waypoint,
  });
  
  @override
  Future<void> onLoad() async {
    size = Vector2.all(PoseComponentSize);
    anchor = Anchor.center;
    animationTimer = Timer(
      2.0,
      onTick: () {
        // 重置动画值
        animationValue = 0.0;
      },
      repeat: true,
    );
    
    // 添加渲染器
    add(PoseComponentRenderer(
      size: PoseComponentSize,
      color: color,
      count: count,
      animationValue: animationValue,
      poseType: poseType,
    ));
    
    // 根据编辑模式与选中状态控制方向环
    _updateDirectionControlVisibility();
  }

  @override
  void update(double dt) {
    animationTimer.update(dt);
    // 直接使用progress，确保动画值在0到1之间
    animationValue = animationTimer.progress;
    
    // 同步更新渲染器的动画值
    final renderer = children.whereType<PoseComponentRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updateAnimationValue(animationValue);
    }

    
    super.update(dt);
  }
  
  int frameCount = 0;

  @override
  void onRemove() {
    super.onRemove();
  }

  void updatePose(RobotPose pose) {
    // 优先使用传入的occMap，如果没有则从RosChannel获取
    OccupancyMap? currentMap = occMap;
    if (currentMap == null && rosChannel != null) {
      currentMap = rosChannel!.map_.value;
    }
    
    if (currentMap != null && currentMap.mapConfig.resolution > 0) {
      var occPose = currentMap.xy2idx(vm.Vector2(pose.x, pose.y));
      position = Vector2(occPose.x, occPose.y);
    } else {
      // 如果没有地图数据，直接使用原始坐标
      position = Vector2(pose.x, pose.y);
    }
    direction = -pose.theta;
  }

  // 更新方向角度
  void updatedirection(double newAngle) {
    direction = newAngle;
    
    // 同步更新DirectionControl的角度
    final directionControl = children.whereType<DirectionControl>().firstOrNull;
    if (directionControl != null) {
      directionControl.updateAngle(direction);
    }
  
    // 触发姿态变化回调
    _triggerPoseChangedCallback();
  }
  
  // 静默设置角度，避免回调递归
  void _setAngleSilent(double newAngle) {
    direction = newAngle;
    final renderer = children.whereType<DirectionControlRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updateAngle(direction);
    }
  }
  
  // 对外暴露：直接设置角度（静默）
  void setAngleDirect(double newAngle) {
    _setAngleSilent(newAngle);
  }
  
  NavPoint? getPointInfo() {
    return navPoint;
  }

  
  // 设置编辑模式（地图编辑工具控制）
  void setEditMode(bool edit) {
    if (isEditMode == edit) return;
    isEditMode = edit;
    _updateDirectionControlVisibility();
  }
  
  void _updateDirectionControlVisibility() {
    final exists = children.whereType<DirectionControl>().firstOrNull;
    final shouldShow = isEditMode;
    if (shouldShow) {
      if (exists == null) {
        final directionControl = DirectionControl(
          controlSize: PoseComponentSize,
          onDirectionChanged: updatedirection,
          initAngle: direction,
        );
        add(directionControl);
      }
    } else {
      if (exists != null) {
        // 在移除前先重置手势状态
        exists._resetDragState();
        exists.removeFromParent();
      }
    }
  }
  
  // 设置导航点信息
  void setNavPoint(NavPoint point) {
    navPoint = point;
  }
  
  // 设置占用地图信息
  void setOccMap(OccupancyMap map) {
    occMap = map;
  }
  
  // 获取占用地图信息
  OccupancyMap? getOccMap() {
    // 优先返回传入的occMap，如果没有则从RosChannel获取
    if (occMap != null) {
      return occMap;
    }
    if (rosChannel != null) {
      return rosChannel!.map_.value;
    }
    return null;
  }
  
  // 触发姿态变化回调
  void _triggerPoseChangedCallback() {
    if (onPoseChanged != null) {
      // 创建RobotPose对象，包含当前位置和方向
      var occMap = getOccMap();
      double mapPosex=0;
      double mapPosey=0;
      if(occMap != null){
        var p = occMap.idx2xy(vm.Vector2(position.x, position.y));
        mapPosex=p.x;
        mapPosey=p.y;
      }
      final robotPose = RobotPose(
        mapPosex,  // x坐标
        mapPosey,  // y坐标
        -direction,   // 方向角度
      );
      onPoseChanged!(robotPose);
    }
  }
  
  // 设置RosChannel引用
  void setRosChannel(RosChannel channel) {
    rosChannel = channel;
  }
  
}

// 方向控制组件
class DirectionControl extends PositionComponent with DragCallbacks {
  final double controlSize;
  final Function(double) onDirectionChanged;
  final double initAngle;
  
  double _currentAngle = 0.0;
  bool _isUpdating = false; // 防止重复调用
  Vector2? _lastMousePosition;
  bool _isRotating = false;
  bool _isPositionDragging = false;
  Vector2? _positionDragStart;
  
  // 手势区域检测在方法中动态计算

  DirectionControl({
    required this.controlSize,
    required this.onDirectionChanged,
    required this.initAngle,
  });

  @override
  Future<void> onLoad() async {
    size = Vector2.all(controlSize);
    _currentAngle = initAngle;

    
    add(DirectionControlRenderer(
      size: controlSize,
      angle: _currentAngle,
    ));
  }
  
  @override
  void onRemove() {
    super.onRemove();
    // 确保在移除时重置所有手势状态
    _resetDragState();
  }
 
  
  @override
  bool onDragStart(DragStartEvent event) {
    super.onDragStart(event);
    
    // 检查父组件是否处于编辑模式
    if (parent is PoseComponent) {
      final poseComponent = parent as PoseComponent;
      if (!poseComponent.isEditMode) {
        return false; // 不在编辑模式时不拦截手势
      }
    }
    
    _lastMousePosition = event.localPosition;
    final center = size / 2;
    final distance = (event.localPosition - center).length;
    final ringRadius = controlSize / 2;
    if (distance <= ringRadius / 2) {
      // 中心区域：位置拖拽
      _isRotating = false;
      _isPositionDragging = true;
      _positionDragStart = event.localPosition;
      return true;
    } else if (distance <= ringRadius + 2) {
      // 外圈：旋转
      _isRotating = true;
      _isPositionDragging = false;
      return true;
    }
    return false;
  }

  @override
  bool onDragUpdate(DragUpdateEvent event) {
    super.onDragUpdate(event);
    
    // 检查父组件是否处于编辑模式
    if (parent is PoseComponent) {
      final poseComponent = parent as PoseComponent;
      if (!poseComponent.isEditMode) {
        return false; // 不在编辑模式时不拦截手势
      }
    }
    
    if (_isRotating) {
      if (_lastMousePosition != null) {
        _lastMousePosition = _lastMousePosition! + event.localDelta;
        final center = size / 2;
        final dx = _lastMousePosition!.x - center.x;
        final dy = _lastMousePosition!.y - center.y;
        final newAngle = atan2(dy, dx);
        _currentAngle = newAngle;
        onDirectionChanged(_currentAngle);
        final renderer = children.whereType<DirectionControlRenderer>().firstOrNull;
        if (renderer != null) {
          renderer.updateAngle(_currentAngle);
        }
      }
      return true;
    } else if (_isPositionDragging) {
      if (parent is PoseComponent && _positionDragStart != null) {
        final poseComponent = parent as PoseComponent;
        final delta = event.localDelta;
        poseComponent.position = poseComponent.position + delta;
        // 触发姿态变化回调
        poseComponent._triggerPoseChangedCallback();
      }
      return true;
    }
    return false;
  }

  @override
  bool onDragEnd(DragEndEvent event) {
    super.onDragEnd(event);
    
    // 检查父组件是否处于编辑模式
    if (parent is PoseComponent) {
      final poseComponent = parent as PoseComponent;
      if (!poseComponent.isEditMode) {
        return false; // 不在编辑模式时不拦截手势
      }
    }
    
    _resetDragState();
    return true;
  }

  // 重置拖拽状态
  void _resetDragState() {
    _lastMousePosition = null;
    _isRotating = false;
    _isPositionDragging = false;
    _positionDragStart = null;
  }
  
  // 更新角度（由外部手势处理调用）
  void updateAngle(double newAngle) {
    
    if (_isUpdating) {
      return; // 防止重复调用
    }
    
    _isUpdating = true;
    _currentAngle = newAngle;
    
    // 通知父组件更新方向角度
    onDirectionChanged(_currentAngle);
    
    // 更新渲染器
    final renderer = children.whereType<DirectionControlRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updateAngle(_currentAngle);
    }
    
    _isUpdating = false;
  }
}

// 方向控制渲染器
class DirectionControlRenderer extends Component {
  final double size;
  double angle;
  
  DirectionControlRenderer({
    required this.size,
    required this.angle,
  });
  
  void updateAngle(double newAngle) {
    angle = newAngle;
  }
  
  @override
  void render(Canvas canvas) {
    // 计算绘制起始点，使圆环以点位为中心
    final center = Offset(size / 2, size / 2);
    final radius = size / 2; // 调整半径，确保圆环能包裹点位
    canvas.save();
    // canvas.translate(-size / 2, -size / 2);
    // 绘制主圆圈
    final circlePaint = Paint()
      ..color = Colors.blue.withOpacity(0.6)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 0.5;
    
    canvas.drawCircle(center, radius, circlePaint);
    
    // 绘制方向指示点
    final pointPaint = Paint()
      ..color = Colors.red
      ..style = PaintingStyle.fill;
    
    final pointOffset = Offset(
      center.dx + radius * cos(angle),
      center.dy + radius * sin(angle),
    );
    
    canvas.drawCircle(pointOffset, 0.6, pointPaint);
    canvas.restore();
  }
}

// 路径点渲染器
class PoseComponentRenderer extends Component with HasGameRef {
  final double size;
  final Color color;
  final int count;
  double animationValue; // 移除final，允许更新
  final PoseType poseType;

  PoseComponentRenderer({
    required this.size,
    required this.color,
    required this.count,
    required this.animationValue,
    required this.poseType,
  });
  
  // 添加更新动画值的方法
  void updateAnimationValue(double value) {
    animationValue = value;
  }

  @override
  void render(Canvas canvas) {
    // 添加安全检查
    if (!isMounted) {
      return;
    }
    
    try {
      canvas.save();
      
      // 获取父组件的方向角度
      final parentComponent = parent;
      if (parentComponent is PoseComponent) {
        final double rotationAngle = parentComponent.direction;
        
        // 以组件中心为原点进行旋转和绘制
        canvas.save();
        canvas.translate(size / 2, size / 2);
        canvas.rotate(rotationAngle);
        
        // 绘制机器人坐标
        double radius = min(size / 2, size / 2);

        // 根据点位类型绘制不同的水波纹样式
        if (poseType == PoseType.robot) {
          // 机器人类型：绘制圆形水波纹
          for (int i = count; i >= 0; i--) {
            final double opacity = (1.0 - ((i + animationValue) / (count + 1)));
            final paint = Paint()
              ..color = color.withOpacity(opacity)
              ..style = PaintingStyle.fill;

            double _radius = radius * ((i + animationValue) / (count + 1));
            canvas.drawCircle(Offset.zero, _radius, paint);
          }
        } else {
          // 导航点类型：绘制菱形水波纹
          for (int i = count; i >= 0; i--) {
            final double opacity = (1.0 - ((i + animationValue) / (count + 1)));
            final paint = Paint()
              ..color = color.withOpacity(opacity)
              ..style = PaintingStyle.fill;

            double _radius = radius * ((i + animationValue) / (count + 1));

            // 计算菱形的四个顶点，中心点在(0, 0)
            final path = Path()
              ..moveTo(_radius, 0) // 右顶点
              ..lineTo(0, _radius) // 下顶点
              ..lineTo(-_radius, 0) // 左顶点
              ..lineTo(0, -_radius) // 上顶点
              ..close(); // 闭合路径

            // 绘制路径
            canvas.drawPath(path, paint);
          }
        }

        // 根据点位类型绘制不同的中心样式
        if (poseType == PoseType.robot) {
          // 机器人类型：绘制圆形中心，添加轻微的脉动动画
          final double centerPulse = 1.0 + 0.1 * sin(animationValue * 4 * pi);
          final centerPaint = Paint()
            ..color = color.withOpacity(0.8 + 0.2 * sin(animationValue * 2 * pi))
            ..style = PaintingStyle.fill;
          
          final centerRadius = radius / 3 * centerPulse;
          canvas.drawCircle(Offset.zero, centerRadius, centerPaint);
        } else {
          // 导航点类型：绘制菱形中心，添加轻微的脉动动画
          final double centerPulse = 1.0 + 0.1 * sin(animationValue * 4 * pi);
          final centerPaint = Paint()
            ..color = color.withOpacity(0.8 + 0.2 * sin(animationValue * 2 * pi))
            ..style = PaintingStyle.fill;
          
          final centerPath = Path()
            ..moveTo(radius / 3 * centerPulse, 0) // 右顶点
            ..lineTo(0, radius / 3 * centerPulse) // 下顶点
            ..lineTo(-radius / 3 * centerPulse, 0) // 左顶点
            ..lineTo(0, -radius / 3 * centerPulse) // 上顶点
            ..close(); // 闭合路径

          // 绘制路径
          canvas.drawPath(centerPath, centerPaint);
        }

        // 绘制方向指示器，中心点在(0, 0)
        Paint dirPainter = Paint()
          ..style = PaintingStyle.fill
          ..color = color.withOpacity(0.6);
        
        Rect rect = Rect.fromCircle(
            center: Offset.zero, radius: radius);
        canvas.drawArc(rect, -deg2rad(15), deg2rad(30), true, dirPainter);
        
        canvas.restore(); // 恢复旋转变换
      }
      
      canvas.restore();
    } catch (e) {
      print('Error rendering PoseComponent: $e');
    }
  }
}

