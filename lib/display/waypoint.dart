import 'dart:math';
import 'dart:ui';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
// import 'package:vector_math/vector_math_64.dart' as vm;

class WayPoint extends PositionComponent with HasGameRef {
  late double waypointSize;
  late Color color;
  int count;
  late Timer animationTimer;
  double animationValue = 0.0;
  
  // 添加选中状态
  bool isSelected = false;
  
  // 添加编辑模式控制
  bool isEditMode = false;
  
  // 添加方向角度（弧度）
  double directionAngle = 0.0;
  
  // 添加拖拽更新回调
  VoidCallback? onDragUpdate;
  
  // 添加点击回调
  Function(NavPoint)? onTap;
  
  // 存储导航点信息
  NavPoint? navPoint;
  
  // 组件内处理手势，无需对外状态

  WayPoint({
    required this.waypointSize,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
    this.isEditMode = false,
    this.directionAngle = 0.0,
    this.onDragUpdate,
    this.onTap,
    this.navPoint,
  });
  
  @override
  Future<void> onLoad() async {
    size = Vector2.all(waypointSize);
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
    add(WayPointRenderer(
      size: waypointSize,
      color: color,
      count: count,
      animationValue: animationValue,
    ));
    
    // 根据编辑模式决定是否添加方向控制组件
    if (isEditMode) {
      // 添加方向控制组件
      final directionControl = DirectionControl(
        controlSize: waypointSize + 6, // 调整大小，确保圆环能完全包裹点位
        onDirectionChanged: updateDirectionAngle,
        initAngle: directionAngle,
      );
      // 设置方向控制组件只响应圆环区域的拖拽
      directionControl.size = Vector2(waypointSize + 6, waypointSize + 6);
      add(directionControl);
    }
  }

  @override
  void update(double dt) {
    animationTimer.update(dt);
    // 直接使用progress，确保动画值在0到1之间
    animationValue = animationTimer.progress;
    
    // 同步更新渲染器的动画值
    final renderer = children.whereType<WayPointRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updateAnimationValue(animationValue);
    }
    
    // 添加调试信息：每100帧打印一次位置信息
    frameCount++;
    if (frameCount % 100 == 0) {
      print('WayPoint调试: 位置=${position}, 大小=${size}, 方向角度=${directionAngle}');
      print('WayPoint调试: 动画值=${animationValue}, 帧数=${frameCount}');
    }
    
    super.update(dt);
  }
  
  int frameCount = 0;

  @override
  void onRemove() {
    super.onRemove();
  }

  // 更新方向角度
  void updateDirectionAngle(double newAngle) {
    print('WayPoint.updateDirectionAngle: 旧角度=$directionAngle, 新角度=$newAngle');
    directionAngle = newAngle;
    
    // 同步更新DirectionControl的角度
    final directionControl = children.whereType<DirectionControl>().firstOrNull;
    if (directionControl != null) {
      print('WayPoint: 找到DirectionControl，调用updateAngle');
      directionControl.updateAngle(directionAngle);
    } else {
      print('WayPoint: 未找到DirectionControl');
    }
  }
  
  // 静默设置角度，避免回调递归
  void _setAngleSilent(double newAngle) {
    directionAngle = newAngle;
    final renderer = children.whereType<DirectionControlRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updateAngle(directionAngle);
    }
  }
  
  // 对外暴露：直接设置角度（静默）
  void setAngleDirect(double newAngle) {
    _setAngleSilent(newAngle);
  }
  
  // 设置导航点信息
  void setNavPoint(NavPoint point) {
    print('WayPoint.setNavPoint: 设置导航点 ${point.name}');
    navPoint = point;
  }
  
  // 处理点击事件（由外部调用）
  void handleTap() {
    if (onTap != null && navPoint != null) {
      print('WayPoint.handleTap: 调用onTap回调，导航点: ${navPoint!.name}');
      onTap!(navPoint!);
    } else {
      print('WayPoint.handleTap: onTap或navPoint为空: onTap=${onTap}, navPoint=${navPoint}');
    }
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
    
    // 设置位置为0,0，确保与父组件中心对齐
    position = Vector2.zero();
    
    // 设置锚点为中心
    anchor = Anchor.center;
    
    add(DirectionControlRenderer(
      size: controlSize,
      angle: _currentAngle,
    ));
  }
  
  // 检查触摸点是否在旋转控制区域内
  bool isInRotationArea(Vector2 touchPoint) {
    // 以组件中心(0,0)为原点计算半径
    final double radius = touchPoint.length;
    final double outerRadius = controlSize / 2;       // 外圈半径（方向环外缘）
    final double innerRadius = outerRadius - 3.0;     // 内圈半径（方向环内缘，3像素厚度）
    
    // 旋转：在外层细环内；移动：在内圈（不包含细环）
    final bool inRotationRing = radius >= innerRadius && radius <= outerRadius;
    return inRotationRing;
  }
  
  @override
  bool onDragStart(DragStartEvent event) {
    super.onDragStart(event);
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
      if (parent is WayPoint && _positionDragStart != null) {
        final wayPoint = parent as WayPoint;
        final delta = event.localDelta;
        wayPoint.position = wayPoint.position + delta;
        wayPoint.onDragUpdate?.call();
      }
      return true;
    }
    return false;
  }

  @override
  bool onDragEnd(DragEndEvent event) {
    super.onDragEnd(event);
    _lastMousePosition = null;
    _isRotating = false;
    _isPositionDragging = false;
    _positionDragStart = null;
    return true;
  }

  // 更新角度（由外部手势处理调用）
  void updateAngle(double newAngle) {
    print('DirectionControl.updateAngle: 旧角度=$_currentAngle, 新角度=$newAngle, _isUpdating=$_isUpdating');
    
    if (_isUpdating) {
      print('DirectionControl.updateAngle: 正在更新中，跳过重复调用');
      return; // 防止重复调用
    }
    
    _isUpdating = true;
    _currentAngle = newAngle;
    
    print('DirectionControl.updateAngle: 调用onDirectionChanged回调');
    // 通知父组件更新方向角度
    onDirectionChanged(_currentAngle);
    
    // 更新渲染器
    final renderer = children.whereType<DirectionControlRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updateAngle(_currentAngle);
    }
    
    _isUpdating = false;
    print('DirectionControl.updateAngle: 更新完成');
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
    
    // 绘制主圆圈
    final circlePaint = Paint()
      ..color = Colors.blue.withOpacity(0.6)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;
    
    canvas.drawCircle(center, radius, circlePaint);
    
    // 绘制方向指示点
    final pointPaint = Paint()
      ..color = Colors.red
      ..style = PaintingStyle.fill;
    
    final pointOffset = Offset(
      center.dx + radius * cos(angle),
      center.dy + radius * sin(angle),
    );
    
    canvas.drawCircle(pointOffset, 1, pointPaint);
  }
}

// 路径点渲染器
class WayPointRenderer extends Component with HasGameRef {
  final double size;
  final Color color;
  final int count;
  double animationValue; // 移除final，允许更新

  WayPointRenderer({
    required this.size,
    required this.color,
    required this.count,
    required this.animationValue,
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
      if (parentComponent is WayPoint) {
        final double rotationAngle = parentComponent.directionAngle;
        
        // 应用旋转变换，整个组件按照中心旋转
        canvas.save();
        canvas.rotate(rotationAngle);
        
        // 绘制机器人坐标
        double radius = min(size / 2, size / 2);

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

        // 绘制中心菱形，中心点在(0, 0)，添加轻微的脉动动画
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
      print('Error rendering waypoint: $e');
    }
  }
}

