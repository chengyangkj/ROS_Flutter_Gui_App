import 'dart:math';
import 'dart:ui';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';

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

  WayPoint({
    required this.waypointSize,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
    this.isEditMode = false,
    this.directionAngle = 0.0,
  });

  @override
  Future<void> onLoad() async {
    size = Vector2.all(waypointSize);
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
    
    
    super.update(dt);
  }
  
  int frameCount = 0;

  @override
  void onRemove() {
    super.onRemove();
  }

  // 更新方向角度
  void updateDirectionAngle(double newAngle) {
    directionAngle = newAngle;
  }
}

// 方向控制组件
class DirectionControl extends PositionComponent with DragCallbacks {
  final double controlSize;
  final Function(double) onDirectionChanged;
  final double initAngle;
  
  double _currentAngle = 0.0;
  Vector2? _lastMousePosition; // 添加鼠标位置跟踪
  
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
  
  @override
  bool onDragStart(DragStartEvent event) {
    _lastMousePosition = event.localPosition;
    return true;
  }
  
  @override
  bool onDragUpdate(DragUpdateEvent event) {
    if (_lastMousePosition != null) {
      // 累积鼠标位置
      _lastMousePosition = _lastMousePosition! + event.localDelta;
      
      // 计算鼠标相对于组件中心的位置
      final center = size / 2;
      
      // 计算相对于中心的角度
      final deltaX = _lastMousePosition!.x - center.x;
      final deltaY = _lastMousePosition!.y - center.y;
      final newAngle = atan2(deltaY, deltaX);
      
      // 更新角度
      _currentAngle = newAngle;
      
      print("newAngle: $_currentAngle delta: $deltaX, $deltaY");
      
      // 通知父组件更新方向角度
      onDirectionChanged(_currentAngle);
      
      // 更新渲染器
      final renderer = children.whereType<DirectionControlRenderer>().firstOrNull;
      if (renderer != null) {
        renderer.updateAngle(_currentAngle);
      }
    }
    
    return true;
  }
  
  @override
  bool onDragEnd(DragEndEvent event) {
    _lastMousePosition = null;
    return true;
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
    final startX = -size / 2;
    final startY = -size / 2;
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

