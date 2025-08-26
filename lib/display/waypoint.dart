import 'dart:math';
import 'dart:ui';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

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

  WayPoint({
    required this.waypointSize,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
    this.isEditMode = false,
    this.directionAngle = 0.0,
    this.onDragUpdate,
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
  bool _isRotating = false; // 添加旋转状态标志
  bool _isPositionDragging = false; // 添加位置拖拽状态标志
  Vector2? _positionDragStart; // 位置拖拽起始位置

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
    
    // 判断拖拽类型：如果点击在圆环附近，认为是旋转拖拽
    final center = size / 2;
    final distance = (event.localPosition - center).length;
    final ringRadius = controlSize / 2;
    
    if (distance <= ringRadius/2) { // 中心区域，位置拖拽
      _isRotating = false;
      _isPositionDragging = true;
      _positionDragStart = event.localPosition;
      return true; // 返回true来处理位置拖拽
    } else if (distance <= ringRadius + 2) { // 圆环区域，旋转拖拽
      _isRotating = true;
      _isPositionDragging = false;
      return true;
    } else {
      // 超出控制区域，不处理
      return false;
    }
  }
  
  @override
  bool onDragUpdate(DragUpdateEvent event) {
    if (_isRotating) {
      // 处理旋转拖拽
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
        
        print("DirectionControl: 旋转角度更新: $_currentAngle");
        
        // 通知父组件更新方向角度
        onDirectionChanged(_currentAngle);
        
        // 如果父组件是WayPoint，调用拖拽更新回调
        if (parent is WayPoint) {
          final wayPoint = parent as WayPoint;
          wayPoint.onDragUpdate?.call();
        }
        
        // 更新渲染器
        final renderer = children.whereType<DirectionControlRenderer>().firstOrNull;
        if (renderer != null) {
          renderer.updateAngle(_currentAngle);
        }
      }
      return true;
    } else if (_isPositionDragging) {
      // 处理位置拖拽，直接传递给父组件
      if (parent is WayPoint && _positionDragStart != null) {
        final wayPoint = parent as WayPoint;
        final game = wayPoint.gameRef;
        
        if (game != null) {
          // 直接使用事件提供的增量
          final delta = event.localDelta;
          
          // 将屏幕增量转换为世界坐标增量
          final worldDelta = delta;
          
          // 更新路径点位置
          wayPoint.position = wayPoint.position + worldDelta;
          
          // 调用父组件的拖拽更新回调
          wayPoint.onDragUpdate?.call();
        }
      }
      return true;
    }
    
    return false;
  }
  
  @override
  bool onDragEnd(DragEndEvent event) {
    if (_isRotating) {
      print('DirectionControl: 旋转拖拽结束');
    } else if (_isPositionDragging) {
      print('DirectionControl: 位置拖拽结束');
    }
    
    _lastMousePosition = null;
    _isRotating = false;
    _isPositionDragging = false;
    _positionDragStart = null;
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

class WayPointRenderer extends Component with HasGameRef, DragCallbacks {
  final double size;
  final Color color;
  final int count;
  double animationValue; // 移除final，允许更新
  
  // 拖拽相关变量
  Vector2? _dragStartPosition;
  Vector2? _dragStartWorldPosition;
  bool _isDragging = false; // 添加拖拽状态标志

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
  
  // 拖拽开始
  @override
  bool onDragStart(DragStartEvent event) {
    print('WayPointRenderer: 拖拽开始');
    _dragStartPosition = event.localPosition;
    _isDragging = true; // 设置拖拽状态
    if (parent is WayPoint) {
      _dragStartWorldPosition = (parent as WayPoint).position.clone();
      print('WayPointRenderer: 记录起始位置 ${_dragStartWorldPosition}');
    }
    return true;
  }
  
  // 拖拽更新
  @override
  bool onDragUpdate(DragUpdateEvent event) {
    if (_dragStartPosition != null && _dragStartWorldPosition != null && parent is WayPoint) {
      final wayPoint = parent as WayPoint;
      final game = gameRef;
      
      if (game != null) {
        // 计算拖拽增量
        final delta = event.localDelta;
        
        // 将屏幕增量转换为世界坐标增量
        final worldDelta = delta / game.camera.viewfinder.zoom;
        
        // 更新路径点位置
        wayPoint.position = _dragStartWorldPosition! + worldDelta;
        print('WayPointRenderer: 更新位置 ${wayPoint.position}');
        
        // 调用父组件的拖拽更新回调
        wayPoint.onDragUpdate?.call();
      }
    }
    return true;
  }
  
  // 拖拽结束
  @override
  bool onDragEnd(DragEndEvent event) {
    print('WayPointRenderer: 拖拽结束');
    _dragStartPosition = null;
    _dragStartWorldPosition = null;
    _isDragging = false; // 清除拖拽状态
    return true;
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
        
        // 根据拖拽状态调整颜色
        final currentColor = _isDragging ? color.withOpacity(0.7) : color;
        
        // 绘制机器人坐标
        double radius = min(size / 2, size / 2);

        for (int i = count; i >= 0; i--) {
          final double opacity = (1.0 - ((i + animationValue) / (count + 1)));
          final paint = Paint()
            ..color = currentColor.withOpacity(opacity)
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
          ..color = currentColor.withOpacity(0.8 + 0.2 * sin(animationValue * 2 * pi))
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
          ..color = currentColor.withOpacity(0.6);
        
        Rect rect = Rect.fromCircle(
            center: Offset.zero, radius: radius);
        canvas.drawArc(rect, -deg2rad(15), deg2rad(30), true, dirPainter);
        
        // 如果正在拖拽，绘制拖拽指示器
        if (_isDragging) {
          final dragPaint = Paint()
            ..color = Colors.yellow.withOpacity(0.8)
            ..style = PaintingStyle.stroke
            ..strokeWidth = 2;
          
          canvas.drawCircle(Offset.zero, radius + 3, dragPaint);
        }
        
        canvas.restore(); // 恢复旋转变换
      }
      
      canvas.restore();
    } catch (e) {
      print('Error rendering waypoint: $e');
    }
  }
}

