import 'dart:math';
import 'dart:ui';
import 'package:flame/components.dart';
import 'package:flame/game.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';

class WayPoint extends PositionComponent with HasGameRef {
  late double waypointSize;
  late Color color;
  int count;
  late Timer animationTimer;
  double animationValue = 0.0;

  WayPoint({
    required this.waypointSize,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
  });

  @override
  Future<void> onLoad() async {
    size = Vector2.all(waypointSize);
    animationTimer = Timer(
      2.0,
      onTick: () {
        animationValue = 0.0;
      },
      repeat: true,
    );
    add(WayPointRenderer(
      size: waypointSize,
      color: color,
      count: count,
      animationValue: animationValue,
    ));
  }

  @override
  void update(double dt) {
    animationTimer.update(dt);
    animationValue = (animationTimer.progress * 2.0) % 1.0;
    super.update(dt);
  }

  @override
  void onRemove() {
    super.onRemove();
  }
}

class WayPointRenderer extends Component with HasGameRef {
  final double size;
  final Color color;
  final int count;
  final double animationValue;

  WayPointRenderer({
    required this.size,
    required this.color,
    required this.count,
    required this.animationValue,
  });

  @override
  void render(Canvas canvas) {
    // 添加安全检查
    if (!isMounted) {
      return;
    }
    
    try {
      canvas.save();
      
      // 绘制机器人坐标
      double radius = min(size / 2, size / 2);

      for (int i = count; i >= 0; i--) {
        final double opacity = (1.0 - ((i + animationValue) / (count + 1)));
        final paint = Paint()
          ..color = color.withOpacity(opacity)
          ..style = PaintingStyle.fill;

        double _radius = radius * ((i + animationValue) / (count + 1));

        // 计算菱形的四个顶点
        final path = Path()
          ..moveTo(size / 2 + _radius, size / 2) // 右顶点
          ..lineTo(size / 2, size / 2 + _radius) // 下顶点
          ..lineTo(size / 2 - _radius, size / 2) // 左顶点
          ..lineTo(size / 2, size / 2 - _radius) // 上顶点
          ..close(); // 闭合路径

        // 绘制路径
        canvas.drawPath(path, paint);
      }

      // 绘制中心菱形
      final centerPaint = Paint()
        ..color = color
        ..style = PaintingStyle.fill;
      
      final centerPath = Path()
        ..moveTo(size / 2 + radius / 3, size / 2) // 右顶点
        ..lineTo(size / 2, size / 2 + radius / 3) // 下顶点
        ..lineTo(size / 2 - radius / 3, size / 2) // 左顶点
        ..lineTo(size / 2, size / 2 - radius / 3) // 上顶点
        ..close(); // 闭合路径

      // 绘制路径
      canvas.drawPath(centerPath, centerPaint);

      // 绘制方向指示器
      Paint dirPainter = Paint()..style = PaintingStyle.fill;
      dirPainter.color = color.withOpacity(0.3);
      Rect rect = Rect.fromCircle(
          center: Offset(size / 2, size / 2), radius: radius);
      canvas.drawArc(rect, -deg2rad(15), deg2rad(30), true, dirPainter);
      
      canvas.restore();
    } catch (e) {
      print('Error rendering waypoint: $e');
    }
  }
}
