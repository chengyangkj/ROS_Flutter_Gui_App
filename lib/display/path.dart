import 'dart:math';
import 'dart:ui';
import 'package:flame/components.dart';
import 'package:flame/game.dart';
import 'package:flutter/material.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

class PathComponent extends Component with HasGameRef {
  List<vm.Vector2> pointList = [];
  Color color = Colors.green;
  late Timer animationTimer;
  double animationValue = 0.0;
  
  PathComponent({
    required this.pointList, 
    required this.color
  });

  @override
  Future<void> onLoad() async {
    animationTimer = Timer(
      2.0,
      onTick: () {
        animationValue = 0.0;
      },
      repeat: true,
    );
    add(PathRenderer(
      pointList: pointList,
      color: color,
      animationValue: animationValue,
    ));
  }

  @override
  void update(double dt) {
    animationTimer.update(dt);
    animationValue = (animationTimer.progress * 2.0) % 1.0;
    
    // 更新渲染器的动画值
    final renderer = children.whereType<PathRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updateAnimationValue(animationValue);
    }
    
    super.update(dt);
  }

  void updatePath(List<vm.Vector2> newPoints) {
    pointList = newPoints;
    // 更新渲染器的点列表
    final renderer = children.whereType<PathRenderer>().firstOrNull;
    if (renderer != null) {
      renderer.updatePath(newPoints);
    }
  }

  @override
  void onRemove() {
    super.onRemove();
  }
}

class PathRenderer extends Component with HasGameRef {
  List<vm.Vector2> pointList = [];
  Color color;
  double animationValue;

  PathRenderer({
    required this.pointList,
    required this.color,
    required this.animationValue,
  });

  void updatePath(List<vm.Vector2> newPoints) {
    pointList = newPoints;
  }

  void updateAnimationValue(double newValue) {
    animationValue = newValue;
  }

  @override
  void render(Canvas canvas) {
    // 添加安全检查
    if (!isMounted) {
      return;
    }
    
    try {
      if (pointList.isEmpty || pointList.length < 2) return;

      // 绘制主路径线
      _drawMainPath(canvas);
      
      // 在路径上添加流动箭头纹理
      _drawFlowingArrows(canvas);
    } catch (e) {
      print('Error rendering path: $e');
    }
  }

  void _drawMainPath(Canvas canvas) {
    final path = Path();
    final paint = Paint()
      ..color = color.withOpacity(0.6)
      ..strokeCap = StrokeCap.round
      ..strokeWidth = 1.0
      ..style = PaintingStyle.stroke;

    // 移动到第一个点
    path.moveTo(pointList[0].x, pointList[0].y);
    
    // 连接所有点形成路径
    for (int i = 1; i < pointList.length; i++) {
      path.lineTo(pointList[i].x, pointList[i].y);
    }

    // 绘制主路径线
    canvas.drawPath(path, paint);
  }

  void _drawFlowingArrows(Canvas canvas) {
    if (pointList.length < 2) return;

    final paint = Paint()
      ..color = color.withOpacity(0.7)
      ..strokeWidth = 1.0
      ..style = PaintingStyle.fill;

    // 在路径上绘制流动的箭头
    for (int i = 0; i < pointList.length - 1; i++) {
      vm.Vector2 currentPoint = pointList[i];
      vm.Vector2 nextPoint = pointList[i + 1];
      
      // 计算当前段的方向
      vm.Vector2 direction = (nextPoint - currentPoint).normalized();
      
      // 绘制流动箭头
      _drawFlowingArrowSegment(canvas, currentPoint, nextPoint, direction, paint);
    }
  }

  void _drawFlowingArrowSegment(Canvas canvas, vm.Vector2 start, vm.Vector2 end, vm.Vector2 direction, Paint paint) {
    final distance = (end - start).length;
    if (distance == 0) return;
    
    // 箭头参数
    final arrowSpacing = 12.0;
    final triangleSize = 2.0;
    
    // 动画偏移
    double animationOffset = animationValue * arrowSpacing % arrowSpacing;
    
    // 绘制流动三角形
    double currentDistance = animationOffset;
    
    while (currentDistance < distance) {
      if (currentDistance > triangleSize && currentDistance < distance - triangleSize) {
        final triangleCenter = start + direction * currentDistance;
        
        // 计算三角形透明度
        final distanceRatio = currentDistance / distance;
        final opacity = (0.8 - distanceRatio * 0.3).clamp(0.0, 1.0);
        
        // 绘制实心三角形
        final perpendicular = vm.Vector2(-direction.y, direction.x);
        final trianglePoints = [
          triangleCenter + direction * triangleSize, // 前端点
          triangleCenter - direction * triangleSize * 0.5 + perpendicular * triangleSize * 0.5, // 左后点
          triangleCenter - direction * triangleSize * 0.5 - perpendicular * triangleSize * 0.5, // 右后点
        ];
        
        final path = Path();
        path.moveTo(trianglePoints[0].x, trianglePoints[0].y);
        path.lineTo(trianglePoints[1].x, trianglePoints[1].y);
        path.lineTo(trianglePoints[2].x, trianglePoints[2].y);
        path.close();
        
        paint.color = color.withOpacity(opacity);
        canvas.drawPath(path, paint);
      }
      
      currentDistance += arrowSpacing;
    }
  }
}
