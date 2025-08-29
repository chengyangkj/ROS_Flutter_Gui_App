import 'dart:ui';
import 'dart:math';
import 'package:flame/components.dart';
import 'package:flutter/material.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

class PolygonComponent extends Component {
  List<vm.Vector2> pointList = [];
  Color color = Colors.green;
  bool enableWaterDropAnimation = false;
  
  late double _animationTime;
  
  PolygonComponent({
    required this.pointList, 
    required this.color,
    this.enableWaterDropAnimation = false, // 默认启用动画
  }) {
    _animationTime = 0.0;
  }

  bool get hasLayout => true;

  void updatePointList(List<vm.Vector2> newPointList) {
    pointList = newPointList;
  }

  @override
  void update(double dt) {
    super.update(dt);
    if (enableWaterDropAnimation) {
      _animationTime += dt;
    }
  }

  @override
  void render(Canvas canvas) {
    // 即使点数不足，也要绘制动画效果
    if (enableWaterDropAnimation) {
      _renderWaterDropAnimation(canvas);
    } else if (pointList.length >= 3) {
      _renderNormalPolygon(canvas);
    }
  }

  void _renderNormalPolygon(Canvas canvas) {
    Paint paint = Paint()
      ..color = color
      ..strokeWidth = 0.1
      ..style =  PaintingStyle.stroke;
      
    Path path = Path();
    path.moveTo(pointList[0].x, pointList[0].y);
    
    for (int i = 1; i < pointList.length; i++) {
      path.lineTo(pointList[i].x, pointList[i].y);
    }
    path.close();
    
    canvas.drawPath(path, paint);
  }

  void _renderWaterDropAnimation(Canvas canvas) {
    // 如果没有点，使用默认中心点
    vm.Vector2 center;
    if (pointList.isEmpty) {
      // 使用画布中心作为默认中心点
      center = vm.Vector2(canvas.getLocalClipBounds().center.dx, canvas.getLocalClipBounds().center.dy);
    } else {
      center = _calculatePolygonCenter();
    }
    
    int numLayers = 5;
    double animationValue = (_animationTime / 4.0) % 1.0;
    
    for (int i = 0; i < numLayers; i++) {
      final double baseProgress = animationValue + (i * 0.2);
      final double progress = (baseProgress % 1.0);
      final double opacity = (1.0 - progress) * 0.9;
      
      if (opacity <= 0.0) continue;
      
      double scale = 0.1 + (progress * 0.9);
      
      List<vm.Vector2> scaledPoints = _scalePolygonFromCenter(center, scale);
      
      if (scaledPoints.isNotEmpty) {
        Paint paint = Paint()
          ..color = color.withOpacity(opacity)
          ..strokeWidth = 0.4
          ..style = PaintingStyle.fill;
        
        Path path = Path();
        path.moveTo(scaledPoints[0].x, scaledPoints[0].y);
        
        for (int j = 1; j < scaledPoints.length; j++) {
          path.lineTo(scaledPoints[j].x, scaledPoints[j].y);
        }
        path.close();
        
        canvas.drawPath(path, paint);
      }
    }
  }

  vm.Vector2 _calculatePolygonCenter() {
    double sumX = 0.0;
    double sumY = 0.0;
    
    for (vm.Vector2 point in pointList) {
      sumX += point.x;
      sumY += point.y;
    }
    
    return vm.Vector2(sumX / pointList.length, sumY / pointList.length);
  }

  double _calculateMaxDistanceFromCenter(vm.Vector2 center) {
    double maxDistance = 0.0;
    
    for (vm.Vector2 point in pointList) {
      double distance = (point - center).length;
      if (distance > maxDistance) {
        maxDistance = distance;
      }
    }
    
    return maxDistance;
  }

  List<vm.Vector2> _scalePolygonFromCenter(vm.Vector2 center, double scale) {
    List<vm.Vector2> scaledPoints = [];
    
    if (pointList.isEmpty) {
      // 如果没有点，创建一个简单的圆形动画
      const int numPoints = 8;
      const double radius = 20.0;
      
      for (int i = 0; i < numPoints; i++) {
        double angle = (i * 2 * pi) / numPoints;
        double x = center.x + cos(angle) * radius * scale;
        double y = center.y + sin(angle) * radius * scale;
        scaledPoints.add(vm.Vector2(x, y));
      }
    } else {
      // 有点时，正常缩放
      for (vm.Vector2 point in pointList) {
        vm.Vector2 direction = point - center;
        vm.Vector2 scaledPoint = center + direction * scale;
        scaledPoints.add(scaledPoint);
      }
    }
    
    return scaledPoints;
  }
}
