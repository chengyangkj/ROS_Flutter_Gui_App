import 'dart:ui';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayPolygon extends StatefulWidget {
  List<Offset> pointList = [];
  Color color = Colors.green;
  bool fill = false; // 是否填充多边形
  bool enableWaterDropAnimation = false; // 是否启用水滴动画
  
  DisplayPolygon({
    Key? key,
    required this.pointList, 
    required this.color,
    this.fill = false,
    this.enableWaterDropAnimation = false,
  }) : super(key: key);

  @override
  State<DisplayPolygon> createState() => _DisplayPolygonState();
}

class _DisplayPolygonState extends State<DisplayPolygon>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  late Animation<double> _animation;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      duration: const Duration(milliseconds: 4000),
      vsync: this,
    );

    _animation = Tween<double>(
      begin: 0.0,
      end: 1.0,
    ).animate(_controller);

    _controller.repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // 计算多边形的边界框
    double minX = widget.pointList.isEmpty ? 0 : widget.pointList.map((p) => p.dx).reduce((a, b) => a < b ? a : b);
    double maxX = widget.pointList.isEmpty ? 0 : widget.pointList.map((p) => p.dx).reduce((a, b) => a > b ? a : b);
    double minY = widget.pointList.isEmpty ? 0 : widget.pointList.map((p) => p.dy).reduce((a, b) => a < b ? a : b);
    double maxY = widget.pointList.isEmpty ? 0 : widget.pointList.map((p) => p.dy).reduce((a, b) => a > b ? a : b);
    
    double width = maxX - minX;
    double height = maxY - minY;
    
    return Container(
      width: width > 0 ? width : 100, // 最小宽度100
      height: height > 0 ? height : 100, // 最小高度100
      child: AnimatedBuilder(
        animation: _animation,
        builder: (context, child) {
          return CustomPaint(
            painter: _DisplayPolygonPainter(
              pointList: widget.pointList,
              color: widget.color,
              fill: widget.fill,
              animationValue: _animation.value,
              enableWaterDropAnimation: widget.enableWaterDropAnimation,
            ),
            size: Size.infinite,
          );
        },
      ),
    );
  }
}

class _DisplayPolygonPainter extends CustomPainter {
  List<Offset> pointList = [];
  Color color = Colors.green;
  bool fill = false; // 是否填充多边形
  double animationValue = 0.0; // 动画值，0.0到1.0
  bool enableWaterDropAnimation = false; // 是否启用水滴动画
  
  _DisplayPolygonPainter({
    required this.pointList, 
    required this.color,
    this.fill = false,
    this.animationValue = 0.0,
    this.enableWaterDropAnimation = false,
  });
  
  @override
  void paint(Canvas canvas, Size size) {
    if (pointList.length < 3) return; // 至少需要3个点才能形成多边形

    if (enableWaterDropAnimation) {
      _drawWaterDropAnimation(canvas, size);
    } else {
      _drawNormalPolygon(canvas, size);
    }
  }

  void _drawNormalPolygon(Canvas canvas, Size size) {
    Paint paint = Paint()
      ..color = color
      ..strokeWidth = 2
      ..style = fill ? PaintingStyle.fill : PaintingStyle.stroke;
      
    Path path = Path();
    path.moveTo(pointList[0].dx, pointList[0].dy);
    
    for (int i = 1; i < pointList.length; i++) {
      path.lineTo(pointList[i].dx, pointList[i].dy);
    }
    path.close();
    
    canvas.drawPath(path, paint);
  }

  void _drawWaterDropAnimation(Canvas canvas, Size size) {
    // 计算多边形的中心点
    Offset center = _calculatePolygonCenter();
    
    // 计算从中心到各顶点的最大距离
    double maxDistance = _calculateMaxDistanceFromCenter(center);
    
    // 创建多个同心多边形，从内向外扩散
    int numLayers = 5; // 增加水滴层数以获得更平滑的效果
    
    // 修改动画逻辑，创建连续的水滴效果
    for (int i = 0; i < numLayers; i++) {
      // 计算当前层的动画进度，使用连续的动画值
      final double baseProgress = animationValue + (i * 0.2); // 每层间隔0.2
      final double progress = (baseProgress % 1.0); // 确保进度在0-1之间
      final double opacity = (1.0 - progress) * 0.9; // 降低最大透明度
      
      if (opacity <= 0.0) continue;
      
      // 计算当前层的缩放比例，从0.1开始到1.0
      double scale = 0.3 + (progress * 0.9);
      
      // 创建缩放后的点列表
      List<Offset> scaledPoints = _scalePolygonFromCenter(center, scale);
      
      // 绘制当前层
      Paint paint = Paint()
        ..color = color.withOpacity(opacity)
        ..strokeWidth = 1
        ..style = PaintingStyle.fill;
      
      Path path = Path();
      path.moveTo(scaledPoints[0].dx, scaledPoints[0].dy);
      
      for (int j = 1; j < scaledPoints.length; j++) {
        path.lineTo(scaledPoints[j].dx, scaledPoints[j].dy);
      }
      path.close();
      
      canvas.drawPath(path, paint);
    }
    
  }

  Offset _calculatePolygonCenter() {
    double sumX = 0.0;
    double sumY = 0.0;
    
    for (Offset point in pointList) {
      sumX += point.dx;
      sumY += point.dy;
    }
    
    return Offset(sumX / pointList.length, sumY / pointList.length);
  }

  double _calculateMaxDistanceFromCenter(Offset center) {
    double maxDistance = 0.0;
    
    for (Offset point in pointList) {
      double distance = (point - center).distance;
      if (distance > maxDistance) {
        maxDistance = distance;
      }
    }
    
    return maxDistance;
  }

  List<Offset> _scalePolygonFromCenter(Offset center, double scale) {
    List<Offset> scaledPoints = [];
    
    for (Offset point in pointList) {
      Offset direction = point - center;
      Offset scaledPoint = center + direction * scale;
      scaledPoints.add(scaledPoint);
    }
    
    return scaledPoints;
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}