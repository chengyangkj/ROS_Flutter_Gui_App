import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';

class DisplayTopologyLine extends StatefulWidget {
  final List<NavPoint> points;
  final List<TopologyRoute> routes;
  final double mapResolution;
  final Offset mapOrigin;

  const DisplayTopologyLine({
    Key? key,
    required this.points,
    required this.routes,
    required this.mapResolution,
    required this.mapOrigin,
  }) : super(key: key);

  @override
  _DisplayTopologyLineState createState() => _DisplayTopologyLineState();
}

class _DisplayTopologyLineState extends State<DisplayTopologyLine>
    with TickerProviderStateMixin {
  late AnimationController _animationController;

  @override
  void initState() {
    super.initState();
    _animationController = AnimationController(
      duration: const Duration(seconds: 2),
      vsync: this,
    )..repeat();
  }

  @override
  void dispose() {
    _animationController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _animationController,
      builder: (context, child) {
        return CustomPaint(
          painter: TopologyLinePainter(
            points: widget.points,
            routes: widget.routes,
            mapResolution: widget.mapResolution,
            mapOrigin: widget.mapOrigin,
            animationValue: _animationController.value,
          ),
        );
      },
    );
  }
}

class TopologyLinePainter extends CustomPainter {
  final List<NavPoint> points;
  final List<TopologyRoute> routes;
  final double mapResolution;
  final Offset mapOrigin;
  final double animationValue;

  TopologyLinePainter({
    required this.points,
    required this.routes,
    required this.mapResolution,
    required this.mapOrigin,
    required this.animationValue,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 2.0
      ..strokeCap = StrokeCap.round;

    final Map<String, Offset> pointMap = {};
    for (final point in points) {
      pointMap[point.name] = Offset(point.x, point.y);
    }

    // 统计每条路径的连接数量以判断是否为双向
    final Map<String, List<TopologyRoute>> connectionMap = {};
    for (final route in routes) {
      final key = _getConnectionKey(route.fromPoint, route.toPoint);
      connectionMap.putIfAbsent(key, () => []).add(route);
    }

    // 绘制路径
    for (final entry in connectionMap.entries) {
      final routeList = entry.value;
      final firstRoute = routeList.first;
      
      final fromPoint = pointMap[firstRoute.fromPoint];
      final toPoint = pointMap[firstRoute.toPoint];
      
      if (fromPoint != null && toPoint != null) {
        final isBidirectional = routeList.length > 1;
        
        if (isBidirectional) {
          _drawBidirectionalPath(canvas, fromPoint, toPoint, paint);
        } else {
          _drawUnidirectionalPath(canvas, fromPoint, toPoint, paint);
        }
      }
    }
  }

  String _getConnectionKey(String from, String to) {
    final sorted = [from, to]..sort();
    return '${sorted[0]}_${sorted[1]}';
  }

  Map<String, Offset> _adjustLineToPointEdges(Offset from, Offset to) {
    const double pointRadius = 4.0; // 导航点半径
    
    final direction = to - from;
    final distance = direction.distance;
    
    if (distance == 0) {
      return {'from': from, 'to': to};
    }
    
    final normalizedDirection = direction / distance;
    
    // 调整起点和终点，使线段从点位边缘开始
    final adjustedFrom = from + normalizedDirection * pointRadius;
    final adjustedTo = to - normalizedDirection * pointRadius;
    
    return {'from': adjustedFrom, 'to': adjustedTo};
  }

  void _drawUnidirectionalPath(Canvas canvas, Offset from, Offset to, Paint paint) {
    // 缩短线段到点位边缘
    final adjustedPoints = _adjustLineToPointEdges(from, to);
    final adjustedFrom = adjustedPoints['from']!;
    final adjustedTo = adjustedPoints['to']!;
    
    // 绘制底层路径
    paint.color = Color(0xFF6B7280).withOpacity(0.4); // 现代灰色
    paint.strokeWidth = 2.0;
    canvas.drawLine(adjustedFrom, adjustedTo, paint);
    
    // 绘制箭头流动效果
    _drawArrowFlow(canvas, adjustedFrom, adjustedTo, paint, Color(0xFF3B82F6), 0); // 现代蓝色
  }

  void _drawBidirectionalPath(Canvas canvas, Offset from, Offset to, Paint paint) {
    // 缩短线段到点位边缘
    final adjustedPoints = _adjustLineToPointEdges(from, to);
    final adjustedFrom = adjustedPoints['from']!;
    final adjustedTo = adjustedPoints['to']!;
    
    final direction = (adjustedTo - adjustedFrom).normalize();
    final perpendicular = Offset(-direction.dy, direction.dx);
    final offset = perpendicular * 2.0;

    final line1Start = adjustedFrom + offset;
    final line1End = adjustedTo + offset;
    final line2Start = adjustedFrom - offset;
    final line2End = adjustedTo - offset;

    // 绘制底层路径
    paint.color = Color(0xFF6B7280).withOpacity(0.4); // 现代灰色
    paint.strokeWidth = 2.0;
    canvas.drawLine(line1Start, line1End, paint);
    canvas.drawLine(line2Start, line2End, paint);

    // 绘制箭头流动效果
    _drawArrowFlow(canvas, line1Start, line1End, paint, Color(0xFF10B981), 0); // 现代绿色
    _drawArrowFlow(canvas, line2End, line2Start, paint, Color(0xFF10B981), 0.5);
  }

  void _drawArrowFlow(Canvas canvas, Offset start, Offset end, Paint paint, Color color, double phaseOffset) {
    final direction = end - start;
    final distance = direction.distance;
    if (distance == 0) return;
    
    final normalizedDirection = direction / distance;
    final perpendicular = Offset(-normalizedDirection.dy, normalizedDirection.dx);
    
    // 箭头参数
    final arrowSpacing = 40.0;
    final triangleSize = 4.0; // 三角形大小，确保在路径内
    
    // 动画偏移（修正方向）
    double animationOffset = animationValue * arrowSpacing % arrowSpacing;
    
    // 绘制流动三角形
    double currentDistance = animationOffset;
    
    while (currentDistance < distance) {
      if (currentDistance > triangleSize && currentDistance < distance - triangleSize) {
        final triangleCenter = start + normalizedDirection * currentDistance;
        
        // 计算三角形透明度
        final distanceRatio = currentDistance / distance;
        final opacity = (0.8 - distanceRatio * 0.3).clamp(0.0, 1.0);
        
        // 绘制实心三角形
        final trianglePoints = [
          triangleCenter + normalizedDirection * triangleSize, // 前端点
          triangleCenter - normalizedDirection * triangleSize * 0.5 + perpendicular * triangleSize * 0.5, // 左后点
          triangleCenter - normalizedDirection * triangleSize * 0.5 - perpendicular * triangleSize * 0.5, // 右后点
        ];
        
        final path = Path();
        path.moveTo(trianglePoints[0].dx, trianglePoints[0].dy);
        path.lineTo(trianglePoints[1].dx, trianglePoints[1].dy);
        path.lineTo(trianglePoints[2].dx, trianglePoints[2].dy);
        path.close();
        
        paint.color = color.withOpacity(opacity);
        paint.style = PaintingStyle.fill;
        canvas.drawPath(path, paint);
      }
      
      currentDistance += arrowSpacing;
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

extension OffsetExtension on Offset {
  Offset normalize() {
    final length = distance;
    if (length == 0) return Offset.zero;
    return this / length;
  }
} 