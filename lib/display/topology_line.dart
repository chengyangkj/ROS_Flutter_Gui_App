import 'dart:ui';
import 'package:flame/components.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

class TopologyLine extends Component with HasGameRef {
  final List<NavPoint> points;
  final List<TopologyRoute> routes;
  final OccupancyMap? occMap;
  final RosChannel? rosChannel;

  TopologyLine({
    required this.points,
    required this.routes,
    this.occMap,
    this.rosChannel,
  });

  @override
  Future<void> onLoad() async {
    add(TopologyLineRenderer(
      points: points,
      routes: routes,
      occMap: occMap,
      rosChannel: rosChannel,
    ));
  }
}

class TopologyLineRenderer extends Component with HasGameRef {
  final List<NavPoint> points;
  final List<TopologyRoute> routes;
  final OccupancyMap? occMap;
  final RosChannel? rosChannel;

  TopologyLineRenderer({
    required this.points,
    required this.routes,
    this.occMap,
    this.rosChannel,
  });

  // 获取当前地图数据
  OccupancyMap? get currentMap {
    if (occMap != null) {
      return occMap;
    }
    if (rosChannel != null) {
      return rosChannel!.map_.value;
    }
    return null;
  }

  // 获取地图分辨率
  double get mapResolution => currentMap?.mapConfig.resolution ?? 0.05;

  // 获取地图原点
  Offset get mapOrigin => currentMap != null 
      ? Offset(currentMap!.mapConfig.originX, currentMap!.mapConfig.originY)
      : Offset.zero;

  @override
  void render(Canvas canvas) {
    // 添加安全检查
    if (!isMounted) {
      return;
    }
    
    try {
      final paint = Paint()
        ..strokeWidth = 1.0
        ..strokeCap = StrokeCap.round;

      final currentMapData = currentMap;
      if (currentMapData == null) {
        return; // 没有地图数据时不渲染
      }

      final Map<String, Offset> pointMap = {};
      for (final point in points) {
        var occPose = currentMapData.xy2idx(vm.Vector2(point.x, point.y));
        pointMap[point.name] = Offset(occPose.x, occPose.y);
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
    } catch (e) {
      print('Error rendering topology line: $e');
    }
  }

  String _getConnectionKey(String from, String to) {
    final sorted = [from, to]..sort();
    return '${sorted[0]}_${sorted[1]}';
  }

  Map<String, Offset> _adjustLineToPointEdges(Offset from, Offset to) {
    const double pointRadius = 2.0; // 点位两端缩短距离
    
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
    paint.strokeWidth = 1.0;
    canvas.drawLine(adjustedFrom, adjustedTo, paint);
    
    // 绘制静态箭头
    _drawStaticArrowFlow(canvas, adjustedFrom, adjustedTo, paint, Color(0xFF6B7280));
  }

  void _drawBidirectionalPath(Canvas canvas, Offset from, Offset to, Paint paint) {
    // 缩短线段到点位边缘
    final adjustedPoints = _adjustLineToPointEdges(from, to);
    final adjustedFrom = adjustedPoints['from']!;
    final adjustedTo = adjustedPoints['to']!;
    
    final direction = (adjustedTo - adjustedFrom).normalize();
    final perpendicular = Offset(-direction.dy, direction.dx);
    final offset = perpendicular;

    final line1Start = adjustedFrom + offset;
    final line1End = adjustedTo + offset;
    final line2Start = adjustedFrom - offset;
    final line2End = adjustedTo - offset;

    // 绘制底层路径
    paint.color = Color(0xFF6B7280).withOpacity(0.4); // 现代灰色
    paint.strokeWidth = 1.0;
    canvas.drawLine(line1Start, line1End, paint);
    canvas.drawLine(line2Start, line2End, paint);
    
    // 绘制双向箭头
    _drawStaticArrowFlow(canvas, line1Start, line1End, paint, Color(0xFF6B7280));
    _drawStaticArrowFlow(canvas, line2End, line2Start, paint, Color(0xFF6B7280));
  }

  void _drawStaticArrowFlow(Canvas canvas, Offset start, Offset end, Paint paint, Color color) {
    final direction = end - start;
    final distance = direction.distance;
    if (distance == 0) return;
    
    final normalizedDirection = direction / distance;
    final perpendicular = Offset(-normalizedDirection.dy, normalizedDirection.dx);
    
    // 箭头参数
    final arrowSpacing = 10.0;
    final triangleSize = 1.0; // 三角形大小，确保在路径内
    
    // 绘制静态三角形（无动画偏移）
    double currentDistance = 0.0;
    
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

}

extension OffsetExtension on Offset {
  Offset normalize() {
    final length = distance;
    if (length == 0) return Offset.zero;
    return this / length;
  }
}
