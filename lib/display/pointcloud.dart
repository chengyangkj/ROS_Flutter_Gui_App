import 'dart:ui';
import 'package:flame/components.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/pointcloud2.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

class PointCloudComponent extends Component {
  List<Point3D> pointList = [];
  OccupancyMap map;
  List<vm.Vector2> _transformedPoints = [];
  
  PointCloudComponent({
    required this.pointList, 
    required this.map
  });

  void updatePoints(List<Point3D> newPoints) {
    pointList = newPoints;
    _transformPoints();
  }

  void _transformPoints() {
    _transformedPoints.clear();
    
    if (pointList.isEmpty) return;

    for (Point3D point in pointList) {
      vm.Vector2 mapPoint = map.xy2idx(vm.Vector2(point.x, point.y));
      
      if (mapPoint.x.isFinite && mapPoint.y.isFinite) {
        _transformedPoints.add(vm.Vector2(mapPoint.x, mapPoint.y));
      }
    }
  }

  bool get hasLayout => true;

  @override
  void onMount() {
    super.onMount();
    _transformPoints();
  }

  @override
  void render(Canvas canvas) {
    if (_transformedPoints.isEmpty) return;

    Paint paint = Paint()
      ..color = Colors.orange
      ..strokeCap = StrokeCap.round
      ..strokeWidth = 1;

    List<Offset> offsetPoints = _transformedPoints
        .map((v) => Offset(v.x, v.y))
        .toList();

    canvas.drawPoints(PointMode.points, offsetPoints, paint);
  }
}
