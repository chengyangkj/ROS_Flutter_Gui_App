import 'dart:ui';
import 'package:flame/components.dart';
import 'package:flutter/material.dart';

class LaserComponent extends Component {
  List<Vector2> pointList = [];
  
  LaserComponent({required this.pointList});

  void updateLaser(List<Vector2> newPoints) {
    pointList = newPoints.where((point) => 
        point.x.isFinite && point.y.isFinite).toList();
  }

  bool get hasLayout => true;

  @override
  void render(Canvas canvas) {
    if (pointList.isEmpty) return;

    Paint paint = Paint()
      ..color = Colors.lightGreen
      ..strokeCap = StrokeCap.round
      ..strokeWidth = 3;

    List<Offset> offsetPoints = pointList
        .map((v) => Offset(v.x, v.y))
        .toList();

    canvas.drawPoints(PointMode.points, offsetPoints, paint);
  }
}
