import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';

class DisplayMap extends CustomPainter {
  OccupancyMap map;
  List<Offset> pointList = [];
  DisplayMap({required this.map}) {
    for (int i = 0; i < map.Cols(); i++)
      // ignore: curly_braces_in_flow_control_structures
      for (int j = 0; j < map.Rows(); j++) {
        int mapValue = map.data[j][i];
        if (mapValue > 0) {
          Offset point = Offset(i.toDouble(), j.toDouble());
          pointList.add(point);
        }
      }
  }
  @override
  void paint(Canvas canvas, Size size) {
    Paint paint = Paint()
      ..color = Colors.black // 设置颜色为传入的颜色参数
      ..strokeCap = StrokeCap.round // 设置画笔的端点为圆形，以便绘制圆形点
      ..strokeWidth = 1; // 设置画笔的宽度为5个像素

    canvas.drawPoints(PointMode.points, pointList, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) =>
      this != oldDelegate;
}
