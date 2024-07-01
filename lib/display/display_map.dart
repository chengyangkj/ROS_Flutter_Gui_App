import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayMap extends CustomPainter {
  late OccupancyMap map;
  List<Offset> occPointList = [];
  List<Offset> freePointList = [];
  DisplayMap({required this.map}) {
    for (int i = 0; i < map.Cols(); i++)
      // ignore: curly_braces_in_flow_control_structures
      for (int j = 0; j < map.Rows(); j++) {
        int mapValue = map.data[j][i];
        if (mapValue > 0) {
          Offset point = Offset(i.toDouble(), j.toDouble());
          occPointList.add(point);
        } else if (mapValue == 0) {
          Offset point = Offset(i.toDouble(), j.toDouble());
          freePointList.add(point);
        }
      }
  }
  @override
  void paint(Canvas canvas, Size size) {
    Paint paint = Paint()
      ..color = Colors.lightBlue[600]! // 设置颜色为传入的颜色参数
      ..strokeCap = StrokeCap.butt // 设置画笔的端点为圆形，以便绘制圆形点
      ..strokeWidth = 1; // 设置画笔的宽度为1个像素
    canvas.drawPoints(PointMode.points, occPointList, paint);

    paint.color = Colors.blue[100]!;
    canvas.drawPoints(PointMode.points, freePointList, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) =>
      this != oldDelegate;
}
