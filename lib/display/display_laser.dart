import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayLaser extends CustomPainter {
  List<Offset> pointList = [];
  DisplayLaser({required this.pointList});
  @override
  void paint(Canvas canvas, Size size) {
    Paint paint = Paint()
      ..color = Colors.red // 设置颜色为传入的颜色参数
      ..strokeCap = StrokeCap.butt // 设置画笔的端点为圆形，以便绘制圆形点
      ..strokeWidth = 1; // 设置画笔的宽度为1个像素
    canvas.drawPoints(PointMode.points, pointList, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) =>
      this != oldDelegate;
}
