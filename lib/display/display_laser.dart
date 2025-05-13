import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayLaser extends StatefulWidget {
  List<Offset> pointList = [];

  DisplayLaser({required this.pointList});

  @override
  DisplayLaserState createState() => DisplayLaserState();
}

class DisplayLaserState extends State<DisplayLaser> {
  @override
  Widget build(BuildContext context) {
    if (widget.pointList.isEmpty) {
      return Container(); // 如果没有点，返回空容器
    }

    //计算宽高
    double width = 0;
    double height = 0;
    for (var point in widget.pointList) {
      if (point.dx.isFinite && point.dx > width) {
        width = point.dx;
      }
      if (point.dy.isFinite && point.dy > height) {
        height = point.dy;
      }
    }

    // 确保有最小尺寸
    width = width.clamp(1.0, double.infinity);
    height = height.clamp(1.0, double.infinity);

    return RepaintBoundary(
      child: Container(
        width: width,
        height: height,
        child: CustomPaint(
          painter: DisplayLaserPainter(
            pointList: widget.pointList
                .where((point) => point.dx.isFinite && point.dy.isFinite)
                .toList(),
          ),
        ),
      ),
    );
  }
}

class DisplayLaserPainter extends CustomPainter {
  final List<Offset> pointList;
  final Paint _paint = Paint()
    ..color = Colors.lightGreen
    ..strokeCap = StrokeCap.round
    ..strokeWidth = 3;

  DisplayLaserPainter({required this.pointList});

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawPoints(PointMode.points, pointList, _paint);
  }

  @override
  bool shouldRepaint(DisplayLaserPainter oldDelegate) {
    return oldDelegate.pointList != pointList;
  }
}
