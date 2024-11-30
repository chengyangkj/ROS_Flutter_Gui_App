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
    //计算宽高
    int width = 0;
    int height = 0;
    widget.pointList.forEach((element) {
      if (element.dx > width) {
        width = element.dx.toInt();
      }
      if (element.dy > height) {
        height = element.dy.toInt();
      }
    });
    return RepaintBoundary(
        child: Container(
      width: width.toDouble(),
      height: height.toDouble(),
      child: CustomPaint(
        painter: DisplayLaserPainter(pointList: widget.pointList),
      ),
    ));
  }
}

class DisplayLaserPainter extends CustomPainter {
  final List<Offset> pointList;
  final Paint _paint = Paint()
    ..color = Colors.red
    ..strokeCap = StrokeCap.butt
    ..strokeWidth = 1;

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
