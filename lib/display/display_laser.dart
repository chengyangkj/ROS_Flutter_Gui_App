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

class DisplayLaserState extends State<DisplayLaser>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    _controller =
        AnimationController(vsync: this, duration: Duration(milliseconds: 2000))
          ..repeat();
    super.initState();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  Offset updateMoveOffset = Offset(0, 0);
  Offset startMoveOffset = Offset(0, 0);
  Offset endMoveOffset = Offset(0, 0);
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
    return RepaintBoundary(child: Container(
        width: width.toDouble(),
        height: height.toDouble(),
        // color: Colors.red,
        child: AnimatedBuilder(
          animation: _controller,
          builder: (context, child) {
            return CustomPaint(
              painter: DisplayLaserPainter(pointList: widget.pointList),
            );
          },
        ),
      ) ) ;
  }
}

class DisplayLaserPainter extends CustomPainter {
  List<Offset> pointList = [];

  Paint _paint = Paint()..style = PaintingStyle.fill;

  DisplayLaserPainter({required this.pointList});

  @override
  void paint(Canvas canvas, Size size) {
    Paint paint = Paint()
      ..color = Colors.red // 设置颜色为传入的颜色参数
      ..strokeCap = StrokeCap.butt // 设置画笔的端点为圆形，以便绘制圆形点
      ..strokeWidth = 1; // 设置画笔的宽度为1个像素
    canvas.drawPoints(PointMode.points, pointList, paint);
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return true;
  }
}
