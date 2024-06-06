import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';

class DisplayRobotReloc extends StatefulWidget {
  late double size;
  bool relocMode = false;
  void Function(double) onRotateCallback;
  late Color color = const Color(0xFF0080ff);
  DisplayRobotReloc(
      {required this.size,
      required this.relocMode,
      required this.onRotateCallback});

  @override
  _DisplayRobotRelocState createState() => _DisplayRobotRelocState();
}

class _DisplayRobotRelocState extends State<DisplayRobotReloc>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  double _angle = 0.0;
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
  void _updatePosition(Offset localPosition) {
    setState(() {
      _angle = atan2(localPosition.dy - widget.size / 2,
          localPosition.dx - widget.size / 2);
      widget.onRotateCallback(_angle);
    });
  }

  @override
  Widget build(BuildContext context) {
    if (!widget.relocMode) {
      _angle = 0;
    }
    return GestureDetector(
        onPanUpdate: (details) {
          _updatePosition(details.localPosition);
        },
        child: Container(
          width: widget.size,
          height: widget.size,
          child: AnimatedBuilder(
            animation: _controller,
            builder: (context, child) {
              return CustomPaint(
                painter: DisplayRobotRelocPainter(
                    angle: _angle, relocMode: widget.relocMode),
              );
            },
          ),
        ));
  }
}

class DisplayRobotRelocPainter extends CustomPainter {
  bool relocMode = false;
  Paint _paint = Paint()..style = PaintingStyle.fill;
  double angle = 0;
  DisplayRobotRelocPainter({this.angle = 0, this.relocMode = false});

  @override
  void paint(Canvas canvas, Size size) {
    if (!relocMode) return;
    // //绘制机器人坐标
    Offset center = Offset(size.width / 2, size.height / 2);
    final circlePaint = Paint()
      ..color = Colors.blue
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;

    final pointPaint = Paint()
      ..color = Colors.red
      ..style = PaintingStyle.fill;

    final radius = size.width / 2 - 3;

    // 绘制主圆圈
    canvas.drawCircle(center, radius, circlePaint);

    // 计算并绘制圆点位置
    final pointOffset = Offset(
      center.dx + radius * cos(0),
      center.dy + radius * sin(0),
    );
    canvas.drawCircle(pointOffset, 2, pointPaint);

    Paint paint = Paint()
      ..color = Colors.red // 设置颜色为传入的颜色参数
      ..strokeCap = StrokeCap.butt // 设置画笔的端点为圆形，以便绘制圆形点
      ..strokeWidth = 2; // 设置画笔的宽度为1个像素
    canvas.drawPoints(PointMode.points, [Offset(0, 0), center], paint);
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return true;
  }
}
