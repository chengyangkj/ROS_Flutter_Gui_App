import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';

class DisplayPoseDirection extends StatefulWidget {
  late double size;
  bool resetAngle = false;
  double initAngle = 0;
  void Function(double) onRotateCallback;
  late Color color = const Color(0xFF0080ff);
  DisplayPoseDirection(
      {required this.size,
      required this.resetAngle,
      required this.onRotateCallback,
      this.initAngle = 0});

  @override
  _DisplayPoseDirectionState createState() => _DisplayPoseDirectionState();
}

class _DisplayPoseDirectionState extends State<DisplayPoseDirection>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  double _angle = 0;

  @override
  void initState() {
    _controller =
        AnimationController(vsync: this, duration: Duration(milliseconds: 2000))
          ..repeat();
    super.initState();
    _angle = widget.initAngle;
    if (widget.resetAngle) {
      _angle = 0;
    }
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
    return GestureDetector(
        onPanUpdate: (details) {
          _updatePosition(details.localPosition);
        },
        child: Container(
          width: widget.size,
          height: widget.size,
          decoration: BoxDecoration(
              border: Border.all(
            color: Colors.red, // 红色边框
            width: 1, // 边框宽度
          )),
          child: AnimatedBuilder(
            animation: _controller,
            builder: (context, child) {
              return CustomPaint(
                painter: DisplayPoseDirectionPainter(angle: _angle),
              );
            },
          ),
        ));
  }
}

class DisplayPoseDirectionPainter extends CustomPainter {
  Paint _paint = Paint()..style = PaintingStyle.fill;
  double angle = 0;
  DisplayPoseDirectionPainter({this.angle = 0});

  @override
  void paint(Canvas canvas, Size size) {
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
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return true;
  }
}
