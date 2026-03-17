import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

double _deg2rad(double deg) => deg * pi / 180;

class RobotMarkerPainter extends CustomPainter {
  RobotMarkerPainter({
    required this.animationValue,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
    this.isEditMode = false,
  });

  final double animationValue;
  final Color color;
  final int count;
  final bool isEditMode;

  @override
  void paint(Canvas canvas, Size canvasSize) {
    final radius = min(canvasSize.width, canvasSize.height) / 2;
    canvas.save();
    canvas.translate(canvasSize.width / 2, canvasSize.height / 2);

    for (int i = count; i >= 0; i--) {
      final opacity = (1.0 - ((i + animationValue) / (count + 1)));
      final paint = Paint()
        ..color = color.withValues(alpha: opacity)
        ..style = PaintingStyle.fill;
      final rippleRadius = radius * ((i + animationValue) / (count + 1));
      canvas.drawCircle(Offset.zero, rippleRadius, paint);
    }

    final centerPulse = 1.0 + 0.1 * sin(animationValue * 4 * pi);
    final centerPaint = Paint()
      ..color = color.withValues(alpha: 0.8 + 0.2 * sin(animationValue * 2 * pi))
      ..style = PaintingStyle.fill;
    final centerRadius = radius / 3 * centerPulse;
    canvas.drawCircle(Offset.zero, centerRadius, centerPaint);

    final dirPainter = Paint()
      ..style = PaintingStyle.fill
      ..color = color.withValues(alpha: 0.6);
    final rect = Rect.fromCircle(center: Offset.zero, radius: radius);
    canvas.drawArc(rect, _deg2rad(-15), _deg2rad(30), true, dirPainter);

    if (isEditMode) {
      final ringPaint = Paint()
        ..color = color.withValues(alpha: 0.6)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5;
      canvas.drawCircle(Offset.zero, radius, ringPaint);
      final dotPaint = Paint()
        ..color = Colors.red
        ..style = PaintingStyle.fill;
      final dotOffset = Offset(radius * cos(0), radius * sin(0));
      canvas.drawCircle(dotOffset, 2, dotPaint);
    }

    canvas.restore();
  }

  @override
  bool shouldRepaint(RobotMarkerPainter oldDelegate) =>
      oldDelegate.animationValue != animationValue ||
      oldDelegate.isEditMode != isEditMode;
}

class RobotMarkerWidget extends StatefulWidget {
  const RobotMarkerWidget({
    super.key,
    required this.size,
    this.color = const Color(0xFF0080ff),
    this.isEditMode = false,
  });

  final double size;
  final Color color;
  final bool isEditMode;

  @override
  State<RobotMarkerWidget> createState() => _RobotMarkerWidgetState();
}

class _RobotMarkerWidgetState extends State<RobotMarkerWidget>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    )..repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: widget.size,
      height: widget.size,
      child: AnimatedBuilder(
        animation: _controller,
        builder: (context, child) => CustomPaint(
          size: Size(widget.size, widget.size),
          painter: RobotMarkerPainter(
            animationValue: _controller.value,
            color: widget.color,
            isEditMode: widget.isEditMode,
          ),
        ),
      ),
    );
  }
}

Widget buildRobotMarkerLayer(
  RosChannel rosChannel,
  WorldToLatLngFn worldToLatLng, {
  bool isEditMode = false,
}) {
  final robotPose = rosChannel.robotPoseMap.value;
  final size = globalSetting.robotSize.toDouble();
  return MarkerLayer(
    markers: [
      Marker(
        point: worldToLatLng(robotPose.x, robotPose.y),
        width: size * 5,
        height: size * 5,
        alignment: Alignment.center,
        child: Transform.rotate(
          angle: -robotPose.theta,
          child: RobotMarkerWidget(size: size, isEditMode: isEditMode),
        ),
      ),
    ],
  );
}
