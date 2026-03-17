import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

double _deg2rad(double deg) => deg * pi / 180;

class RobotMarkerPainter extends CustomPainter {
  RobotMarkerPainter({
    required this.animationValue,
    required this.theta,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
    this.isEditMode = false,
  });

  final double animationValue;
  final double theta;
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

    canvas.save();
    canvas.rotate(theta);
    final dirPainter = Paint()
      ..style = PaintingStyle.fill
      ..color = color.withValues(alpha: 0.6);
    final rect = Rect.fromCircle(center: Offset.zero, radius: radius);
    canvas.drawArc(rect, _deg2rad(-15), _deg2rad(30), true, dirPainter);
    canvas.restore();

    if (isEditMode) {
      final ringPaint = Paint()
        ..color = color.withValues(alpha: 0.6)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 3.0;
      canvas.drawCircle(Offset.zero, radius, ringPaint);
      final dotPaint = Paint()
        ..color = Colors.red
        ..style = PaintingStyle.fill;
      final dotOffset = Offset(radius * cos(theta), radius * sin(theta));
      canvas.drawCircle(dotOffset, 7, dotPaint);
    }

    canvas.restore();
  }

  @override
  bool shouldRepaint(RobotMarkerPainter oldDelegate) =>
      oldDelegate.animationValue != animationValue ||
      oldDelegate.isEditMode != isEditMode ||
      oldDelegate.theta != theta;
}

class RobotMarkerWidget extends StatefulWidget {
  const RobotMarkerWidget({
    super.key,
    required this.size,
    required this.theta,
    this.onThetaChanged,
    this.color = const Color(0xFF0080ff),
    this.isEditMode = false,
  });

  final double size;
  final double theta;
  final ValueChanged<double>? onThetaChanged;
  final Color color;
  final bool isEditMode;

  @override
  State<RobotMarkerWidget> createState() => _RobotMarkerWidgetState();
}

class _RobotMarkerWidgetState extends State<RobotMarkerWidget>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  late double _theta;
  bool _isRotating = false;

  @override
  void initState() {
    super.initState();
    _theta = widget.theta;
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    )..repeat();
  }

  @override
  void didUpdateWidget(covariant RobotMarkerWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!_isRotating && oldWidget.theta != widget.theta) {
      _theta = widget.theta;
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  void _updateThetaFromPosition(Offset localPosition) {
    final center = Offset(widget.size / 2, widget.size / 2);
    final delta = localPosition - center;
    if (delta.distanceSquared <= 0.000001) return;
    final newTheta = atan2(delta.dy, delta.dx);
    if (newTheta == _theta) return;
    setState(() {
      _theta = newTheta;
    });
    final radius = widget.size / 2;
    final projected =
        center + Offset(radius * cos(_theta), radius * sin(_theta));

    widget.onThetaChanged?.call(_theta);
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: widget.size,
      height: widget.size,
      child: Listener(
        behavior: HitTestBehavior.opaque,
        onPointerDown: widget.isEditMode
            ? (event) {
                final box = context.findRenderObject() as RenderBox?;
                if (box == null) return;
                final local = box.globalToLocal(event.position);
                _isRotating = true;
                _updateThetaFromPosition(local);
              }
            : null,
        onPointerMove: widget.isEditMode
            ? (event) {
                if (!_isRotating) return;
                final box = context.findRenderObject() as RenderBox?;
                if (box == null) return;
                final local = box.globalToLocal(event.position);
                _updateThetaFromPosition(local);
              }
            : null,
        onPointerUp: widget.isEditMode
            ? (event) {
                // if (_isRotating) {
                //   debugPrint('RobotMarkerWidget rotate end global=${event.position}');
                // }
                _isRotating = false;
              }
            : null,
        onPointerCancel: widget.isEditMode
            ? (event) {
                // debugPrint('RobotMarkerWidget pointerCancel');
                _isRotating = false;
              }
            : null,
        child: AnimatedBuilder(
          animation: _controller,
          builder: (context, child) => CustomPaint(
            size: Size(widget.size, widget.size),
            painter: RobotMarkerPainter(
              animationValue: _controller.value,
              theta: _theta,
              color: widget.color,
              isEditMode: widget.isEditMode,
            ),
          ),
        ),
      ),
    );
  }
}

Widget buildRobotMarkerLayer(
  RosChannel rosChannel,
  WorldToLatLngFn worldToLatLng, {
  RobotPose? poseOverride,
  bool isEditMode = false,
  ValueChanged<double>? onThetaChanged,
}) {
  final robotPose = poseOverride ?? rosChannel.robotPoseMap.value;
  final robotSize = globalSetting.robotSize.toDouble();
  return MarkerLayer(
    markers: [
      Marker(
        point: worldToLatLng(robotPose.x, robotPose.y),
        width: robotSize,
        height: robotSize,
        alignment: Alignment.center,
        child: RobotMarkerWidget(
          size: robotSize,
          theta: robotPose.theta,
          isEditMode: isEditMode,
          onThetaChanged: onThetaChanged,
        ),
      ),
    ],
  );
}
