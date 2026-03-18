import 'dart:math';

import 'package:flutter/material.dart';

enum PoseMarkerType {
  Robot,
  NavPoint,
}

double _deg2rad(double deg) => deg * pi / 180;

void _drawCenteredIcon(
  Canvas canvas,
  IconData icon, {
  required Color color,
  required double size,
}) {
  final textPainter = TextPainter(
    text: TextSpan(
      text: String.fromCharCode(icon.codePoint),
      style: TextStyle(
        fontSize: size,
        fontFamily: icon.fontFamily,
        package: icon.fontPackage,
        color: color,
      ),
    ),
    textDirection: TextDirection.ltr,
  )..layout();
  textPainter.paint(
    canvas,
    Offset(-textPainter.width / 2, -textPainter.height / 2),
  );
}

void _drawDirectionTriangle(
  Canvas canvas, {
  required double radius,
  required double theta,
  required Color color,
}) {
  final dir = Offset(cos(theta), sin(theta));
  final normal = Offset(-sin(theta), cos(theta));

  final tangentPoint = dir * radius;
  const halfBase = 10.0;
  const height = 8.0;
  final p1 = tangentPoint + normal * halfBase;
  final p2 = tangentPoint - normal * halfBase;
  final tip = tangentPoint + dir * height;

  final path = Path()
    ..moveTo(tip.dx, tip.dy)
    ..lineTo(p1.dx, p1.dy)
    ..lineTo(p2.dx, p2.dy)
    ..close();

  final paint = Paint()
    ..color = color
    ..style = PaintingStyle.fill;
  canvas.drawPath(path, paint);
}

class PoseMarkerPainter extends CustomPainter {
  PoseMarkerPainter({
    required this.animationValue,
    required this.theta,
    required this.type,
    required this.isEditMode,
    required this.color,
    required this.count,
  });

  final double animationValue;
  final double theta;
  final PoseMarkerType type;
  final bool isEditMode;
  final Color color;
  final int count;

  @override
  void paint(Canvas canvas, Size canvasSize) {
    final radius = min(canvasSize.width, canvasSize.height) / 2;
    canvas.save();
    canvas.translate(canvasSize.width / 2, canvasSize.height / 2);

    switch (type) {
      case PoseMarkerType.Robot:
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
          ..color = color.withValues(
            alpha: 0.8 + 0.2 * sin(animationValue * 2 * pi),
          )
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
        break;
      case PoseMarkerType.NavPoint:
        _drawCenteredIcon(
          canvas,
          Icons.place,
          color: color,
          size: radius * 2.0,
        );
        break;
    }

    if (type == PoseMarkerType.NavPoint) {
      if (isEditMode) {
        final ringPaint = Paint()
          ..color = Colors.green.withValues(alpha: 0.55)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.0;
        canvas.drawCircle(Offset.zero, radius, ringPaint);
      }
      _drawDirectionTriangle(
        canvas,
        radius: radius,
        theta: theta,
        color: Colors.blue,
      );
    } else if (isEditMode) {
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
  bool shouldRepaint(covariant PoseMarkerPainter oldDelegate) {
    return oldDelegate.animationValue != animationValue ||
        oldDelegate.theta != theta ||
        oldDelegate.isEditMode != isEditMode ||
        oldDelegate.type != type ||
        oldDelegate.color != color ||
        oldDelegate.count != count;
  }
}

class PoseMarkerWidget extends StatefulWidget {
  const PoseMarkerWidget({
    super.key,
    required this.size,
    required this.theta,
    required this.type,
    this.onThetaChanged,
    this.onThetaEnd,
    this.onMoveDelta,
    this.onMoveEnd,
    this.color = const Color(0xFF0080ff),
    this.count = 2,
    this.isEditMode = false,
  });

  final double size;
  final double theta;
  final PoseMarkerType type;
  final ValueChanged<double>? onThetaChanged;
  final ValueChanged<double>? onThetaEnd;
  final ValueChanged<Offset>? onMoveDelta;
  final VoidCallback? onMoveEnd;
  final Color color;
  final int count;
  final bool isEditMode;

  @override
  State<PoseMarkerWidget> createState() => _PoseMarkerWidgetState();
}

class _PoseMarkerWidgetState extends State<PoseMarkerWidget>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  late double _theta;
  bool _isRotating = false;
  bool _isMoving = false;

  bool _hitTestMoveArea(Offset localPosition) {
    final center = Offset(widget.size / 2, widget.size / 2);
    final dist = (localPosition - center).distance;
    return dist <= widget.size * 0.5;
  }

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
  void didUpdateWidget(covariant PoseMarkerWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!_isRotating && !_isMoving && oldWidget.theta != widget.theta) {
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
    widget.onThetaChanged?.call(_theta);
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: widget.size,
      height: widget.size,
      child: GestureDetector(
        behavior: HitTestBehavior.opaque,
        onPanStart: widget.isEditMode
            ? (details) {
                if (_hitTestMoveArea(details.localPosition)) {
                  _isMoving = true;
                  _isRotating = false;
                } else {
                  _isRotating = true;
                  _isMoving = false;
                  _updateThetaFromPosition(details.localPosition);
                }
              }
            : null,
        onPanUpdate: widget.isEditMode
            ? (details) {
                if (_isMoving) {
                  widget.onMoveDelta?.call(details.delta);
                  return;
                }
                if (_isRotating) {
                  _updateThetaFromPosition(details.localPosition);
                }
              }
            : null,
        onPanEnd: widget.isEditMode
            ? (_) {
                _isRotating = false;
                _isMoving = false;
              widget.onThetaEnd?.call(_theta);
              widget.onMoveEnd?.call();
              }
            : null,
        onPanCancel: widget.isEditMode
            ? () {
                _isRotating = false;
                _isMoving = false;
                widget.onThetaEnd?.call(_theta);
                widget.onMoveEnd?.call();
              }
            : null,
        child: AnimatedBuilder(
          animation: _controller,
          builder: (context, child) => CustomPaint(
            size: Size(widget.size, widget.size),
            painter: PoseMarkerPainter(
              animationValue: _controller.value,
              theta: _theta,
              type: widget.type,
              isEditMode: widget.isEditMode,
              color: widget.color,
              count: widget.count,
            ),
          ),
        ),
      ),
    );
  }
}

