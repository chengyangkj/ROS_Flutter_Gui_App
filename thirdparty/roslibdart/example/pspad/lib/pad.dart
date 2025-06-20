import 'package:flutter/material.dart';
import 'dart:math';

class TrianglePainter extends CustomPainter {
  double sideSize;

  Color color;
  TrianglePainter({required this.sideSize, required this.color});

  @override
  void paint(Canvas canvas, Size size) {
    double ySize = sideSize * cos(30 * pi / 180);
    double xSize = sideSize;

    double point0x = xSize / 2;
    double point0y = ySize / 2;

    double point1x = -xSize / 2;
    double point1y = ySize / 2;

    double point2x = 0;
    double point2y = -ySize / 2;

    Path path = Path();
    path.moveTo(point0x, point0y);
    path.lineTo(point1x, point1y);
    path.lineTo(point2x, point2y);
    path.lineTo(point0x, point0y);
    path.close();
    canvas.drawPath(
        path,
        Paint()
          ..color = color
          ..strokeWidth = 4
          ..style = PaintingStyle.stroke);

    canvas.save();
    canvas.restore();
  }

  @override
  bool shouldRepaint(TrianglePainter oldDelegate) {
    return oldDelegate.color != color || oldDelegate.sideSize != sideSize;
  }
}

Widget triangle(double sideSize, Color color, double angle) {
  return Transform.rotate(
      child: CustomPaint(
        painter: TrianglePainter(
          color: color,
          sideSize: sideSize,
        ),
      ),
      angle: angle * pi / 180);
}

class SquarePainter extends CustomPainter {
  double xSize, ySize;

  Color color;
  SquarePainter(
      {required this.xSize, required this.ySize, required this.color});

  @override
  void paint(Canvas canvas, Size size) {
    double point0x = xSize / 2;
    double point0y = ySize / 2;

    double point1x = -xSize / 2;
    double point1y = ySize / 2;

    double point2x = -xSize / 2;
    double point2y = -ySize / 2;

    double point3x = xSize / 2;
    double point3y = -ySize / 2;

    Path path = Path();
    path.moveTo(point0x, point0y);
    path.lineTo(point1x, point1y);
    path.lineTo(point2x, point2y);
    path.lineTo(point3x, point3y);
    path.lineTo(point0x, point0y);
    path.close();
    canvas.drawPath(
        path,
        Paint()
          ..color = color
          ..strokeWidth = 4
          ..style = PaintingStyle.stroke);

    canvas.save();
    canvas.restore();
  }

  @override
  bool shouldRepaint(SquarePainter oldDelegate) {
    return oldDelegate.color != color ||
        oldDelegate.xSize != xSize ||
        oldDelegate.ySize != ySize;
  }
}

Widget square(double size, Color color) {
  return CustomPaint(
      painter: SquarePainter(
    color: color,
    xSize: size,
    ySize: size,
  ));
}

Widget circle(double diameter, Color? color) {
  return Container(
    width: diameter,
    height: diameter,
    decoration: BoxDecoration(
      color: color,
      shape: BoxShape.circle,
    ),
  );
}

Widget hollowCircle(double diameter, Color color) {
  return Container(
    width: diameter,
    height: diameter,
    decoration: BoxDecoration(
      shape: BoxShape.circle,
      border: Border.all(
        width: 4,
        color: color,
      ),
    ),
  );
}

class CrossPainter extends CustomPainter {
  double xSize, ySize;

  Color color;
  CrossPainter({required this.xSize, required this.ySize, required this.color});

  @override
  void paint(Canvas canvas, Size size) {
    double point0x = xSize / 2;
    double point0y = ySize / 2;

    double point1x = -xSize / 2;
    double point1y = ySize / 2;

    double point2x = -xSize / 2;
    double point2y = -ySize / 2;

    double point3x = xSize / 2;
    double point3y = -ySize / 2;

    final paint = Paint()
      ..color = color
      ..strokeWidth = 4;
    canvas.drawLine(Offset(point0x, point0y), Offset(point2x, point2y), paint);
    canvas.drawLine(Offset(point1x, point1y), Offset(point3x, point3y), paint);
  }

  @override
  bool shouldRepaint(CrossPainter oldDelegate) {
    return oldDelegate.color != color ||
        oldDelegate.xSize != xSize ||
        oldDelegate.ySize != ySize;
  }
}

Widget cross(double size, Color color) {
  return CustomPaint(
      painter: CrossPainter(
    color: color,
    xSize: size,
    ySize: size,
  ));
}

class DirectionPad extends StatefulWidget {
  final double diameter;
  final Function leftCallback, rightCallback, forwardCallback, backwardCallback;
  const DirectionPad(
      {Key? key,
      required this.diameter,
      required this.leftCallback,
      required this.rightCallback,
      required this.forwardCallback,
      required this.backwardCallback})
      : super(key: key);

  @override
  State<DirectionPad> createState() => _DirectionPadState();
}

class _DirectionPadState extends State<DirectionPad> {
  Widget horizontalBar(double longSize, double shortSize, Color color) {
    return Container(
        width: longSize,
        height: shortSize,
        decoration: BoxDecoration(
            color: color,
            border: Border.all(
              color: color,
            ),
            borderRadius: const BorderRadius.all(Radius.circular(5))));
  }

  @override
  Widget build(BuildContext context) {
    double diameter = widget.diameter;
    double circleDiameter = diameter;
    double keyLongSize = diameter * 0.8;
    double keyShortSize = diameter * 0.25;
    double longMargin = (circleDiameter - keyLongSize) / 2;
    double shortMargin = (circleDiameter - keyShortSize) / 2;
    double triangleSize = diameter * 0.175;
    Color callbackColor = const Color(0x00000000).withOpacity(0.0);
    return Stack(
      children: [
        circle(circleDiameter, Colors.grey[300]),
        Positioned(
            left: longMargin,
            top: shortMargin,
            child: horizontalBar(keyLongSize, keyShortSize, Colors.grey[600]!)),
        Positioned(
            left: shortMargin,
            top: longMargin,
            child: horizontalBar(keyShortSize, keyLongSize, Colors.grey[600]!)),
        // Left Button
        Positioned(
            left: longMargin + keyLongSize / 6,
            top: shortMargin + keyShortSize / 2,
            child: triangle(triangleSize, Colors.grey[800]!, -90.0)),
        Positioned(
            left: longMargin + keyLongSize / 6 - triangleSize / 2,
            top: shortMargin + keyShortSize / 2 - triangleSize / 2,
            child: Listener(
                onPointerDown: (event) {
                  widget.leftCallback(-1);
                },
                onPointerUp: (event) {
                  widget.leftCallback(1);
                },
                child: Container(
                  width: triangleSize,
                  height: triangleSize,
                  color: callbackColor,
                ))),
        // Forward Button
        Positioned(
            left: diameter / 2,
            top: longMargin + keyLongSize / 6,
            child: triangle(triangleSize, Colors.grey[800]!, 0.0)),
        Positioned(
            left: diameter / 2 - triangleSize / 2,
            top: longMargin + keyLongSize / 6 - triangleSize / 2,
            child: Listener(
                onPointerDown: (event) {
                  widget.forwardCallback(-1);
                },
                onPointerUp: (event) {
                  widget.forwardCallback(1);
                },
                child: Container(
                    width: triangleSize,
                    height: triangleSize,
                    color: callbackColor))),
        // Backward Button
        Positioned(
            left: diameter / 2,
            top: longMargin + keyLongSize * 2 / 3 + keyLongSize / 6,
            child: triangle(triangleSize, Colors.grey[800]!, 180.0)),
        Positioned(
            left: diameter / 2 - triangleSize / 2,
            top: longMargin +
                keyLongSize * 2 / 3 +
                keyLongSize / 6 -
                triangleSize / 2,
            child: Listener(
                onPointerDown: (event) {
                  widget.backwardCallback(-1);
                },
                onPointerUp: (event) {
                  widget.backwardCallback(1);
                },
                child: Container(
                  width: triangleSize,
                  height: triangleSize,
                  color: callbackColor,
                ))),
        // Right Button
        Positioned(
            left: longMargin + keyLongSize * 2 / 3 + keyLongSize / 6,
            top: shortMargin + keyShortSize / 2,
            child: triangle(triangleSize, Colors.grey[800]!, 90.0)),
        Positioned(
            left: longMargin +
                keyLongSize * 2 / 3 +
                keyLongSize / 6 -
                triangleSize / 2,
            top: shortMargin + keyShortSize / 2 - triangleSize / 2,
            child: Listener(
                onPointerDown: (event) {
                  widget.rightCallback(-1);
                },
                onPointerUp: (event) {
                  widget.rightCallback(1);
                },
                child: Container(
                  width: triangleSize,
                  height: triangleSize,
                  color: callbackColor,
                ))),
      ],
    );
  }
}

class KeyPad extends StatefulWidget {
  final double diameter;
  final Function squareCallback,
      circleCallback,
      triangleCallback,
      crossCallback;
  const KeyPad(
      {Key? key,
      required this.diameter,
      required this.squareCallback,
      required this.circleCallback,
      required this.triangleCallback,
      required this.crossCallback})
      : super(key: key);

  @override
  State<KeyPad> createState() => _KeyPadState();
}

class _KeyPadState extends State<KeyPad> {
  @override
  Widget build(BuildContext context) {
    double diameter = widget.diameter;
    double circleDiameter = diameter;
    double shortMargin = diameter * 0.225;
    double keySize = diameter * 0.275;
    double iconSize = diameter * 0.125;
    double hollowCircleDiameter = iconSize + 8;
    Color callbackColor = const Color(0x00000000).withOpacity(0.0);
    return Stack(children: [
      Positioned(child: circle(circleDiameter, Colors.grey[300]!)),
      Positioned(
          left: shortMargin - keySize / 2,
          top: circleDiameter / 2 - keySize / 2,
          child: circle(keySize, Colors.grey[700]!)),
      Positioned(
          left: circleDiameter - shortMargin - keySize / 2,
          top: circleDiameter / 2 - keySize / 2,
          child: circle(keySize, Colors.grey[700]!)),
      Positioned(
          left: circleDiameter / 2 - keySize / 2,
          top: circleDiameter - shortMargin - keySize / 2,
          child: circle(keySize, Colors.grey[700]!)),
      Positioned(
          left: circleDiameter / 2 - keySize / 2,
          top: shortMargin - keySize / 2,
          child: circle(keySize, Colors.grey[700]!)),
      Positioned(
          left: circleDiameter / 2,
          top: shortMargin - 4,
          child: triangle(iconSize, Colors.green[500]!, 0)),
      Positioned(
          left: circleDiameter / 2 - keySize / 2,
          top: shortMargin - keySize / 2,
          child: Listener(
              onPointerDown: (event) {
                widget.triangleCallback(-1);
              },
              onPointerUp: (event) {
                widget.triangleCallback(1);
              },
              child: Container(
                width: keySize,
                height: keySize,
                color: callbackColor,
              ))),
      Positioned(
          left: circleDiameter / 2,
          top: circleDiameter - shortMargin,
          child: cross(iconSize, Colors.cyan[500]!)),
      Positioned(
          left: circleDiameter / 2 - keySize / 2,
          top: circleDiameter - shortMargin - keySize / 2,
          child: Listener(
              onPointerDown: (event) {
                widget.crossCallback(-1);
              },
              onPointerUp: (event) {
                widget.crossCallback(1);
              },
              child: Container(
                width: keySize,
                height: keySize,
                color: callbackColor,
              ))),
      Positioned(
          left: shortMargin,
          top: circleDiameter / 2,
          child: square(iconSize, Colors.pink[500]!)),
      Positioned(
          left: shortMargin - keySize / 2,
          top: circleDiameter / 2 - keySize / 2,
          child: Listener(
              onPointerDown: (event) {
                widget.squareCallback(-1);
              },
              onPointerUp: (event) {
                widget.squareCallback(1);
              },
              child: Container(
                width: keySize,
                height: keySize,
                color: callbackColor,
              ))),
      Positioned(
          left: -hollowCircleDiameter / 2 + circleDiameter - shortMargin,
          top: -hollowCircleDiameter / 2 + circleDiameter / 2,
          child: hollowCircle(hollowCircleDiameter, Colors.orange[500]!)),
      Positioned(
          left: -hollowCircleDiameter / 2 + circleDiameter - keySize,
          top: circleDiameter / 2 - keySize / 2,
          child: Listener(
              onPointerDown: (event) {
                widget.circleCallback(-1);
              },
              onPointerUp: (event) {
                widget.circleCallback(1);
              },
              child: Container(
                width: keySize,
                height: keySize,
                color: callbackColor,
              ))),
    ]);
  }
}
