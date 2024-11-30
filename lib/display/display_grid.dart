import 'package:flutter/material.dart';

class DisplayGrid extends StatelessWidget {
  DisplayGrid({required this.step, required this.width, required this.height});
  double step = 20;
  double width = 300;
  double height = 300;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;
    return RepaintBoundary(
      child: Container(
        width: width,
        height: height,
        color: theme.scaffoldBackgroundColor,
        child: CustomPaint(
          isComplex: true,
          willChange: false,
          painter: GridPainter(
            step: step,
            color: isDarkMode
                ? Colors.white.withAlpha(60)
                : Colors.black.withAlpha(60),
          ),
        ),
      ),
    );
  }
}

class GridPainter extends CustomPainter {
  late double step;
  Color color;

  GridPainter({required this.step, required this.color});

  @override
  void paint(Canvas canvas, Size size) {
    final Paint paint = Paint()
      ..color = color
      ..strokeWidth = 2.0
      ..style = PaintingStyle.fill;

    for (double x = 0; x <= size.width; x += step) {
      for (double y = 0; y <= size.height; y += step) {
        canvas.drawCircle(Offset(x, y), 1.0, paint);
      }
    }
  }

  @override
  bool shouldRepaint(GridPainter oldDelegate) {
    return step != oldDelegate.step || color != oldDelegate.color;
  }
}
