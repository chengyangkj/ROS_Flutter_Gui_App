import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flame/components.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class GridComponent extends RectangleComponent with HasGameRef {
  final RosChannel? rosChannel;
  bool _isDarkMode = true;

  GridComponent({required Vector2 size, this.rosChannel})
      : super(size: size, paint: Paint()..color = const Color(0xFF2C2C2C));

  void updateThemeMode(bool isDarkMode) {
    _isDarkMode = isDarkMode;
    if (_isDarkMode) {
      paint = Paint()..color = const Color(0xFF2C2C2C);
    } else {
      paint = Paint()..color = Colors.white;
    }
  }

  @override
  void render(Canvas canvas) {
    _renderGrid(canvas);
  }

  void _renderGrid(Canvas canvas) {
    double gridStepPixels = 100.0;
    if (rosChannel != null && rosChannel!.map_.value.mapConfig.resolution > 0) {
      gridStepPixels = 1.0 / rosChannel!.map_.value.mapConfig.resolution;
    }

    final paint = Paint()
      ..color = _isDarkMode ? Colors.white.withOpacity(0.2) : Colors.black.withOpacity(0.3)
      ..strokeWidth = 0.5
      ..style = PaintingStyle.stroke;

    final canvasSize = size;
    canvas.save();
    for (double x = 0; x <= canvasSize.x; x += gridStepPixels) {
      canvas.drawLine(Offset(x, 0), Offset(x, canvasSize.y), paint);
    }
    for (double y = 0; y <= canvasSize.y; y += gridStepPixels) {
      canvas.drawLine(Offset(0, y), Offset(canvasSize.x, y), paint);
    }
    canvas.restore();
  }

  @override
  bool get debugMode => false;
}
