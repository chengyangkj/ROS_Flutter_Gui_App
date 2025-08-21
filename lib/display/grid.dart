import 'dart:ui' as ui;
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:flame/components.dart';
class DisplayGrid extends RectangleComponent with HasGameRef {
  final RosChannel? rosChannel;
  
  DisplayGrid({required Vector2 size, this.rosChannel}) : super(
    size: size,
    paint: Paint()..color = const Color(0xFF2C2C2C), // 深灰色背景
  );
  
  @override
  void render(Canvas canvas) {

    // 绘制网格
    _renderGrid(canvas);
  }
  
  void _renderGrid(Canvas canvas) {
    // 获取地图分辨率，计算1米对应的像素数
    double gridStepPixels = 100.0; // 默认值
    
    if (rosChannel != null && rosChannel!.map_.value.mapConfig.resolution > 0) {
      // 1米 / 分辨率(米/像素) = 像素数
      gridStepPixels = 1.0 / rosChannel!.map_.value.mapConfig.resolution;
    }
    
    // 网格线画笔
    final paint = Paint()
      ..color = Colors.white.withOpacity(0.2)
      ..strokeWidth = 0.5
      ..style = PaintingStyle.stroke;
    
    // 计算需要绘制的网格范围（基于画布大小，不考虑相机）
    final canvasSize = size;
    
    // 绘制垂直线（每1米一条）
    for (double x = 0; x <= canvasSize.x; x += gridStepPixels) {
      canvas.drawLine(
        Offset(x, 0),
        Offset(x, canvasSize.y),
        paint,
      );
    }
    
    // 绘制水平线（每1米一条）
    for (double y = 0; y <= canvasSize.y; y += gridStepPixels) {
      canvas.drawLine(
        Offset(0, y),
        Offset(canvasSize.x, y),
        paint,
      );
    }
    
  }
  
  @override
  bool get debugMode => false;
}