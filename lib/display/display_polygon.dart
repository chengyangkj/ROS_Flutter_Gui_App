import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayPolygon extends CustomPainter {
  List<Offset> pointList = [];
  Color color = Colors.green;
  bool fill = false; // 是否填充多边形
  
  DisplayPolygon({
    required this.pointList, 
    required this.color,
    this.fill = false,
  });
  
  @override
  void paint(Canvas canvas, Size size) {
    if (pointList.length < 3) return; // 至少需要3个点才能形成多边形
    
    Paint paint = Paint()
      ..color = color
      ..strokeWidth = 2
      ..style = fill ? PaintingStyle.fill : PaintingStyle.stroke; // 根据fill参数决定是否填充
    
    // 创建路径
    Path path = Path();
    
    // 移动到第一个点
    path.moveTo(pointList[0].dx, pointList[0].dy);
    
    // 连接所有点
    for (int i = 1; i < pointList.length; i++) {
      path.lineTo(pointList[i].dx, pointList[i].dy);
    }
    
    // 闭合路径
    path.close();
    
    // 绘制多边形
    canvas.drawPath(path, paint);
    
    // 如果需要填充，还可以绘制边框
    if (fill) {
      Paint borderPaint = Paint()
        ..color = color
        ..strokeWidth = 1
        ..style = PaintingStyle.stroke;
      canvas.drawPath(path, borderPaint);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
