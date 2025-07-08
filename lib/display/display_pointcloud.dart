import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/pointcloud2.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';

class DisplayPointCloud extends StatefulWidget {
  List<Point3D> pointList = [];
  OccupancyMap map;

  DisplayPointCloud({required this.pointList, required this.map});

  @override
  DisplayPointCloudState createState() => DisplayPointCloudState();
}

class DisplayPointCloudState extends State<DisplayPointCloud> {
  @override
  Widget build(BuildContext context) {
    if (widget.pointList.isEmpty) {
      return Container(); // 如果没有点，返回空容器
    }

    // 将3D点转换为2D点（投影到XY平面）
    List<Offset> point2DList = [];
    for (Point3D point in widget.pointList) {
      // 将世界坐标转换为地图坐标
      Offset mapPoint = widget.map.xy2idx(Offset(point.x, point.y));
      point2DList.add(mapPoint);
    }

    // 过滤无效点
    point2DList = point2DList
        .where((point) => point.dx.isFinite && point.dy.isFinite)
        .toList();

    if (point2DList.isEmpty) {
      return Container();
    }

    //计算宽高
    double width = 0;
    double height = 0;
    for (var point in point2DList) {
      if (point.dx.isFinite && point.dx > width) {
        width = point.dx;
      }
      if (point.dy.isFinite && point.dy > height) {
        height = point.dy;
      }
    }

    // 确保有最小尺寸
    width = width.clamp(1.0, double.infinity);
    height = height.clamp(1.0, double.infinity);

    return RepaintBoundary(
      child: Container(
        width: width,
        height: height,
        child: CustomPaint(
          painter: DisplayPointCloudPainter(
            pointList: point2DList,
          ),
        ),
      ),
    );
  }
}

class DisplayPointCloudPainter extends CustomPainter {
  final List<Offset> pointList;
  final Paint _paint = Paint()
    ..color = Colors.blue
    ..strokeCap = StrokeCap.round
    ..strokeWidth = 2;

  DisplayPointCloudPainter({required this.pointList});

  @override
  void paint(Canvas canvas, Size size) {
    // 绘制点云
    canvas.drawPoints(PointMode.points, pointList, _paint);
  }

  @override
  bool shouldRepaint(DisplayPointCloudPainter oldDelegate) {
    return oldDelegate.pointList != pointList;
  }
} 