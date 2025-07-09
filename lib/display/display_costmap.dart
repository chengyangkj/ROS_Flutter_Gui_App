import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayCostMap extends StatefulWidget {
  final double opacity;
  final bool isGlobal; // 标识是全局还是局部代价地图

  const DisplayCostMap({
    super.key,
    this.opacity = 0.5,
    this.isGlobal = false, // 默认为局部代价地图
  });

  @override
  _DisplayCostMapState createState() => _DisplayCostMapState();
}

class _DisplayCostMapState extends State<DisplayCostMap> {
  List<CostMapPoint> costMapPoints = [];

  @override
  void initState() {
    super.initState();
  }

  void _processCostMapData(OccupancyMap costMap) {
    costMapPoints.clear();
    
    if (costMap.data.isEmpty || costMap.mapConfig.width == 0 || costMap.mapConfig.height == 0) {
      return;
    }

    // 获取代价地图的颜色数据
    List<int> costMapColors = costMap.getCostMapData();
    
    // 创建图像数据
    final int width = costMap.mapConfig.width;
    final int height = costMap.mapConfig.height;
    
    // 处理每个像素
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        final int index = (y * width + x) * 4; // 每个像素4个值 (RGBA)
        
        if (index + 3 < costMapColors.length) {
          final int r = costMapColors[index];
          final int g = costMapColors[index + 1];
          final int b = costMapColors[index + 2];
          final int a = costMapColors[index + 3];
          
          // 如果像素不是透明的，则添加到绘制列表
          if (a > 0) {
            costMapPoints.add(CostMapPoint(
              point: Offset(x.toDouble(), y.toDouble()),
              color: Color.fromARGB(
                (a * widget.opacity).round(),
                r,
                g,
                b,
              ),
            ));
          }
        }
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    
    return RepaintBoundary(
      child: ValueListenableBuilder<OccupancyMap>(
        valueListenable: widget.isGlobal 
          ? Provider.of<RosChannel>(context, listen: true).globalCostmap
          : Provider.of<RosChannel>(context, listen: true).localCostmap,
        builder: (context, costMap, child) {
          _processCostMapData(costMap);
          return Container(
            width: costMap.width().toDouble() + 1,
            height: costMap.height().toDouble() + 1,
            child: CustomPaint(
              painter: DisplayCostMapPainter(
                costMapPoints: costMapPoints,
              ),
            ),
          );
        },
      ),
    );
  }
}

class DisplayCostMapPainter extends CustomPainter {
  final List<CostMapPoint> costMapPoints;

  DisplayCostMapPainter({
    required this.costMapPoints,
  });

  @override
  void paint(Canvas canvas, Size size) {
    if (costMapPoints.isEmpty) {
      return;
    }

    Paint paint = Paint()
      ..strokeCap = StrokeCap.butt
      ..style = PaintingStyle.fill
      ..strokeWidth = 1;

    // 批量绘制代价地图点
    for (var costMapPoint in costMapPoints) {
      paint.color = costMapPoint.color;
      canvas.drawPoints(ui.PointMode.points, [costMapPoint.point], paint);
    }
  }

  @override
  bool shouldRepaint(covariant DisplayCostMapPainter oldDelegate) {
    return true;
  }
}

// 新建 CostMapPoint 类来存储位置信息和颜色
class CostMapPoint {
  final Offset point;
  final Color color;

  CostMapPoint({required this.point, required this.color});
}
