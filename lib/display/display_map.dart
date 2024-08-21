import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';

class DisplayMap extends StatefulWidget {
  late OccupancyMap map;

  DisplayMap({required this.map});

  @override
  _DisplayMapState createState() => _DisplayMapState();
}

class _DisplayMapState extends State<DisplayMap>
    with SingleTickerProviderStateMixin {
  @override
  void initState() {
    super.initState();
  }

  @override
  void dispose() {
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      width: widget.map.width().toDouble() + 1,
      height: widget.map.height().toDouble() + 1,
      child: CustomPaint(
        painter: DisplayMapPainter(map: widget.map),
      ),
    );
  }
}

class DisplayMapPainter extends CustomPainter {
  late OccupancyMap map;
  List<Offset> occPointList = [];
  List<Offset> freePointList = [];
  DisplayMapPainter({required this.map}) {
    for (int i = 0; i < map.Cols(); i++)
      // ignore: curly_braces_in_flow_control_structures
      for (int j = 0; j < map.Rows(); j++) {
        int mapValue = map.data[j][i];
        if (mapValue > 0) {
          Offset point = Offset(i.toDouble(), j.toDouble());
          occPointList.add(point);
        } else if (mapValue == 0) {
          Offset point = Offset(i.toDouble(), j.toDouble());
          freePointList.add(point);
        }
      }
  }

  @override
  void paint(Canvas canvas, Size size) {
    Paint paint = Paint()
      ..color = Colors.lightBlue[600]! // 设置颜色为传入的颜色参数
      ..strokeCap = StrokeCap.butt
      ..style = PaintingStyle.fill
      ..strokeWidth = 1.3; // 设置画笔的宽度为1个像素
    canvas.drawPoints(PointMode.points, occPointList, paint);

    paint.color = Colors.blue[100]!;
    canvas.drawPoints(PointMode.points, freePointList, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) =>
      this != oldDelegate;
}
