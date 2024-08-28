import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayMap extends StatefulWidget {
  const DisplayMap();

  @override
  _DisplayMapState createState() => _DisplayMapState();
}

class _DisplayMapState extends State<DisplayMap> {
  List<Offset> occPointList = [];
  List<Offset> freePointList = [];

  @override
  void initState() {
    super.initState();
  }

  void _processMapData(OccupancyMap map) {
    print("process map");
    occPointList.clear();
    freePointList.clear();
    for (int i = 0; i < map.Cols(); i++) {
      for (int j = 0; j < map.Rows(); j++) {
        int mapValue = map.data[j][i];
        Offset point = Offset(i.toDouble(), j.toDouble());
        if (mapValue > 0) {
          occPointList.add(point);
        } else if (mapValue == 0) {
          freePointList.add(point);
        }
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;
    return ValueListenableBuilder<OccupancyMap>(
        valueListenable: Provider.of<RosChannel>(context, listen: false).map,
        builder: (context, occMap, child) {
          _processMapData(occMap);
          return Container(
            width: occMap.width().toDouble() + 1,
            height: occMap.height().toDouble() + 1,
            child: CustomPaint(
              painter: DisplayMapPainter(
                  occPointList: occPointList,
                  freePointList: freePointList,
                  freeColor: theme.colorScheme.surface.withAlpha(98),
                  occColor: isDarkMode ? Colors.white : Colors.black),
            ),
          );
        });
  }
}

class DisplayMapPainter extends CustomPainter {
  final List<Offset> occPointList;
  final List<Offset> freePointList;
  final Color freeColor;
  final Color occColor;

  DisplayMapPainter(
      {required this.occPointList,
      required this.freePointList,
      required this.freeColor,
      required this.occColor});

  @override
  void paint(Canvas canvas, Size size) {
    print("on paint");
    Paint paint = Paint()
      ..color = occColor
      ..strokeCap = StrokeCap.butt
      ..style = PaintingStyle.fill
      ..strokeWidth = 1;
    canvas.drawPoints(PointMode.points, occPointList, paint);

    paint.color = freeColor;
    canvas.drawPoints(PointMode.points, freePointList, paint);
  }

  @override
  bool shouldRepaint(covariant DisplayMapPainter oldDelegate) {
    return occPointList != oldDelegate.occPointList ||
        freePointList != oldDelegate.freePointList ||
        freeColor != oldDelegate.freeColor ||
        occColor != oldDelegate.occColor;
  }
}
