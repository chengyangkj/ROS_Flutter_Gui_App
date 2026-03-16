import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';

Widget buildLocalCostMapOverlayLayer(OccupancyMap cm, double opacity) {
  if (cm.data.isEmpty || cm.mapConfig.width == 0) return const SizedBox.shrink();
  return const SizedBox.shrink();
}
