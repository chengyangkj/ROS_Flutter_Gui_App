import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

Widget buildPathLayer(List<LatLng> points, Color color, {double strokeWidth = 2}) {
  if (points.length < 2) return const SizedBox.shrink();
  return PolylineLayer(
    polylines: [
      Polyline(
        points: points,
        color: color,
        strokeWidth: strokeWidth,
      ),
    ],
  );
}
