import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildGridLayer({
  required double resolution,
  required double originX,
  required double originY,
  required int width,
  required int height,
  required WorldToLatLngFn worldToLatLng,
  required bool isDark,
}) {
  final widthM = width * resolution;
  final heightM = height * resolution;
  final color = isDark ? Colors.white.withOpacity(0.2) : Colors.black.withOpacity(0.3);
  final polylines = <Polyline>[];
  for (double x = 0; x <= widthM; x += 1.0) {
    polylines.add(Polyline(
      points: [
        worldToLatLng(originX + x, originY),
        worldToLatLng(originX + x, originY + heightM),
      ],
      color: color,
      strokeWidth: 0.5,
    ));
  }
  for (double y = 0; y <= heightM; y += 1.0) {
    polylines.add(Polyline(
      points: [
        worldToLatLng(originX, originY + y),
        worldToLatLng(originX + widthM, originY + y),
      ],
      color: color,
      strokeWidth: 0.5,
    ));
  }
  return PolylineLayer(polylines: polylines);
}
