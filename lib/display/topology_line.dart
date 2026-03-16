import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildTopologyLineLayer(
  List<NavPoint> points,
  List<TopologyRoute> routes,
  WorldToLatLngFn worldToLatLng, {
  void Function(NavPoint?)? onNavPointTap,
}) {
  final polylines = <Polyline>[];
  for (final route in routes) {
    NavPoint? from;
    NavPoint? to;
    for (final p in points) {
      if (p.name == route.fromPoint) from = p;
      if (p.name == route.toPoint) to = p;
    }
    if (from != null && to != null) {
      polylines.add(Polyline(
        points: [
          worldToLatLng(from.x, from.y),
          worldToLatLng(to.x, to.y),
        ],
        color: Colors.orange.withOpacity(0.6),
        strokeWidth: 1.5,
      ));
    }
  }

  final markers = points
      .map((p) => Marker(
            point: worldToLatLng(p.x, p.y),
            width: 24,
            height: 24,
            alignment: Alignment.center,
            child: GestureDetector(
              onTap: onNavPointTap != null ? () => onNavPointTap(p) : null,
              child: Icon(Icons.place, color: Colors.green, size: 24),
            ),
          ))
      .toList();

  return Stack(
    children: [
      if (polylines.isNotEmpty) PolylineLayer(polylines: polylines),
      MarkerLayer(markers: markers),
    ],
  );
}
