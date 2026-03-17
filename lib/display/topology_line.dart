import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/pose_marker.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildTopologyLineLayer(
  List<NavPoint> points,
  List<TopologyRoute> routes,
  WorldToLatLngFn worldToLatLng, {
  void Function(NavPoint?)? onNavPointTap,
  bool isEditMode = false,
  ValueChanged<NavPoint>? onNavPointChanged,
  void Function(NavPoint point, Offset delta)? onNavPointMoveDelta,
  String? selectedNavPointName,
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
            width: globalSetting.robotSize.toDouble(),
            height: globalSetting.robotSize.toDouble(),
            alignment: Alignment.center,
            child: GestureDetector(
              onTap: onNavPointTap != null ? () => onNavPointTap(p) : null,
              child: PoseMarkerWidget(
                size: globalSetting.robotSize.toDouble(),
                theta: -p.theta,
                type: PoseMarkerType.NavPoint,
                color: selectedNavPointName != null && selectedNavPointName == p.name
                    ? Colors.red
                    : Colors.green,
                isEditMode: isEditMode,
                onThetaChanged: onNavPointChanged != null
                    ? (theta) {
                        onNavPointChanged(
                          NavPoint(
                            name: p.name,
                            x: p.x,
                            y: p.y,
                            theta: theta,
                            type: p.type,
                          ),
                        );
                      }
                    : null,
                onMoveDelta: onNavPointMoveDelta != null
                    ? (delta) => onNavPointMoveDelta(p, delta)
                    : null,
              ),
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
