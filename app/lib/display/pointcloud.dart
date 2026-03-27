import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/provider/ws_channel.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildPointCloudLayer(WsChannel wsChannel, WorldToLatLngFn worldToLatLng) {
  final points = wsChannel.pointCloud2Data.value;
  if (points.isEmpty) return const SizedBox.shrink();
  final markers = points
      .where((p) => p.x.isFinite && p.y.isFinite)
      .map((p) => Marker(
            point: worldToLatLng(p.x, p.y),
            width: 2,
            height: 2,
            child: Container(
              width: 2,
              height: 2,
              decoration: BoxDecoration(
                color: Colors.orange,
                shape: BoxShape.circle,
              ),
            ),
          ))
      .toList();
  if (markers.isEmpty) return const SizedBox.shrink();
  return MarkerLayer(markers: markers);
}
