import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildLaserLayer(RosChannel rosChannel, WorldToLatLngFn worldToLatLng) {
  final laserData = rosChannel.laserPointData.value;
  if (laserData.laserPoseBaseLink.isEmpty) return const SizedBox.shrink();
  final robotPose = laserData.robotPose;
  final points = <LatLng>[];
  for (final lp in laserData.laserPoseBaseLink) {
    final poseMap = absoluteSum(robotPose, RobotPose(lp.x, lp.y, 0));
    points.add(worldToLatLng(poseMap.x, poseMap.y));
  }
  if (points.isEmpty) return const SizedBox.shrink();
  return PolylineLayer(
    polylines: [
      Polyline(
        points: points,
        color: Colors.red.withOpacity(0.8),
        strokeWidth: 1,
      ),
    ],
  );
}
