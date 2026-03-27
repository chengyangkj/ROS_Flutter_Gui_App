import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/provider/ws_channel.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildLaserLayer(
  WsChannel wsChannel,
  WorldToLatLngFn worldToLatLng, {
  RobotPose? robotPoseOverride,
  Color? color,
  double dotRadius = 2,
}) {
  final laserData = wsChannel.laserPointData.value;
  if (laserData.laserPoseBaseLink.isEmpty) return const SizedBox.shrink();
  final robotPose = robotPoseOverride ?? laserData.robotPose;
  final c = color ?? Colors.red;
  final alpha = (c.a * 0.8).clamp(0.0, 1.0);
  final fill = c.withValues(alpha: alpha);
  final points = <LatLng>[];
  for (final lp in laserData.laserPoseBaseLink) {
    final poseMap = absoluteSum(robotPose, RobotPose(lp.x, lp.y, 0));
    points.add(worldToLatLng(poseMap.x, poseMap.y));
  }
  if (points.isEmpty) return const SizedBox.shrink();
  final r = dotRadius.clamp(0.5, 24.0);
  final circles = points
      .map((p) => CircleMarker(
            point: p,
            radius: r,
            color: fill,
          ))
      .toList();
  return CircleLayer(circles: circles);
}
