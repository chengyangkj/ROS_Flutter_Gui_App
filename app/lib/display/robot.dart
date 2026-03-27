import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/display/pose_marker.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ws_channel.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildRobotMarkerLayer(
  WsChannel wsChannel,
  WorldToLatLngFn worldToLatLng, {
  RobotPose? poseOverride,
  bool isEditMode = false,
  ValueChanged<double>? onThetaChanged,
  ValueChanged<Offset>? onMoveDelta,
}) {
  final robotPose = poseOverride ?? wsChannel.robotPoseMap.value;
  final robotSize = globalSetting.robotSize.toDouble();
  return MarkerLayer(
    markers: [
      Marker(
        point: worldToLatLng(robotPose.x, robotPose.y),
        width: robotSize,
        height: robotSize,
        alignment: Alignment.center,
        child: PoseMarkerWidget(
          size: robotSize,
          theta: -robotPose.theta,
          type: PoseMarkerType.Robot,
          isEditMode: isEditMode,
          onThetaChanged: onThetaChanged,
          onMoveDelta: onMoveDelta,
          color: const Color(0xFF0080ff),
        ),
      ),
    ],
  );
}
