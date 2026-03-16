import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

Widget buildRobotMarkerLayer(RosChannel rosChannel, WorldToLatLngFn worldToLatLng) {
  final robotPose = rosChannel.robotPoseMap.value;
  return MarkerLayer(
    markers: [
      Marker(
        point: worldToLatLng(robotPose.x, robotPose.y),
        width: globalSetting.robotSize.toDouble(),
        height: globalSetting.robotSize.toDouble(),
        alignment: Alignment.center,
        child: Transform.rotate(
          angle: robotPose.theta,
          child: Icon(Icons.smart_toy, color: Colors.blue, size: globalSetting.robotSize),
        ),
      ),
    ],
  );
}
