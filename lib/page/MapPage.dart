import 'dart:async';

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:matrix_gesture_detector_pro/matrix_gesture_detector_pro.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/display/display_robot.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/hardware/gamepad.dart';
import 'package:ros_flutter_gui_app/display/display_map.dart';

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPageState();
}

class _MapPageState extends State<MapPage> {
  double scaleValue_ = 1;
  @override
  Widget build(BuildContext context) {
    final ValueNotifier<Matrix4> notifier = ValueNotifier(Matrix4.identity());
    return Scaffold(
        backgroundColor: Colors.grey,
        body: Listener(
            // 监听手势事件
            onPointerSignal: (event) {
              if (event is PointerScrollEvent) {
                if (event.scrollDelta.dy < 0) {
                  scaleValue_ += 0.1;
                } else {
                  scaleValue_ -= 0.1;
                }
                setState(() {});
              }
            },
            child: MatrixGestureDetector(
              onMatrixUpdate: (m, tm, sm, rm) {
                notifier.value = m;
              },
              child: AnimatedBuilder(
                animation: notifier,
                builder: (ctx, child) {
                  return Transform(
                    transform: notifier.value
                        .scaled(scaleValue_, scaleValue_, scaleValue_),
                    child: ValueListenableBuilder<OccupancyMap>(
                      valueListenable:
                          Provider.of<RosChannel>(context, listen: false).map,
                      builder: (context, value, child) {
                        return Container(
                          transform: notifier.value
                              .scaled(scaleValue_, scaleValue_, scaleValue_),
                          child: CustomPaint(
                            foregroundPainter: DisplayMap(map: value),
                            child: Stack(
                              children: <Widget>[
                                Container(
                                  color: Colors.white,
                                ),
                                Positioned(child: Consumer<RosChannel>(
                                  builder: (context, rosChannel, child) {
                                    var pose = rosChannel.robotPoseScene;
                                    const double robotSize = 30;
                                    return Container(
                                      transform: Matrix4.identity()
                                        ..translate(pose.x, pose.y)
                                        ..rotateZ(-pose.theta),
                                      width: robotSize,
                                      height: robotSize,
                                      child: DisplayRobot(
                                        size: robotSize,
                                        color: Colors.blue,
                                        count: 2,
                                      ),
                                    );
                                  },
                                ))
                              ],
                            ),
                          ),
                        );
                      },
                    ),
                  );
                },
              ),
            )));
  }
}
