import 'dart:async';

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:matrix_gesture_detector_pro/matrix_gesture_detector_pro.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/display/display_laser.dart';
import 'package:ros_flutter_gui_app/display/display_robot.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/hardware/gamepad.dart';
import 'package:ros_flutter_gui_app/display/display_map.dart';
import 'package:ros_flutter_gui_app/display/display_grid.dart';

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPageState();
}

class _MapPageState extends State<MapPage> {
  double scaleValue_ = 2;
  @override
  Widget build(BuildContext context) {
    final ValueNotifier<Matrix4> notifier = ValueNotifier(Matrix4.identity());
    final screenSize = MediaQuery.of(context).size;
    return Scaffold(
      backgroundColor: Colors.white,
      body: Row(
        children: [
          Expanded(
              child: Listener(
                  // 监听手势事件
                  onPointerSignal: (event) {
                    if (event is PointerScrollEvent) {
                      if (event.scrollDelta.dy < 0) {
                        scaleValue_ += 0.1;
                      } else {
                        scaleValue_ -= 0.1;
                      }
                      if (scaleValue_ < 1) scaleValue_ = 1;
                      if (scaleValue_ > 5) scaleValue_ = 5;
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
                                Provider.of<RosChannel>(context, listen: false)
                                    .map,
                            builder: (context, occMap, child) {
                              return Container(
                                color: Colors.red,
                                transform: notifier.value.scaled(
                                    scaleValue_, scaleValue_, scaleValue_),
                                width: screenSize.width,
                                height: screenSize.height,
                                child: Container(
                                  child: Stack(
                                    children: <Widget>[
                                      //网格
                                      Container(
                                        color: Colors.white,
                                        child: DisplayGrid(
                                          step: 1 / occMap.mapConfig.resolution,
                                          width: screenSize.width,
                                          height: screenSize.height,
                                        ),
                                      ),
                                      //地图
                                      CustomPaint(
                                        foregroundPainter:
                                            DisplayMap(map: occMap),
                                      ),

                                      //机器人位置
                                      Positioned(
                                        child: Consumer<RosChannel>(
                                          builder:
                                              (context, rosChannel, child) {
                                            var pose =
                                                rosChannel.robotPoseScene;
                                            const double robotSize = 20;
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
                                        ),
                                      ),
                                      //激光
                                      Positioned(
                                        child: Consumer<RosChannel>(
                                          builder:
                                              (context, rosChannel, child) {
                                            return Container(
                                              child: CustomPaint(
                                                painter: DisplayLaser(
                                                    pointList: rosChannel
                                                        .laserPointScene),
                                              ),
                                            );
                                          },
                                        ),
                                      )
                                    ],
                                  ),
                                ),
                              );
                            },
                          ),
                        );
                      },
                    ),
                  )))
        ],
      ),
    );
  }
}
