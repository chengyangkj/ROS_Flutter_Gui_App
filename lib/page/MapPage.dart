import 'dart:async';

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/matrix_gesture_detector.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/display/display_laser.dart';
import 'package:ros_flutter_gui_app/display/display_path.dart';
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
  @override
  Widget build(BuildContext context) {
    final ValueNotifier<Matrix4> gestureNotifier =
        ValueNotifier(Matrix4.identity());
    final screenSize = MediaQuery.of(context).size;
    return Scaffold(
      backgroundColor: Colors.white,
      body: Stack(
        children: [
          MatrixGestureDetector(
            onMatrixUpdate: (m, tm, sm, rm) {
              gestureNotifier.value = m;
            },
            child: AnimatedBuilder(
              animation: gestureNotifier,
              builder: (ctx, child) {
                return Transform(
                  transform: gestureNotifier.value,
                  child: ValueListenableBuilder<OccupancyMap>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: false).map,
                    builder: (context, occMap, child) {
                      return Container(
                        color: Colors.red,
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
                                foregroundPainter: DisplayMap(map: occMap),
                              ),

                              //机器人位置
                              Positioned(
                                child: Consumer<RosChannel>(
                                  builder: (context, rosChannel, child) {
                                    var pose = rosChannel.robotPoseScene;
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
                                  builder: (context, rosChannel, child) {
                                    return Container(
                                      child: CustomPaint(
                                        painter: DisplayLaser(
                                            pointList:
                                                rosChannel.laserPointScene),
                                      ),
                                    );
                                  },
                                ),
                              ),
                              //全局路径
                              Positioned(
                                child: Consumer<RosChannel>(
                                  builder: (context, rosChannel, child) {
                                    return Container(
                                      child: CustomPaint(
                                        painter: DisplayPath(
                                            pointList:
                                                rosChannel.globalPathScene,
                                            color: Colors.green),
                                      ),
                                    );
                                  },
                                ),
                              ),
                              //局部路径
                              Positioned(
                                child: Consumer<RosChannel>(
                                  builder: (context, rosChannel, child) {
                                    return Container(
                                      child: CustomPaint(
                                        painter: DisplayPath(
                                            pointList:
                                                rosChannel.localPathScene,
                                            color: Colors.yellow),
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
          ),
          Positioned(
            left: 5,
            top: 10,
            child: Card(
              color: Colors.white70,
            elevation: 10,
            child: Container(
              height: 80,
              child: Column(
                children: [
                  IconButton(onPressed: () {}, icon: const Icon(Icons.layers)),
                  IconButton(
                      onPressed: () {}, icon: const Icon(Icons.podcasts)),
                ],
              ),
            ),
          ) )
         
        ],
      ),
    );
  }
}
