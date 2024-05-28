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
import 'package:vector_math/vector_math_64.dart' as vector;

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPageState();
}

class _MapPageState extends State<MapPage> {
  ValueNotifier<bool> relocMode_ = ValueNotifier(false);
  final ValueNotifier<Matrix4> globalTransform =
      ValueNotifier(Matrix4.identity());
  final ValueNotifier<Matrix4> robotPoseMatrix =
      ValueNotifier(Matrix4.identity());
  @override
  Widget build(BuildContext context) {
    final screenSize = MediaQuery.of(context).size;
    return Scaffold(
      backgroundColor: Colors.white,
      body: Stack(
        children: [
          AnimatedBuilder(
            animation: globalTransform,
            builder: (ctx, child) {
              return ValueListenableBuilder<OccupancyMap>(
                valueListenable:
                    Provider.of<RosChannel>(context, listen: false).map,
                builder: (context, occMap, child) {
                  return Container(
                    color: Colors.grey,
                    width: screenSize.width,
                    height: screenSize.height,
                    child: MatrixGestureDetector(
                      onMatrixUpdate: (m, tm, sm, rm) {
                        globalTransform.value = m;
                      },
                      child: Stack(
                        children: [
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
                          Container(
                            transform: globalTransform.value,
                            child: CustomPaint(
                              foregroundPainter: DisplayMap(map: occMap),
                            ),
                          ),

                          //激光
                          Container(
                            transform: globalTransform.value,
                            child: Consumer<RosChannel>(
                              builder: (context, rosChannel, child) {
                                return Container(
                                  child: CustomPaint(
                                    painter: DisplayLaser(
                                        pointList: rosChannel.laserPointScene),
                                  ),
                                );
                              },
                            ),
                          ),
                          //全局路径
                          Container(
                            transform: globalTransform.value,
                            child: Consumer<RosChannel>(
                              builder: (context, rosChannel, child) {
                                return Container(
                                  child: CustomPaint(
                                    painter: DisplayPath(
                                        pointList: rosChannel.globalPathScene,
                                        color: Colors.green),
                                  ),
                                );
                              },
                            ),
                          ),
                          //局部路径
                          Container(
                            transform: globalTransform.value,
                            child: Consumer<RosChannel>(
                              builder: (context, rosChannel, child) {
                                return Container(
                                  child: CustomPaint(
                                    painter: DisplayPath(
                                        pointList: rosChannel.localPathScene,
                                        color: Colors.yellow),
                                  ),
                                );
                              },
                            ),
                          ),
                          //机器人位置

                          Transform(
                            transform: globalTransform.value,
                            child: Consumer<RosChannel>(
                              builder: (context, rosChannel, child) {
                                var pose = rosChannel.robotPoseScene;
                                const double robotSize = 20;
                                if (!relocMode_.value) {
                                  robotPoseMatrix.value = Matrix4.identity()
                                    ..translate(pose.x - robotSize / 2,
                                        pose.y - robotSize / 2)
                                    ..rotateZ(-pose.theta);
                                }
                                return Transform(
                                    transform: robotPoseMatrix.value,
                                    child: MatrixGestureDetector(
                                      onMatrixUpdate: (m, tm, sm, rm) {
                                        if (relocMode_.value) {
                                          // vector.Vector3 position =
                                          //     vector.Vector3.zero();
                                          // vector.Quaternion rotation =
                                          //     vector.Quaternion.identity();
                                          // vector.Vector3 scale =
                                          //     vector.Vector3.zero();
                                          // tm.decompose(
                                          //     position, rotation, scale);
                                          // print("get posetion:${position}");
                                          robotPoseMatrix.value =
                                              tm * robotPoseMatrix.value;
                                          robotPoseMatrix.value =
                                              rm * robotPoseMatrix.value;
                                        }
                                      },
                                      child: DisplayRobot(
                                        size: robotSize,
                                        color: Colors.blue,
                                        count: 2,
                                      ),
                                    ));
                              },
                            ),
                          ),
                        ],
                      ),
                    ),
                  );
                },
              );
            },
          ),

          //左侧菜单栏
          Positioned(
              left: 5,
              top: 10,
              child: Card(
                color: Colors.white70,
                elevation: 10,
                child: Container(
                  child: Column(
                    children: [
                      IconButton(
                          onPressed: () {}, icon: const Icon(Icons.layers)),
                      IconButton(
                          onPressed: () {
                            if (relocMode_.value == false) {
                              relocMode_.value = true;
                            } else {
                              relocMode_.value = false;
                            }
                            print("set value:${relocMode_.value}");
                          },
                          icon: Icon(Icons.location_on_outlined)),
                      IconButton(
                          onPressed: () {
                            if (relocMode_.value == false) {
                              relocMode_.value = true;
                            } else {
                              relocMode_.value = false;
                            }
                            print("set value:${relocMode_.value}");
                          },
                          icon: Icon(Icons.location_on_outlined))
                    ],
                  ),
                ),
              )),
          //右侧菜单栏
          Positioned(
              right: 5,
              top: 10,
              child: Card(
                color: Colors.white70,
                elevation: 10,
                child: Container(
                  child: Column(
                    children: [
                      IconButton(
                          onPressed: () {}, icon: const Icon(Icons.layers)),
                      IconButton(
                          onPressed: () {}, icon: const Icon(Icons.podcasts)),
                      IconButton(
                          onPressed: () {},
                          icon: Icon(Icons.location_on_outlined))
                    ],
                  ),
                ),
              )),
          //右下方菜单栏
          Positioned(
            right: 5,
            bottom: 10,
            child: Container(
              child: Row(
                children: [
                  IconButton(onPressed: () {}, icon: const Icon(Icons.zoom_in)),
                  IconButton(
                      onPressed: () {}, icon: const Icon(Icons.zoom_out)),
                  IconButton(
                      onPressed: () {}, icon: Icon(Icons.location_searching))
                ],
              ),
            ),
          )
        ],
      ),
    );
  }
}
