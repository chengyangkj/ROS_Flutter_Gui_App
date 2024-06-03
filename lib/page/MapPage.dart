import 'dart:async';

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
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
                          Transform(
                            alignment: Alignment.center,
                            transform: globalTransform.value,
                            child: CustomPaint(
                              foregroundPainter: DisplayMap(map: occMap),
                            ),
                          ),

                          //全局路径
                          Transform(
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
                          Transform(
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

                          //激光
                          Transform(
                            transform: globalTransform.value,
                            child: Consumer<RosChannel>(
                              builder: (context, rosChannel, child) {
                                LaserData laserData = rosChannel.laserPointData;
                                RobotPose robotPoseMap = laserData.robotPose;

                                //重定位模式 从图层坐标转换
                                if (relocMode_.value) {
                                  vector.Vector3 translation =
                                      vector.Vector3.zero();
                                  vector.Quaternion rotation =
                                      vector.Quaternion.identity();

                                  //根据机器人的当前位置 做坐标转换
                                  robotPoseMatrix.value.decompose(translation,
                                      rotation, vector.Vector3.zero());
                                  RobotPose robotPoseScene = RobotPose(
                                      translation.x, translation.y, rotation.z);

                                  Offset poseMap = rosChannel.map.value.idx2xy(
                                      Offset(
                                          robotPoseScene.x, robotPoseScene.y));
                                  // print(
                                  //     "trans pose map:${poseMap} real:${rosChannel.robotPoseMap}");
                                  robotPoseMap = RobotPose(
                                      poseMap.dx,
                                      poseMap.dy,
                                      rosChannel.robotPoseMap.theta);
                                }

                                List<Offset> laserPointsScene = [];
                                for (var point in laserData.laserPoseBaseLink) {
                                  RobotPose pointMap = absoluteSum(robotPoseMap,
                                      RobotPose(point.dx, point.dy, 0));
                                  Offset pointScene = rosChannel.map.value
                                      .xy2idx(Offset(pointMap.x, pointMap.y));
                                  laserPointsScene.add(pointScene);
                                }
                                return DisplayLaser(
                                    pointList: laserPointsScene);
                              },
                            ),
                          ),
                          //机器人位置
                          Transform(
                            alignment: Alignment.center,
                            transform: globalTransform.value,
                            child: Consumer<RosChannel>(
                              builder: (context, rosChannel, child) {
                                var pose = rosChannel.robotPoseScene;
                                const double robotSize = 20;
                                vector.Vector3 globalScale =
                                    vector.Vector3.zero();
                                globalTransform.value.decompose(
                                    vector.Vector3.zero(),
                                    vector.Quaternion.identity(),
                                    globalScale);
                                double robotSizeScaled =
                                    (robotSize / 2) / globalScale.z;
                                if (!relocMode_.value) {
                                  robotPoseMatrix.value = Matrix4.identity()
                                    ..translate(pose.x - robotSizeScaled,
                                        pose.y - robotSizeScaled)
                                    ..rotateZ(-pose.theta);
                                }
                                return Transform(
                                    alignment: Alignment.center,
                                    transform: robotPoseMatrix.value,
                                    child: MatrixGestureDetector(
                                      onMatrixUpdate: (m, tm, sm, rm) {
                                        if (relocMode_.value) {
                                          //获取global的scale值
                                          vector.Vector3 globalScale =
                                              vector.Vector3.zero();
                                          globalTransform.value.decompose(
                                              vector.Vector3.zero(),
                                              vector.Quaternion.identity(),
                                              globalScale);
                                          //获取移动距离的变化量矩阵的距离值
                                          vector.Vector3 deltaDis =
                                              vector.Vector3.zero();
                                          tm.decompose(
                                              deltaDis,
                                              vector.Quaternion.identity(),
                                              vector.Vector3.zero());
                                          //移动距离的deleta距离需要除于当前的scale的值(放大后，相同移动距离，地图实际移动的要少)
                                          Matrix4 deltaMatrix =
                                              Matrix4.identity()
                                                ..translate(
                                                    deltaDis.x / globalScale.x,
                                                    deltaDis.y / globalScale.y,
                                                    deltaDis.z / globalScale.z);
                                          //坐标变换sum
                                          robotPoseMatrix.value = deltaMatrix *
                                              robotPoseMatrix.value;
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
                            setState(() {});
                          },
                          icon: Icon(
                            Icons.location_on_outlined,
                            color:
                                relocMode_.value ? Colors.blue : Colors.black,
                          )),
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
