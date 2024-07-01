import 'dart:async';
import 'dart:math';
import 'dart:math';
import 'package:flutter/cupertino.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_expandable_fab/flutter_expandable_fab.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/math.dart';
import 'package:ros_flutter_gui_app/basic/matrix_gesture_detector.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/display/display_laser.dart';
import 'package:ros_flutter_gui_app/display/display_path.dart';
import 'package:ros_flutter_gui_app/display/display_robot.dart';
import 'package:ros_flutter_gui_app/display/display_robot_reloc.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
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
  ValueNotifier<bool> manualCtrlMode_ = ValueNotifier(false);
  final ValueNotifier<Matrix4> globalTransform =
      ValueNotifier(Matrix4.identity());
  final ValueNotifier<Matrix4> robotPoseMatrix =
      ValueNotifier(Matrix4.identity());
  final ValueNotifier<double> globalScale_ = ValueNotifier(1);
  RobotPose poseSceneStartReloc = RobotPose(0, 0, 0);
  RobotPose poseSceneOnReloc = RobotPose(0, 0, 0);
  double calculateApexAngle(double r, double d) {
    // 使用余弦定理求顶角的余弦值
    double cosC = (r * r + r * r - d * d) / (2 * r * r);
    // 计算顶角弧度
    double apexAngleRadians = acos(cosC);
    return apexAngleRadians;
  }

  @override
  void initState() {
    super.initState();
    globalSetting.init();
  }

  @override
  Widget build(BuildContext context) {
    final _key = GlobalKey<ExpandableFabState>();
    final screenSize = MediaQuery.of(context).size;

    return Scaffold(
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
                    color: Colors.blue[400],
                    width: screenSize.width,
                    height: screenSize.height,
                    child: MatrixGestureDetector(
                      onMatrixUpdate:
                          (matrix, transDelta, scaleDelta, rotateDelta) {
                        globalTransform.value = matrix;
                        vector.Vector3 scale = vector.Vector3.zero();
                        globalTransform.value.decompose(vector.Vector3.zero(),
                            vector.Quaternion.identity(), scale);
                        globalScale_.value = scale.z;
                      },
                      child: Stack(
                        children: [
                          //网格
                          Container(
                            color: Colors.blue[800],
                            child: DisplayGrid(
                              step: (1 / occMap.mapConfig.resolution) *
                                  globalScale_.value,
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
                                        color: Colors.yellow[200]!),
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
                                  Offset poseMap = rosChannel.map.value.idx2xy(
                                      Offset(poseSceneOnReloc.x,
                                          poseSceneOnReloc.y));
                                  robotPoseMap = RobotPose(poseMap.dx,
                                      poseMap.dy, poseSceneOnReloc.theta);
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
                                var robotPose = rosChannel.robotPoseScene;

                                //由于变换在机器人图片中心点 需要根据机器人的图片尺寸及缩放比例 计算实际机器人位置
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
                                    ..translate(robotPose.x - robotSizeScaled,
                                        robotPose.y - robotSizeScaled)
                                    ..rotateZ(-robotPose.theta);
                                }
                                return Transform(
                                    alignment: Alignment.center,
                                    transform: robotPoseMatrix.value,
                                    child: MatrixGestureDetector(
                                      onMatrixUpdate: (matrix, transDelta,
                                          scaleDelta, rotateDelta) {
                                        if (relocMode_.value) {
                                          //获取global的scale值
                                          vector.Vector3 globalScale =
                                              vector.Vector3.zero();
                                          globalTransform.value.decompose(
                                              vector.Vector3.zero(),
                                              vector.Quaternion.identity(),
                                              globalScale);

                                          //移动距离的deleta距离需要除于当前的scale的值(放大后，相同移动距离，地图实际移动的要少)
                                          double dx =
                                              transDelta.dx / globalScale.x;
                                          double dy =
                                              transDelta.dy / globalScale.y;
                                          double theta = poseSceneOnReloc.theta;
                                          poseSceneOnReloc = absoluteSum(
                                              RobotPose(poseSceneOnReloc.x,
                                                  poseSceneOnReloc.y, 0),
                                              RobotPose(dx, dy, 0));
                                          poseSceneOnReloc.theta = theta;
                                          //坐标变换sum
                                          robotPoseMatrix
                                              .value = Matrix4.identity()
                                            ..translate(
                                                poseSceneOnReloc.x -
                                                    robotSizeScaled,
                                                poseSceneOnReloc.y -
                                                    robotSizeScaled)
                                            ..rotateZ(-poseSceneOnReloc.theta);
                                          vector.Vector3 pose =
                                              vector.Vector3.zero();
                                          robotPoseMatrix.value.decompose(
                                              pose,
                                              vector.Quaternion.identity(),
                                              vector.Vector3.zero());
                                        }
                                      },
                                      child: Container(
                                        height: robotSize + 10,
                                        width: robotSize + 10,
                                        // // 设置边框
                                        // decoration: BoxDecoration(
                                        //   // 设置边框
                                        //   border: Border.all(
                                        //     color: Colors.red, // 边框颜色
                                        //     width: 1, // 边框宽度
                                        //   ),
                                        // ),
                                        child: Stack(
                                          alignment: Alignment.center,
                                          children: [
                                            //重定位旋转框
                                            DisplayRobotReloc(
                                              size: robotSize + 10,
                                              relocMode: relocMode_.value,
                                              onRotateCallback: (angle) {
                                                poseSceneOnReloc.theta =
                                                    (poseSceneStartReloc.theta -
                                                        angle);
                                                //坐标变换sum
                                                robotPoseMatrix
                                                    .value = Matrix4.identity()
                                                  ..translate(
                                                      poseSceneOnReloc.x -
                                                          robotSizeScaled,
                                                      poseSceneOnReloc.y -
                                                          robotSizeScaled)
                                                  ..rotateZ(
                                                      -poseSceneOnReloc.theta);
                                              },
                                            ),

                                            //机器人图标
                                            DisplayRobot(
                                              size: robotSize,
                                              color: Colors.yellow,
                                              count: 2,
                                            ),
                                            // IconButton(
                                            //   onPressed: () {},
                                            //   iconSize: robotSize / 2,
                                            //   icon: Icon(Icons.check),
                                            // ),
                                          ],
                                        ),
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
              child: Column(
                children: [
                  Card(
                    color: Colors.white70,
                    elevation: 10,
                    child: Container(
                      child: Row(
                        children: [
                          IconButton(
                              onPressed: () {
                                if (relocMode_.value == false) {
                                  relocMode_.value = true;
                                  poseSceneStartReloc = Provider.of<RosChannel>(
                                          context,
                                          listen: false)
                                      .robotPoseScene;

                                  poseSceneOnReloc = Provider.of<RosChannel>(
                                          context,
                                          listen: false)
                                      .robotPoseScene;
                                  setState(() {});
                                } else {
                                  relocMode_.value = false;
                                }
                                setState(() {});
                              },
                              icon: Icon(
                                Icons.location_on_outlined,
                                color: relocMode_.value
                                    ? Colors.blue
                                    : Colors.black,
                              )),
                          Visibility(
                              visible: relocMode_.value,
                              child: IconButton(
                                  onPressed: () {
                                    relocMode_.value = false;
                                    setState(() {});
                                  },
                                  icon: const Icon(
                                    Icons.close,
                                    color: Colors.red,
                                  ))),
                          Visibility(
                              visible: relocMode_.value,
                              child: IconButton(
                                  onPressed: () {
                                    relocMode_.value = false;
                                    Provider.of<RosChannel>(context,
                                            listen: false)
                                        .sendRelocPoseScene(poseSceneOnReloc);
                                    setState(() {});
                                  },
                                  icon: const Icon(Icons.check,
                                      color: Colors.green))),
                        ],
                      ),
                    ),
                  ),
                  Card(
                    color: Colors.white70,
                    elevation: 10,
                    child: IconButton(
                      icon: Icon(manualCtrlMode_.value
                          ? Icons.gamepad
                          : Icons.gamepad_outlined),
                      onPressed: () {
                        if (manualCtrlMode_.value) {
                          manualCtrlMode_.value = false;
                          setState(() {});
                        } else {
                          manualCtrlMode_.value = true;
                          setState(() {});
                        }
                      },
                    ),
                  )
                ],
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
          ),
          Positioned(
            left: 30,
            bottom: 10,
            child: Visibility(
                visible: manualCtrlMode_.value,
                child: Joystick(
                  mode: JoystickMode.all,
                  listener: (details) {
                    setState(() {
                      double max_vx =
                          double.parse(globalSetting.getConfig('MaxVx'));
                      double max_vy =
                          double.parse(globalSetting.getConfig('MaxVy'));
                      double max_vw =
                          double.parse(globalSetting.getConfig('MaxVw'));
                      double vx = max_vx * details.y * -1;
                      double vy = max_vy * details.x * -1;
                      //x决定方向 1-y决定比例
                      double vw = (1 - details.y.abs()) * max_vw;

                      if (details.x > 0) {
                        if (details.y < 0) {
                          vw = -vw;
                        }
                      } else if (details.x < 0) {
                        if (details.y > 0) {
                          vw = -vw;
                        }
                      }
                      //y小于一定值 只有w
                      if (details.y.abs() <= 0.1) {
                        vx = 0;
                        if (details.x > 0) {
                          vw = -vw.abs();
                        } else {
                          vw = vw.abs();
                        }
                      }
                      if (details.y == 0) {
                        vw = 0;
                      }
                      Provider.of<RosChannel>(context, listen: false)
                          .sendSpeed(vx, vy, vw);
                    });
                  },
                )),
          )
        ],
      ),
    );
  }
}
