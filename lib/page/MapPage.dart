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
import 'package:ros_flutter_gui_app/display/display_pose_direction.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/hardware/gamepad.dart';
import 'package:ros_flutter_gui_app/display/display_map.dart';
import 'package:ros_flutter_gui_app/display/display_grid.dart';
import 'package:vector_math/vector_math_64.dart' as vector;
import 'package:ros_flutter_gui_app/display/display_waypoint.dart';

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPageState();
}

enum Mode {
  noraml,
  reloc,
  addNavPoint,
}

class _MapPageState extends State<MapPage> {
  ValueNotifier<Mode> mode_ = ValueNotifier(Mode.noraml);
  ValueNotifier<bool> manualCtrlMode_ = ValueNotifier(false);
  ValueNotifier<List<RobotPose>> navPointList_ =
      ValueNotifier<List<RobotPose>>([]);
  final ValueNotifier<Matrix4> globalTransform =
      ValueNotifier(Matrix4.identity());
  final ValueNotifier<RobotPose> robotPose_ = ValueNotifier(RobotPose(0, 0, 0));
  final ValueNotifier<double> globalScale_ = ValueNotifier(1);

  int poseDirectionSwellSize = 10;
  double navPoseSize = 15;
  double robotSize = 20;
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
                    width: screenSize.width,
                    height: screenSize.height,
                    child: MatrixGestureDetector(
                      onMatrixUpdate:
                          (matrix, transDelta, scaleValue, rotateDelta) {
                        globalTransform.value = matrix;
                        globalScale_.value = scaleValue;
                      },
                      child: Stack(
                        children: [
                          //网格
                          Container(
                            child: DisplayGrid(
                              step: (1 / occMap.mapConfig.resolution) *
                                  (globalScale_.value > 0.5
                                      ? globalScale_.value
                                      : 0.5),
                              width: screenSize.width,
                              height: screenSize.height,
                            ),
                          ),
                          //地图
                          Transform(
                            transform: globalTransform.value,
                            child: GestureDetector(
                              child: DisplayMap(map: occMap),
                              onTapDown: (details) {
                                if (mode_.value == Mode.addNavPoint) {
                                  navPointList_.value.add(RobotPose(
                                      details.localPosition.dx,
                                      details.localPosition.dy,
                                      0));
                                  setState(() {});
                                }
                              },
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
                              if (mode_.value == Mode.reloc) {
                                Offset poseMap = rosChannel.map.value.idx2xy(
                                    Offset(poseSceneOnReloc.x,
                                        poseSceneOnReloc.y));
                                robotPoseMap = RobotPose(poseMap.dx, poseMap.dy,
                                    poseSceneOnReloc.theta);
                              }

                              List<Offset> laserPointsScene = [];
                              for (var point in laserData.laserPoseBaseLink) {
                                RobotPose pointMap = absoluteSum(robotPoseMap,
                                    RobotPose(point.dx, point.dy, 0));
                                Offset pointScene = rosChannel.map.value
                                    .xy2idx(Offset(pointMap.x, pointMap.y));
                                laserPointsScene.add(pointScene);
                              }
                              return IgnorePointer(
                                  ignoring: true,
                                  child: DisplayLaser(
                                      pointList: laserPointsScene));
                            }),
                          ),
                          //机器人位置
                          Transform(
                            transform: globalTransform.value,
                            child: Consumer<RosChannel>(
                              builder: (context, rosChannel, child) {
                                if (!(mode_.value == Mode.reloc)) {
                                  robotPose_.value = rosChannel.robotPoseScene;
                                }
                                return Transform(
                                    alignment: Alignment.center,
                                    transform: Matrix4.identity()
                                      ..translate(
                                          robotPose_.value.x - robotSize / 2,
                                          robotPose_.value.y - robotSize / 2)
                                      ..rotateZ(-robotPose_.value.theta),
                                    child: MatrixGestureDetector(
                                      onMatrixUpdate: (matrix, transDelta,
                                          scaleDelta, rotateDelta) {
                                        if (mode_.value == Mode.reloc) {
                                          //获取global的scale值

                                          //移动距离的deleta距离需要除于当前的scale的值(放大后，相同移动距离，地图实际移动的要少)
                                          double dx = transDelta.dx /
                                              globalScale_.value;
                                          double dy = transDelta.dy /
                                              globalScale_.value;
                                          double theta = poseSceneOnReloc.theta;
                                          poseSceneOnReloc = absoluteSum(
                                              RobotPose(poseSceneOnReloc.x,
                                                  poseSceneOnReloc.y, 0),
                                              RobotPose(dx, dy, 0));
                                          poseSceneOnReloc.theta = theta;
                                          //坐标变换sum
                                          robotPose_.value = poseSceneOnReloc;
                                        }
                                      },
                                      child: Container(
                                        height:
                                            robotSize + poseDirectionSwellSize,
                                        width:
                                            robotSize + poseDirectionSwellSize,
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
                                            Visibility(
                                              visible:
                                                  mode_.value == Mode.reloc,
                                              child: DisplayPoseDirection(
                                                size: robotSize +
                                                    poseDirectionSwellSize,
                                                resetAngle:
                                                    mode_.value != Mode.reloc,
                                                onRotateCallback: (angle) {
                                                  poseSceneOnReloc.theta =
                                                      (poseSceneStartReloc
                                                              .theta -
                                                          angle);
                                                  //坐标变换sum
                                                  robotPose_.value = RobotPose(
                                                      poseSceneOnReloc.x,
                                                      poseSceneOnReloc.y,
                                                      poseSceneOnReloc.theta);
                                                },
                                              ),
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
                          //导航点
                          ...navPointList_.value.map((pose) {
                            return Transform(
                              transform: globalTransform.value,
                              child: Transform(
                                  alignment: Alignment.center,
                                  transform: Matrix4.identity()
                                    ..translate(
                                        pose.x -
                                            navPoseSize / 2 -
                                            poseDirectionSwellSize / 2,
                                        pose.y -
                                            navPoseSize / 2 -
                                            poseDirectionSwellSize / 2)
                                    ..rotateZ(-pose.theta),
                                  child: MatrixGestureDetector(
                                      onMatrixUpdate: (matrix, transDelta,
                                          scaleDelta, rotateDelta) {
                                        // print("transDelta:${transDelta}");
                                        if (mode_.value == Mode.addNavPoint) {
                                          //移动距离的deleta距离需要除于当前的scale的值(放大后，相同移动距离，地图实际移动的要少)
                                          double dx = transDelta.dx /
                                              globalScale_.value;
                                          double dy = transDelta.dy /
                                              globalScale_.value;
                                          double tmpTheta = pose.theta;
                                          pose = absoluteSum(
                                              RobotPose(
                                                  pose.x, pose.y, pose.theta),
                                              RobotPose(dx, dy, 0));
                                          pose.theta = tmpTheta;
                                          print("trans pose:${pose}");
                                          setState(() {});
                                        }
                                      },
                                      child: Container(
                                        height: navPoseSize +
                                            poseDirectionSwellSize,
                                        width: navPoseSize +
                                            poseDirectionSwellSize,
                                        child: Stack(
                                          alignment: Alignment.center,
                                          children: [
                                            Visibility(
                                                visible: mode_.value ==
                                                    Mode.addNavPoint,
                                                child: DisplayPoseDirection(
                                                  size: navPoseSize +
                                                      poseDirectionSwellSize,
                                                  initAngle: -pose.theta,
                                                  resetAngle: false,
                                                  onRotateCallback: (angle) {
                                                    pose.theta = -angle;
                                                    setState(() {});
                                                  },
                                                )),
                                            DisplayWayPoint(
                                              size: navPoseSize,
                                              color: Colors.green,
                                              count: 4,
                                            ),
                                          ],
                                        ),
                                      ))),
                            );
                          }).toList(),
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
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Card(
                    color: Colors.white70,
                    elevation: 10,
                    child: Container(
                      child: Row(
                        children: [
                          IconButton(
                              onPressed: () {
                                if (!(mode_.value == Mode.reloc)) {
                                  mode_.value = Mode.reloc;
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
                                  mode_.value = Mode.noraml;
                                }
                                setState(() {});
                              },
                              icon: Icon(
                                const IconData(0xe60f, fontFamily: "Reloc"),
                                color: mode_.value == Mode.reloc
                                    ? Colors.blue
                                    : Colors.black,
                              )),
                          Visibility(
                              visible: mode_.value == Mode.reloc,
                              child: IconButton(
                                  onPressed: () {
                                    mode_.value = Mode.noraml;
                                    setState(() {});
                                  },
                                  icon: const Icon(
                                    Icons.close,
                                    color: Colors.red,
                                  ))),
                          Visibility(
                              visible: mode_.value == Mode.reloc,
                              child: IconButton(
                                  onPressed: () {
                                    mode_.value = Mode.noraml;
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
                  //设置导航目标点
                  Card(
                    color: Colors.white70,
                    elevation: 10,
                    child: IconButton(
                      icon: Icon(
                        const IconData(0xeba1, fontFamily: "NavPoint"),
                        color: (mode_.value == Mode.addNavPoint)
                            ? Colors.green
                            : Colors.black,
                      ),
                      onPressed: () {
                        if (!(mode_.value == Mode.addNavPoint)) {
                          mode_.value = Mode.addNavPoint;
                          setState(() {});
                        } else {
                          mode_.value = Mode.noraml;
                          setState(() {});
                        }
                      },
                    ),
                  ),
                  //手动控制
                  Card(
                    color: Colors.white70,
                    elevation: 10,
                    child: IconButton(
                      icon: Icon(const IconData(0xea45, fontFamily: "GamePad"),
                          color: manualCtrlMode_.value
                              ? Colors.green
                              : Colors.black),
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
                maintainState: true,
                child: Joystick(
                  mode: JoystickMode.all,
                  listener: (details) {
                    if (!manualCtrlMode_.value) return;
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
                  },
                )),
          )
        ],
      ),
    );
  }
}
