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
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/hardware/gamepad.dart';
import 'package:ros_flutter_gui_app/display/display_map.dart';
import 'package:ros_flutter_gui_app/display/display_grid.dart';
import 'package:toast/toast.dart';
import 'package:vector_math/vector_math_64.dart' as vector;
import 'package:ros_flutter_gui_app/display/display_waypoint.dart';

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPageState();
}

class _MapPageState extends State<MapPage> with SingleTickerProviderStateMixin {
  ValueNotifier<bool> manualCtrlMode_ = ValueNotifier(false);
  ValueNotifier<List<RobotPose>> navPointList_ =
      ValueNotifier<List<RobotPose>>([]);
  final ValueNotifier<Matrix4> gestureTransform =
      ValueNotifier(Matrix4.identity());

  Matrix4 cameraFixedTransform = Matrix4.identity(); //固定相机视角(以机器人为中心)
  double cameraFixedScaleValue_ = 1; //固定相机视角时的缩放值

  late AnimationController animationController;
  late Animation<double> animationValue;

  final ValueNotifier<RobotPose> robotPose_ = ValueNotifier(RobotPose(0, 0, 0));
  final ValueNotifier<double> gestureScaleValue_ = ValueNotifier(1);
  OverlayEntry? _overlayEntry;
  RobotPose currentNavGoal_ = RobotPose.zero();

  int poseDirectionSwellSize = 10; //机器人方向旋转控件膨胀的大小
  double navPoseSize = 15; //导航点的大小
  double robotSize = 20; //机器人坐标的大小
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
    // 初始化 AnimationController
    animationController = AnimationController(
      duration: const Duration(seconds: 2), // 动画持续 1 秒
      vsync: this,
    );

    // 初始化 Tween，从 1.0 到 2.0
    animationValue =
        Tween<double>(begin: 1.0, end: 4.0).animate(animationController)
          ..addListener(() {
            setState(() {
              cameraFixedScaleValue_ =
                  animationValue.value; // 更新 cameraFixedScaleValue_
            });
          });
  }

  void _showContextMenu(BuildContext context, Offset position) {
    final overlay =
        Overlay.of(context)?.context.findRenderObject() as RenderBox;
    final menuOverlay = Overlay.of(context);

    _overlayEntry = OverlayEntry(
      builder: (context) => Positioned(
        left: position.dx,
        top: position.dy,
        child: Material(
          color: Colors.transparent,
          child: Container(
            width: 300,
            height: 300,
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(8),
              boxShadow: const [
                BoxShadow(
                  color: Colors.black26,
                  blurRadius: 10,
                  spreadRadius: 1,
                ),
              ],
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Row(
                  children: [
                    TextButton(onPressed: () {}, child: Text("确定")),
                    TextButton(
                        onPressed: () {
                          _hideContextMenu();
                        },
                        child: Text("取消"))
                  ],
                )
              ],
            ),
          ),
        ),
      ),
    );

    menuOverlay?.insert(_overlayEntry!);
  }

  void _hideContextMenu() {
    _overlayEntry?.remove();
    _overlayEntry = null;
  }

  @override
  Widget build(BuildContext context) {
    ToastContext().init(context);
    final _key = GlobalKey<ExpandableFabState>();
    final screenSize = MediaQuery.of(context).size;
    final screenCenter = Offset(screenSize.width / 2, screenSize.height / 2);
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;

    return Scaffold(
      body: Stack(
        children: [
          AnimatedBuilder(
            animation: gestureTransform,
            builder: (ctx, child) {
              return Container(
                  width: screenSize.width,
                  height: screenSize.height,
                  child: MatrixGestureDetector(
                    onMatrixUpdate:
                        (matrix, transDelta, scaleValue, rotateDelta) {
                      if (Provider.of<GlobalState>(context, listen: false)
                              .mode
                              .value ==
                          Mode.robotFixedCenter) {
                        Toast.show("相机视角固定时不可调整图层！",
                            duration: Toast.lengthShort, gravity: Toast.bottom);
                        return;
                      }

                      gestureTransform.value = matrix;
                      gestureScaleValue_.value = scaleValue;
                    },
                    child: ValueListenableBuilder<RobotPose>(
                        valueListenable:
                            Provider.of<RosChannel>(context, listen: false)
                                .robotPoseScene,
                        builder: (context, robotPoseScene, child) {
                          double scaleValue = gestureScaleValue_.value;
                          var globalTransform = gestureTransform.value;
                          var originPose = Offset.zero;
                          if (Provider.of<GlobalState>(context, listen: false)
                                  .mode
                                  .value ==
                              Mode.robotFixedCenter) {
                            scaleValue = cameraFixedScaleValue_;
                            globalTransform = Matrix4.identity()
                              ..translate(screenCenter.dx - robotPoseScene.x,
                                  screenCenter.dy - robotPoseScene.y)
                              ..rotateZ(robotPoseScene.theta - deg2rad(90))
                              ..scale(scaleValue);
                            originPose =
                                Offset(robotPoseScene.x, robotPoseScene.y);
                          }

                          return Stack(
                            children: [
                              //网格
                              Container(
                                child: DisplayGrid(
                                  step: (1 /
                                          Provider.of<RosChannel>(context)
                                              .map
                                              .value
                                              .mapConfig
                                              .resolution) *
                                      (scaleValue > 0.8 ? scaleValue : 0.8),
                                  width: screenSize.width,
                                  height: screenSize.height,
                                ),
                              ),
                              //地图
                              Transform(
                                transform: globalTransform,
                                origin: originPose,
                                child: GestureDetector(
                                  child: const DisplayMap(),
                                  onTapDown: (details) {
                                    if (Provider.of<GlobalState>(context,
                                                listen: false)
                                            .mode
                                            .value ==
                                        Mode.addNavPoint) {
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
                                transform: globalTransform,
                                origin: originPose,
                                child: RepaintBoundary(
                                  child: ValueListenableBuilder<List<Offset>>(
                                    valueListenable: Provider.of<RosChannel>(
                                            context,
                                            listen: false)
                                        .globalPath,
                                    builder: (context, path, child) {
                                      return Container(
                                        child: CustomPaint(
                                          painter: DisplayPath(
                                              pointList: path,
                                              color: Colors.green),
                                        ),
                                      );
                                    },
                                  ),
                                ),
                              ),
                              //局部路径
                              Transform(
                                transform: globalTransform,
                                origin: originPose,
                                child: RepaintBoundary(
                                    child: ValueListenableBuilder<List<Offset>>(
                                  valueListenable: Provider.of<RosChannel>(
                                          context,
                                          listen: false)
                                      .localPath,
                                  builder: (context, path, child) {
                                    return Container(
                                      child: CustomPaint(
                                        painter: DisplayPath(
                                            pointList: path,
                                            color: Colors.yellow[200]!),
                                      ),
                                    );
                                  },
                                )),
                              ),

                              //激光
                              Transform(
                                transform: globalTransform,
                                origin: originPose,
                                child: RepaintBoundary(
                                    child: ValueListenableBuilder<LaserData>(
                                        valueListenable:
                                            Provider.of<RosChannel>(context,
                                                    listen: false)
                                                .laserPointData,
                                        builder: (context, laserData, child) {
                                          RobotPose robotPoseMap =
                                              laserData.robotPose;
                                          var map = Provider.of<RosChannel>(
                                                  context,
                                                  listen: false)
                                              .map
                                              .value;
                                          //重定位模式 从图层坐标转换
                                          if (Provider.of<GlobalState>(context,
                                                      listen: false)
                                                  .mode
                                                  .value ==
                                              Mode.reloc) {
                                            Offset poseMap = map.idx2xy(Offset(
                                                poseSceneOnReloc.x,
                                                poseSceneOnReloc.y));
                                            robotPoseMap = RobotPose(
                                                poseMap.dx,
                                                poseMap.dy,
                                                poseSceneOnReloc.theta);
                                          }

                                          List<Offset> laserPointsScene = [];
                                          for (var point
                                              in laserData.laserPoseBaseLink) {
                                            RobotPose pointMap = absoluteSum(
                                                robotPoseMap,
                                                RobotPose(
                                                    point.dx, point.dy, 0));
                                            Offset pointScene = map.xy2idx(
                                                Offset(pointMap.x, pointMap.y));
                                            laserPointsScene.add(pointScene);
                                          }
                                          return IgnorePointer(
                                              ignoring: true,
                                              child: DisplayLaser(
                                                  pointList: laserPointsScene));
                                        })),
                              ),
                              //导航点
                              ...navPointList_.value.map((pose) {
                                return Transform(
                                  transform: globalTransform,
                                  origin: originPose,
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
                                            if (Provider.of<GlobalState>(
                                                        context,
                                                        listen: false)
                                                    .mode
                                                    .value ==
                                                Mode.addNavPoint) {
                                              //移动距离的deleta距离需要除于当前的scale的值(放大后，相同移动距离，地图实际移动的要少)
                                              double dx =
                                                  transDelta.dx / scaleValue;
                                              double dy =
                                                  transDelta.dy / scaleValue;
                                              double tmpTheta = pose.theta;
                                              pose = absoluteSum(
                                                  RobotPose(pose.x, pose.y,
                                                      pose.theta),
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
                                                    visible:
                                                        Provider.of<GlobalState>(
                                                                    context,
                                                                    listen:
                                                                        false)
                                                                .mode
                                                                .value ==
                                                            Mode.addNavPoint,
                                                    child: DisplayPoseDirection(
                                                      size: navPoseSize +
                                                          poseDirectionSwellSize,
                                                      initAngle: -pose.theta,
                                                      resetAngle: false,
                                                      onRotateCallback:
                                                          (angle) {
                                                        pose.theta = -angle;
                                                        setState(() {});
                                                      },
                                                    )),
                                                GestureDetector(
                                                    onTapDown: (details) {
                                                      if (Provider.of<GlobalState>(
                                                                  context,
                                                                  listen: false)
                                                              .mode
                                                              .value ==
                                                          Mode.noraml) {
                                                        // _showContextMenu(context,
                                                        //     details.globalPosition);
                                                        Provider.of<RosChannel>(
                                                                context,
                                                                listen: false)
                                                            .sendNavigationGoal(
                                                                RobotPose(
                                                                    pose.x,
                                                                    pose.y,
                                                                    pose.theta));
                                                        currentNavGoal_ = pose;
                                                        setState(() {});
                                                      }
                                                    },
                                                    child: DisplayWayPoint(
                                                      size: navPoseSize,
                                                      color: currentNavGoal_ ==
                                                              pose
                                                          ? Colors.pink
                                                          : Colors.green,
                                                      count: 4,
                                                    )),
                                              ],
                                            ),
                                          ))),
                                );
                              }).toList(),
                              //机器人位置（固定视角）
                              Visibility(
                                visible: Provider.of<GlobalState>(context,
                                            listen: false)
                                        .mode
                                        .value ==
                                    Mode.robotFixedCenter,
                                child: Positioned(
                                  left: screenCenter.dx -
                                      (robotSize / 2 * cameraFixedScaleValue_),
                                  top: screenCenter.dy -
                                      (robotSize / 2 * cameraFixedScaleValue_),
                                  child: Transform(
                                    transform: Matrix4.identity()
                                      ..scale(cameraFixedScaleValue_),
                                    child: DisplayRobot(
                                      direction: deg2rad(-90),
                                      size: robotSize,
                                      color: Colors.blue,
                                      count: 2,
                                    ),
                                  ),
                                ),
                              ),
                              //机器人位置(不固定视角)
                              Visibility(
                                  visible: Provider.of<GlobalState>(context,
                                              listen: false)
                                          .mode
                                          .value !=
                                      Mode.robotFixedCenter,
                                  child: Transform(
                                    transform: Provider.of<GlobalState>(context,
                                                    listen: false)
                                                .mode
                                                .value ==
                                            Mode.robotFixedCenter
                                        ? cameraFixedTransform
                                        : gestureTransform.value,
                                    child: Consumer<RosChannel>(
                                      builder: (context, rosChannel, child) {
                                        if (!(Provider.of<GlobalState>(context,
                                                    listen: false)
                                                .mode
                                                .value ==
                                            Mode.reloc)) {
                                          robotPose_.value = robotPoseScene;
                                        }

                                        return Transform(
                                            alignment: Alignment.center,
                                            transform: Matrix4.identity()
                                              ..translate(
                                                  robotPose_.value.x -
                                                      robotSize / 2 -
                                                      poseDirectionSwellSize /
                                                          2,
                                                  robotPose_.value.y -
                                                      robotSize / 2 -
                                                      poseDirectionSwellSize /
                                                          2)
                                              ..rotateZ(
                                                  -robotPose_.value.theta),
                                            child: MatrixGestureDetector(
                                              onMatrixUpdate: (matrix,
                                                  transDelta,
                                                  scaleDelta,
                                                  rotateDelta) {
                                                if (Provider.of<GlobalState>(
                                                            context,
                                                            listen: false)
                                                        .mode
                                                        .value ==
                                                    Mode.reloc) {
                                                  //获取global的scale值

                                                  //移动距离的deleta距离需要除于当前的scale的值(放大后，相同移动距离，地图实际移动的要少)
                                                  double dx = transDelta.dx /
                                                      scaleValue;
                                                  double dy = transDelta.dy /
                                                      scaleValue;
                                                  double theta =
                                                      poseSceneOnReloc.theta;
                                                  poseSceneOnReloc =
                                                      absoluteSum(
                                                          RobotPose(
                                                              poseSceneOnReloc
                                                                  .x,
                                                              poseSceneOnReloc
                                                                  .y,
                                                              0),
                                                          RobotPose(dx, dy, 0));
                                                  poseSceneOnReloc.theta =
                                                      theta;
                                                  //坐标变换sum
                                                  robotPose_.value =
                                                      poseSceneOnReloc;
                                                }
                                              },
                                              child: Container(
                                                height: robotSize +
                                                    poseDirectionSwellSize,
                                                width: robotSize +
                                                    poseDirectionSwellSize,
                                                child: Stack(
                                                  alignment: Alignment.center,
                                                  children: [
                                                    //重定位旋转框
                                                    Visibility(
                                                      visible:
                                                          Provider.of<GlobalState>(
                                                                      context,
                                                                      listen:
                                                                          false)
                                                                  .mode
                                                                  .value ==
                                                              Mode.reloc,
                                                      child:
                                                          DisplayPoseDirection(
                                                        size: robotSize +
                                                            poseDirectionSwellSize,
                                                        resetAngle: Provider.of<
                                                                        GlobalState>(
                                                                    context,
                                                                    listen:
                                                                        false)
                                                                .mode
                                                                .value !=
                                                            Mode.reloc,
                                                        onRotateCallback:
                                                            (angle) {
                                                          poseSceneOnReloc
                                                                  .theta =
                                                              (poseSceneStartReloc
                                                                      .theta -
                                                                  angle);
                                                          //坐标变换sum
                                                          robotPose_.value =
                                                              RobotPose(
                                                                  poseSceneOnReloc
                                                                      .x,
                                                                  poseSceneOnReloc
                                                                      .y,
                                                                  poseSceneOnReloc
                                                                      .theta);
                                                        },
                                                      ),
                                                    ),

                                                    //机器人图标
                                                    DisplayRobot(
                                                      size: robotSize,
                                                      color: Colors.blue,
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
                                  )),
                            ],
                          );
                        }),
                  ));
            },
          ),

          //菜单栏
          Positioned(
              left: 5,
              top: 1,
              child: Container(
                height: 50,
                padding: const EdgeInsets.symmetric(vertical: 8),
                child: SingleChildScrollView(
                  scrollDirection: Axis.horizontal, // 水平滚动
                  child: Row(
                    children: [
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 4.0),
                        child: RawChip(
                          avatar: Icon(
                            const IconData(0xe606, fontFamily: "Speed"),
                            color: Colors.green[400],
                          ), // 图标放在文本前
                          label: Text(
                              '${(Provider.of<RosChannel>(context, listen: true).robotSpeed.vx).toStringAsFixed(2)} m/s'),
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 4.0),
                        child: RawChip(
                          avatar: const Icon(
                              IconData(0xe680, fontFamily: "Speed")), // 图标放在文本前
                          label: Text(
                              '${rad2deg(Provider.of<RosChannel>(context, listen: true).robotSpeed.vw).toStringAsFixed(2)} deg/s'),
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 4.0),
                        child: RawChip(
                          avatar: Icon(
                            const IconData(0xe995, fontFamily: "Battery"),
                            color: Colors.amber[300],
                          ), // 图标放在文本前
                          label: Text(
                              '${rad2deg(Provider.of<RosChannel>(context, listen: true).battery).toStringAsFixed(2)} %'),
                        ),
                      ),
                    ],
                  ),
                ),
              )),
          //左侧工具栏
          Positioned(
              left: 5,
              top: 60,
              child: FittedBox(
                  child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Card(
                    elevation: 10,
                    child: Container(
                      child: Row(
                        children: [
                          IconButton(
                              onPressed: () {
                                if (!(Provider.of<GlobalState>(context,
                                            listen: false)
                                        .mode
                                        .value ==
                                    Mode.reloc)) {
                                  Provider.of<GlobalState>(context,
                                          listen: false)
                                      .mode
                                      .value = Mode.reloc;
                                  poseSceneStartReloc = Provider.of<RosChannel>(
                                          context,
                                          listen: false)
                                      .robotPoseScene
                                      .value;

                                  poseSceneOnReloc = Provider.of<RosChannel>(
                                          context,
                                          listen: false)
                                      .robotPoseScene
                                      .value;
                                  setState(() {});
                                } else {
                                  Provider.of<GlobalState>(context,
                                          listen: false)
                                      .mode
                                      .value = Mode.noraml;
                                }
                                setState(() {});
                              },
                              icon: Icon(
                                const IconData(0xe60f, fontFamily: "Reloc"),
                                color: Provider.of<GlobalState>(context,
                                                listen: false)
                                            .mode
                                            .value ==
                                        Mode.reloc
                                    ? Colors.green
                                    : theme.iconTheme.color,
                              )),
                          Visibility(
                              visible: Provider.of<GlobalState>(context,
                                          listen: false)
                                      .mode
                                      .value ==
                                  Mode.reloc,
                              child: IconButton(
                                  onPressed: () {
                                    Provider.of<GlobalState>(context,
                                            listen: false)
                                        .mode
                                        .value = Mode.noraml;
                                    setState(() {});
                                  },
                                  icon: const Icon(
                                    Icons.close,
                                    color: Colors.red,
                                  ))),
                          Visibility(
                              visible: Provider.of<GlobalState>(context,
                                          listen: false)
                                      .mode
                                      .value ==
                                  Mode.reloc,
                              child: IconButton(
                                  onPressed: () {
                                    Provider.of<GlobalState>(context,
                                            listen: false)
                                        .mode
                                        .value = Mode.noraml;
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
                    elevation: 10,
                    child: IconButton(
                      icon: Icon(
                        const IconData(0xeba1, fontFamily: "NavPoint"),
                        color: (Provider.of<GlobalState>(context, listen: false)
                                    .mode
                                    .value ==
                                Mode.addNavPoint)
                            ? Colors.green
                            : theme.iconTheme.color,
                      ),
                      onPressed: () {
                        if (!(Provider.of<GlobalState>(context, listen: false)
                                .mode
                                .value ==
                            Mode.addNavPoint)) {
                          Provider.of<GlobalState>(context, listen: false)
                              .mode
                              .value = Mode.addNavPoint;
                          setState(() {});
                        } else {
                          Provider.of<GlobalState>(context, listen: false)
                              .mode
                              .value = Mode.noraml;
                          setState(() {});
                        }
                      },
                    ),
                  ),
                  //手动控制
                  Card(
                    elevation: 10,
                    child: IconButton(
                      icon: Icon(const IconData(0xea45, fontFamily: "GamePad"),
                          color: manualCtrlMode_.value
                              ? Colors.green
                              : theme.iconTheme.color),
                      onPressed: () {
                        if (manualCtrlMode_.value) {
                          manualCtrlMode_.value = false;
                          Provider.of<RosChannel>(context, listen: false)
                              .stopMunalCtrl();
                          setState(() {});
                        } else {
                          manualCtrlMode_.value = true;
                          Provider.of<RosChannel>(context, listen: false)
                              .startMunalCtrl();
                          setState(() {});
                        }
                      },
                    ),
                  )
                ],
              ))),
          //左侧顶部状态栏
          // Positioned(
          //     right: 5,
          //     top: 10,
          //     child: Card(
          //       color: Colors.white70,
          //       elevation: 10,
          //       child: Container(
          //         child: Column(
          //           children: [
          //             IconButton(
          //                 onPressed: () {}, icon: const Icon(Icons.layers)),
          //             IconButton(
          //                 onPressed: () {}, icon: const Icon(Icons.podcasts)),
          //             IconButton(
          //                 onPressed: () {},
          //                 icon: Icon(Icons.location_on_outlined))
          //           ],
          //         ),
          //       ),
          //     )),
          //右方菜单栏

          Positioned(
            right: 5,
            top: 30,
            child: FittedBox(
              child: Column(
                children: [
                  IconButton(
                      onPressed: () {
                        if (Provider.of<GlobalState>(context, listen: false)
                                .mode
                                .value ==
                            Mode.robotFixedCenter) {
                          cameraFixedScaleValue_ += 0.3;
                        } else {}
                        setState(() {});
                      },
                      icon: const Icon(Icons.zoom_in)),
                  IconButton(
                      onPressed: () {
                        setState(() {
                          cameraFixedScaleValue_ -= 0.3;
                        });
                      },
                      icon: const Icon(
                        Icons.zoom_out,
                      )),
                  IconButton(
                      onPressed: () {
                        if (Provider.of<GlobalState>(context, listen: false)
                                .mode
                                .value ==
                            Mode.robotFixedCenter) {
                          Provider.of<GlobalState>(context, listen: false)
                              .mode
                              .value = Mode.noraml;
                          cameraFixedScaleValue_ = 1;
                        } else {
                          Provider.of<GlobalState>(context, listen: false)
                              .mode
                              .value = Mode.robotFixedCenter;
                        }
                        if (animationController.isAnimating) return; // 防止多次触发动画

                        animationController.reset(); // 重置动画控制器
                        animationController.forward(); // 启动动画
                        setState(() {});
                      },
                      icon: Icon(Icons.location_searching,
                          color:
                              Provider.of<GlobalState>(context, listen: false)
                                          .mode
                                          .value ==
                                      Mode.robotFixedCenter
                                  ? Colors.green
                                  : theme.iconTheme.color))
                ],
              ),
            ),
          ),

          //左摇杆控制线速度
          Positioned(
            left: 30,
            bottom: 10,
            child: Visibility(
              visible: manualCtrlMode_.value,
              maintainState: true,
              child: Container(
                  width: screenSize.width * 0.2,
                  height: screenSize.width * 0.2,
                  child: Joystick(
                    mode: JoystickMode.all,
                    listener: (details) {
                      if (!manualCtrlMode_.value) return;
                      double max_vx =
                          double.parse(globalSetting.getConfig('MaxVx'));

                      double vx = max_vx * details.y * -1;
                      Provider.of<RosChannel>(context, listen: false).setVx(vx);
                    },
                  )),
            ),
          ),

          //右摇杆控制角速度
          Positioned(
            right: 30,
            bottom: 10,
            child: Visibility(
                visible: manualCtrlMode_.value,
                maintainState: true,
                child: Container(
                    width: screenSize.width * 0.2,
                    height: screenSize.width * 0.2,
                    child: Joystick(
                      mode: JoystickMode.all,
                      listener: (details) {
                        if (!manualCtrlMode_.value) return;

                        double max_vw =
                            double.parse(globalSetting.getConfig('MaxVw'));

                        double vw = max_vw * details.x * -1;
                        Provider.of<RosChannel>(context, listen: false)
                            .setVw(vw);
                      },
                    ))),
          )
        ],
      ),
    );
  }
}
