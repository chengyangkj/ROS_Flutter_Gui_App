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
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/page/gamepad_widget.dart';
import 'package:ros_flutter_gui_app/page/gamepad_widget.dart';
import 'package:ros_flutter_gui_app/basic/math.dart';
import 'package:ros_flutter_gui_app/basic/matrix_gesture_detector.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/display_laser.dart';
import 'package:ros_flutter_gui_app/display/display_path.dart';
import 'package:ros_flutter_gui_app/display/display_robot.dart';
import 'package:ros_flutter_gui_app/display/display_pose_direction.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/display/display_map.dart';
import 'package:ros_flutter_gui_app/display/display_grid.dart';
import 'package:toast/toast.dart';
import 'package:vector_math/vector_math_64.dart' as vector;
import 'package:ros_flutter_gui_app/display/display_waypoint.dart';
import 'package:ros_flutter_gui_app/display/display_polygon.dart';
import 'package:ros_flutter_gui_app/display/display_costmap.dart';
import 'package:ros_flutter_gui_app/display/display_pointcloud.dart';
import 'package:ros_flutter_gui_app/basic/pointcloud2.dart';

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPageState();
}

class _MapPageState extends State<MapPage> with SingleTickerProviderStateMixin {
  ValueNotifier<bool> manualCtrlMode_ = ValueNotifier(false);
  ValueNotifier<TopologyMap> navPointList_ =
      ValueNotifier<TopologyMap>(TopologyMap(points: []));
  final ValueNotifier<Matrix4> gestureTransform =
      ValueNotifier(Matrix4.identity());

  bool showCamera = false;
  bool showCostMap = true; // 控制代价地图显示
  
  // 图层开关状态
  bool showLayerControl = false; // 控制图层开关面板的展开/收起

  Offset camPosition = Offset(30, 10); // 初始位置
  bool isCamFullscreen = false; // 是否全屏
  Offset camPreviousPosition = Offset(30, 10); // 保存进入全屏前的位置
  late double camWidgetWidth;
  late double camWidgetHeight;

  Matrix4 cameraFixedTransform = Matrix4.identity(); //固定相机视角(以机器人为中心)
  double cameraFixedScaleValue_ = 1; //固定相机视角时的缩放值

  late AnimationController animationController;
  late Animation<double> animationValue;

  final ValueNotifier<RobotPose> robotPose_ = ValueNotifier(RobotPose(0, 0, 0));
  final ValueNotifier<double> gestureScaleValue_ = ValueNotifier(1);
  OverlayEntry? _overlayEntry;
  RobotPose currentNavGoal_ = RobotPose.zero();
  // 定义一个变量用于表示是否达到目标点
  final ValueNotifier<bool> hasReachedGoal_ = ValueNotifier(false);
  final ValueNotifier<double> currentRobotSpeed_ = ValueNotifier(0);

  bool isLandscape = false; // 用于跟踪屏幕方向
  int poseDirectionSwellSize = 10; //机器人方向旋转控件膨胀的大小
  double navPoseSize = 10; //导航点的大小
  double robotSize = 10; //机器人坐标的大小
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
          }); // 监听 robotPose_ 的变化，判断是否达到目标点
    robotPose_.addListener(() {
      // 获取机器人当前速度
      double distance = calculateDistance(robotPose_.value, currentNavGoal_);

      // 如果距离小于0.5，判断速度为0时候， 表示已达到目标点
      if (currentRobotSpeed_.value < 0.001 &&
          rad2deg(currentRobotSpeed_.value) < 0.01) {
        hasReachedGoal_.value = true;
      } else {
        hasReachedGoal_.value = false;
      }

      // print(currentRobotSpeed_.value);
      // print(rad2deg(currentRobotSpeed_.value));
      // print(robotPose_);
      // print(currentNavGoal_);
      // print(hasReachedGoal_.value);
    });
  }

// 计算两点之间的距离的方法
  double calculateDistance(RobotPose pose1, RobotPose pose2) {
    double dx = pose1.x - pose2.x;
    double dy = pose1.y - pose2.y;
    return sqrt(dx * dx + dy * dy);
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

  // 示例：执行单点导航的函数
  void executeNavigation(RobotPose pose) {
    // 创建 pose 的副本，避免修改原始值
    RobotPose targetPose = RobotPose(pose.x, pose.y, pose.theta);

    // print(targetPose); // 打印副本以确保不改变原始 pose
    Provider.of<RosChannel>(context, listen: false)
        .sendNavigationGoal(targetPose);
    print("执行单点导航到: (${pose.x}, ${pose.y}, ${pose.theta})");

    currentNavGoal_ = pose;
  }

  // 等待函数，当 hasReachedGoal_ 变为 true 时返回
  Future<void> waitForGoalReached() async {
    // 使用 Completer 实现异步等待
    Completer<void> completer = Completer<void>();

    // 创建监听器
    void listener() {
      if (hasReachedGoal_.value) {
        completer.complete(); // 达到目标点，完成等待
      }
    }

    // 添加监听器
    hasReachedGoal_.addListener(listener);

    // 等待完成
    await completer.future;

    // 移除监听器，防止内存泄漏
    hasReachedGoal_.removeListener(listener);
  }
  
  // 保存图层配置
  void _saveLayerConfig() {
    globalSetting.setShowGlobalCostmap(Provider.of<GlobalState>(context, listen: false).showGlobalCostmap.value);
    globalSetting.setShowLocalCostmap(Provider.of<GlobalState>(context, listen: false).showLocalCostmap.value);
    globalSetting.setShowLaser(Provider.of<GlobalState>(context, listen: false).showLaser.value);
    globalSetting.setShowPointCloud(Provider.of<GlobalState>(context, listen: false).showPointCloud.value);
  }

  @override
  Widget build(BuildContext context) {
    ToastContext().init(context);
    final _key = GlobalKey<ExpandableFabState>();
    final screenSize = MediaQuery.of(context).size;
    final screenCenter = Offset(screenSize.width / 2, screenSize.height / 2);
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;
    
    // 初始化图层开关状态（只在第一次构建时）
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      final globalState = Provider.of<GlobalState>(context, listen: false);
      if (!globalState.showGlobalCostmap.value && !globalState.showLocalCostmap.value && 
          !globalState.showLaser.value && !globalState.showPointCloud.value) {
        globalState.showGlobalCostmap.value = globalSetting.showGlobalCostmap;
        globalState.showLocalCostmap.value = globalSetting.showLocalCostmap;
        globalState.showLaser.value = globalSetting.showLaser;
        globalState.showPointCloud.value = globalSetting.showPointCloud;
      }
    });
    camWidgetWidth = screenSize.width / 3.5;
    camWidgetHeight =
        camWidgetWidth / (globalSetting.imageWidth / globalSetting.imageHeight);
    camWidgetWidth = screenSize.width / 3.5;
    camWidgetHeight =
        camWidgetWidth / (globalSetting.imageWidth / globalSetting.imageHeight);

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
                                                           //全局代价地图
                              ValueListenableBuilder<bool>(
                                valueListenable: Provider.of<GlobalState>(context, listen: false).showGlobalCostmap,
                                builder: (context, showGlobalCostmap, child) {
                                  return Visibility(
                                    visible: showGlobalCostmap,
                                    child: Transform(
                                      transform: globalTransform,
                                      origin: Offset.zero,
                                      child: IgnorePointer(
                                        ignoring: true,
                                        child: DisplayCostMap(
                                          opacity: 0.4,
                                          isGlobal: true,
                                        ),
                                      ),
                                    ),
                                  );
                                },
                              ),
                              //局部代价地图
                              ValueListenableBuilder<bool>(
                                valueListenable: Provider.of<GlobalState>(context, listen: false).showLocalCostmap,
                                builder: (context, showLocalCostmap, child) {
                                  return Visibility(
                                    visible: showLocalCostmap,
                                    child: Transform(
                                      transform: globalTransform,
                                      origin: Offset.zero,
                                      child: DisplayCostMap(
                                        opacity: 0.6,
                                        isGlobal: false,
                                      ),
                                    ),
                                  );
                                },
                              ),
                      
                              //地图
                              Transform(
                                transform: globalTransform,
                                origin: originPose,
                                child: GestureDetector(
                                  child: const DisplayMap(),
                                  onTapDown: (details) {
                                    print("addNavPoint tapDown");
                                    if (Provider.of<GlobalState>(context,
                                                listen: false)
                                            .mode
                                            .value ==
                                        Mode.addNavPoint) {
                                      // 生成基础名称
                                      String baseName = "navGoal";

                                      // 查找已存在的同名点
                                      int suffix = 0;
                                      String newName = baseName;

                                      // 检查是否存在同名点，如果存在则增加后缀
                                      while (navPointList_.value.points.any(
                                          (point) => point.name == newName)) {
                                        suffix++;
                                        newName = "${baseName}_$suffix";
                                      }

                                      // 添加新的导航点
                                      navPointList_.value.points.add(NavPoint(
                                          x: details.localPosition.dx,
                                          y: details.localPosition.dy,
                                          theta: 0,
                                          name: newName,
                                          type: NavPointType.navGoal));

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
                              ValueListenableBuilder<bool>(
                                valueListenable: Provider.of<GlobalState>(context, listen: false).showLaser,
                                builder: (context, showLaser, child) {
                                  return Visibility(
                                    visible: showLaser,
                                    child: Transform(
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
                                  );
                                },
                              ),
                              // 点云显示
                              ValueListenableBuilder<bool>(
                                valueListenable: Provider.of<GlobalState>(context, listen: false).showPointCloud,
                                builder: (context, showPointCloud, child) {
                                  return Visibility(
                                    visible: showPointCloud,
                                    child: Transform(
                                      transform: globalTransform,
                                      origin: originPose,
                                      child: RepaintBoundary(
                                        child: ValueListenableBuilder<OccupancyMap>(
                                          valueListenable: Provider.of<RosChannel>(
                                                  context,
                                                  listen: false)
                                              .map,
                                          builder: (context, map, child) {
                                            return ValueListenableBuilder<List<Point3D>>(
                                              valueListenable: Provider.of<RosChannel>(
                                                      context,
                                                      listen: false)
                                                  .pointCloud2Data,
                                              builder: (context, pointCloudData, child) {
                                                return IgnorePointer(
                                                    ignoring: true,
                                                    child: DisplayPointCloud(
                                                        pointList: pointCloudData,
                                                        map: map));
                                              },
                                            );
                                          },
                                        ),
                                      ),
                                    ),
                                  );
                                },
                              ),
                              // //机器人足迹多边形
                              Transform(
                                transform: globalTransform,
                                origin: originPose,
                                child: RepaintBoundary(
                                  child: ValueListenableBuilder<List<Offset>>(
                                    valueListenable: Provider.of<RosChannel>(
                                            context,
                                            listen: false)
                                        .robotFootprint,
                                    builder: (context, footprint, child) {
              
                                      return  DisplayPolygon(
                                          pointList: footprint,
                                          color: Colors.blue,
                                          fill: true,
                                          enableWaterDropAnimation: true,
                                        ) ;
                                    },
                                  ),
                                ),
                              ),
                              ValueListenableBuilder<TopologyMap>(
                                valueListenable: Provider.of<RosChannel>(
                                        context,
                                        listen: false)
                                    .topologyMap_,
                                builder: (context, topologyMap, child) {
                                  // 合并更新navPointList_，已有的保持不变，只新增
                                  List<NavPoint> mergedPoints = List.from(navPointList_.value.points);
                                  
                                  for (var newPoint in topologyMap.points) {
                                    // 检查是否已存在相同的点（坐标相同）
                                    bool pointExists = mergedPoints.any((existingPoint) =>
                                        (existingPoint.x - newPoint.x).abs() < 0.001 &&
                                        (existingPoint.y - newPoint.y).abs() < 0.001 &&
                                        (existingPoint.theta - newPoint.theta).abs() < 0.001);
                                    
                                    // 如果不存在，则添加新点
                                    if (!pointExists) {
                                      mergedPoints.add(newPoint);
                                    }
                                  }
                                  
                                  // 更新导航点列表
                                  navPointList_.value = TopologyMap(points: mergedPoints);
                                  
                                  return Container();
                                },
                              ),
                              //导航点
                              ...navPointList_.value.points.map((pose) {
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
                                        if (Provider.of<GlobalState>(context,
                                                    listen: false)
                                                .mode
                                                .value ==
                                            Mode.addNavPoint) {
                                          // 移动距离的delta距离需要除于当前的scale的值
                                          double dx =
                                              transDelta.dx / scaleValue;
                                          double dy =
                                              transDelta.dy / scaleValue;
                                          double tmpTheta = pose.theta;
                                          RobotPose poseRobot = absoluteSum(
                                              RobotPose(
                                                  pose.x, pose.y, pose.theta),
                                              RobotPose(dx, dy, 0));
                                          pose.theta = tmpTheta;
                                          pose.x = poseRobot.x;
                                          pose.y = poseRobot.y;
                                          setState(() {});
                                        }
                                      },
                                      child: GestureDetector(
                                        onDoubleTap: () {
                                          if (Provider.of<GlobalState>(context,
                                                      listen: false)
                                                  .mode
                                                  .value ==
                                              Mode.addNavPoint) {
                                            // 双击删除导航点
                                            navPointList_.value.points =
                                                List.from(
                                                    navPointList_.value.points)
                                                  ..remove(pose);
                                            setState(() {});
                                          }
                                        },
                                        onPanUpdate: (details) {
                                          if (Provider.of<GlobalState>(context,
                                                      listen: false)
                                                  .mode
                                                  .value ==
                                              Mode.addNavPoint) {
                                            // 更新导航点位置
                                            double dx =
                                                details.delta.dx / scaleValue;
                                            double dy =
                                                details.delta.dy / scaleValue;
                                            pose.x += dx;
                                            pose.y += dy;
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
                                                                listen: false)
                                                            .mode
                                                            .value ==
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
                                                ),
                                              ),
                                              GestureDetector(
                                                onTapDown: (details) {
                                                  if (Provider.of<GlobalState>(
                                                              context,
                                                              listen: false)
                                                          .mode
                                                          .value ==
                                                      Mode.normal) {
                                                    Provider.of<RosChannel>(
                                                            context,
                                                            listen: false)
                                                        .sendNavigationGoal(
                                                            RobotPose(
                                                                pose.x,
                                                                pose.y,
                                                                pose.theta));
                                                    currentNavGoal_ = RobotPose(
                                                        pose.x,
                                                        pose.y,
                                                        pose.theta);
                                                    setState(() {});
                                                  }
                                                },
                                                child: DisplayWayPoint(
                                                  size: navPoseSize,
                                                  color: currentNavGoal_ == pose
                                                      ? Colors.pink
                                                      : Colors.green,
                                                  count: 4,
                                                ),
                                              ),
                                            ],
                                          ),
                                        ),
                                      ),
                                    ),
                                  ),
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
                                      size: robotSize,
                                      color: Colors.blue,
                                      direction: deg2rad(-45),
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
                                                      direction: deg2rad(45),
                                                    ),
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
                          label: ValueListenableBuilder<RobotSpeed>(
                              valueListenable:
                                  Provider.of<RosChannel>(context, listen: true)
                                      .robotSpeed_,
                              builder: (context, speed, child) {
                                currentRobotSpeed_.value = speed.vx;
                                // print("speed.vx:${speed.vx}");
                                return Text(
                                    '${(speed.vx).toStringAsFixed(2)} m/s');
                              }),
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 4.0),
                        child: RawChip(
                          avatar: const Icon(
                              IconData(0xe680, fontFamily: "Speed")), // 图标放在文本前
                          label: ValueListenableBuilder<RobotSpeed>(
                              valueListenable:
                                  Provider.of<RosChannel>(context, listen: true)
                                      .robotSpeed_,
                              builder: (context, speed, child) {
                                return Text(
                                    '${rad2deg(speed.vx).toStringAsFixed(2)} deg/s');
                              }),
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 4.0),
                        child: RawChip(
                          avatar: Icon(
                            const IconData(0xe995, fontFamily: "Battery"),
                            color: Colors.amber[300],
                          ), // 图标放在文本前
                          label: ValueListenableBuilder<double>(
                              valueListenable: Provider.of<RosChannel>(context,
                                      listen: false)
                                  .battery_,
                              builder: (context, battery, child) {
                                return Text('${battery.toStringAsFixed(0)} %');
                              }),
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 4.0),
                        child: RawChip(
                          avatar: const Icon(Icons.navigation,
                              color: Colors.green, size: 16), // 图标放在文本前
                          label: ValueListenableBuilder<ActionStatus>(
                              valueListenable:
                                  Provider.of<RosChannel>(context, listen: true)
                                      .navStatus_,
                              builder: (context, navStatus, child) {
                                return Text('${navStatus.toString()}');
                              }),
                        ),
                      ),
                    ],
                  ),
                ),
              )),
          //图像
          Visibility(
            visible: showCamera, // 根据需要显示或隐藏
            child: Positioned(
              left: camPosition.dx,
              top: camPosition.dy,
              child: GestureDetector(
                onPanUpdate: (details) {
                  if (!isCamFullscreen) {
                    setState(() {
                      double newX = camPosition.dx + details.delta.dx;
                      double newY = camPosition.dy + details.delta.dy;
                      // 限制位置在屏幕范围内
                      newX = newX.clamp(0.0, screenSize.width - camWidgetWidth);
                      newY =
                          newY.clamp(0.0, screenSize.height - camWidgetHeight);
                      camPosition = Offset(newX, newY);
                    });
                  }
                },
                child: Container(
                  child: Stack(
                    children: [
                      LayoutBuilder(
                        builder: (context, constraints) {
                          // 在非全屏状态下，获取屏幕宽高
                          double containerWidth = isCamFullscreen
                              ? screenSize.width
                              : camWidgetWidth;
                          double containerHeight = isCamFullscreen
                              ? screenSize.height
                              : camWidgetHeight;

                          return Mjpeg(
                            stream:
                                'http://${globalSetting.robotIp}:${globalSetting.imagePort}/stream?topic=${globalSetting.imageTopic}',
                            isLive: true,
                            width: containerWidth,
                            height: containerHeight,
                            fit: BoxFit.fill,
                            // BoxFit.fill：拉伸填充满容器，可能会改变图片的宽高比。
                            // BoxFit.contain：按照图片的原始比例缩放，直到一边填满容器。
                            // BoxFit.cover：按照图片的原始比例缩放，直到容器被填满，可能会裁剪图片。
                          );
                        },
                      ),
                      Positioned(
                        right: 0,
                        top: 0,
                        child: IconButton(
                          icon: Icon(
                            isCamFullscreen
                                ? Icons.fullscreen_exit
                                : Icons.fullscreen,
                            color: Colors.black,
                          ),
                          constraints: BoxConstraints(), // 移除按钮的默认大小约束，变得更加紧凑
                          onPressed: () {
                            setState(() {
                              isCamFullscreen = !isCamFullscreen;
                              if (isCamFullscreen) {
                                // 进入全屏时，保存当前位置，并将位置设为 (0, 0)
                                camPreviousPosition = camPosition;
                                camPosition = Offset(0, 0);
                              } else {
                                // 退出全屏时，恢复之前的位置
                                camPosition = camPreviousPosition;
                              }
                            });
                          },
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
          //左侧工具栏
          Positioned(
              left: 5,
              top: 60,
              child: FittedBox(
                  child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  //图层开关控制
                  Card(
                    elevation: 10,
                    child: Container(
                      child: Row(
                        children: [
                          IconButton(
                            icon: Icon(Icons.layers),
                            color: showLayerControl ? Colors.green : theme.iconTheme.color,
                            onPressed: () {
                              setState(() {
                                showLayerControl = !showLayerControl;
                              });
                            },
                          ),
                          if (showLayerControl) ...[
                            // 全局代价地图开关
                            ValueListenableBuilder<bool>(
                              valueListenable: Provider.of<GlobalState>(context, listen: false).showGlobalCostmap,
                              builder: (context, showGlobalCostmap, child) {
                                return IconButton(
                                  icon: Icon(Icons.map, size: 20),
                                  color: showGlobalCostmap ? Colors.green : Colors.grey,
                                  onPressed: () {
                                    Provider.of<GlobalState>(context, listen: false).showGlobalCostmap.value = !showGlobalCostmap;
                                    _saveLayerConfig();
                                  },
                                );
                              },
                            ),
                            // 局部代价地图开关
                            ValueListenableBuilder<bool>(
                              valueListenable: Provider.of<GlobalState>(context, listen: false).showLocalCostmap,
                              builder: (context, showLocalCostmap, child) {
                                return IconButton(
                                  icon: Icon(Icons.map_outlined, size: 20),
                                  color: showLocalCostmap ? Colors.green : Colors.grey,
                                  onPressed: () {
                                    Provider.of<GlobalState>(context, listen: false).showLocalCostmap.value = !showLocalCostmap;
                                    _saveLayerConfig();
                                  },
                                );
                              },
                            ),
                            // 激光开关
                            ValueListenableBuilder<bool>(
                              valueListenable: Provider.of<GlobalState>(context, listen: false).showLaser,
                              builder: (context, showLaser, child) {
                                return IconButton(
                                  icon: Icon(Icons.radar, size: 20),
                                  color: showLaser ? Colors.green : Colors.grey,
                                  onPressed: () {
                                    Provider.of<GlobalState>(context, listen: false).showLaser.value = !showLaser;
                                    _saveLayerConfig();
                                  },
                                );
                              },
                            ),
                            // 点云开关
                            ValueListenableBuilder<bool>(
                              valueListenable: Provider.of<GlobalState>(context, listen: false).showPointCloud,
                              builder: (context, showPointCloud, child) {
                                return IconButton(
                                  icon: Icon(Icons.cloud, size: 20),
                                  color: showPointCloud ? Colors.green : Colors.grey,
                                  onPressed: () {
                                    Provider.of<GlobalState>(context, listen: false).showPointCloud.value = !showPointCloud;
                                    _saveLayerConfig();
                                  },
                                );
                              },
                            ),
                          ],
                        ],
                      ),
                    ),
                  ),
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
                                      .value = Mode.normal;
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
                                        .value = Mode.normal;
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
                                        .value = Mode.normal;
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
                    child:  IconButton(
                        icon: Icon(
                          const IconData(0xeba1, fontFamily: "NavPoint"),
                          color:
                              (Provider.of<GlobalState>(context, listen: false)
                                          .mode
                                          .value ==
                                      Mode.addNavPoint)
                                  ? Colors.green
                                  : theme.iconTheme.color,
                        ),
                        onPressed: () {
                          var globalState =
                              Provider.of<GlobalState>(context, listen: false);
                          if (globalState.mode.value == Mode.addNavPoint) {
                            globalState.mode.value = Mode.normal;
                          } else {
                            globalState.mode.value = Mode.addNavPoint;
                          }
                          setState(() {});
                        },
                      ),
                    ),
                  //显示相机图像
                  Card(
                    elevation: 10,
                    child: IconButton(
                      icon: Icon(Icons.camera_alt),
                      color: showCamera ? Colors.green : theme.iconTheme.color,
                      onPressed: () {
                        showCamera = !showCamera;
                        setState(() {});
                      },
                    ),
                  ),

                  //手动控制
                  Card(
                    elevation: 10,
                    child: IconButton(
                      icon: Icon(const IconData(0xea45, fontFamily: "GamePad"),
                          color:
                              Provider.of<GlobalState>(context, listen: false)
                                      .isManualCtrl
                                      .value
                                  ? Colors.green
                                  : theme.iconTheme.color),
                      onPressed: () {
                        if (Provider.of<GlobalState>(context, listen: false)
                            .isManualCtrl
                            .value) {
                          Provider.of<GlobalState>(context, listen: false)
                              .isManualCtrl
                              .value = false;
                          Provider.of<RosChannel>(context, listen: false)
                              .stopMunalCtrl();
                          setState(() {});
                        } else {
                          Provider.of<GlobalState>(context, listen: false)
                              .isManualCtrl
                              .value = true;
                          Provider.of<RosChannel>(context, listen: false)
                              .startMunalCtrl();
                          setState(() {});
                        }
                      },
                    ),
                  ),
                
                ],
              ))),
          //底部状态栏
          Visibility(
              visible: !Provider.of<GlobalState>(context, listen: false)
                  .isManualCtrl
                  .value,
              child: Positioned(
                left: 5,
                bottom: 10,
                child: Container(
                  child: Row(
                    children: [
                      Card(
                        color: Colors.red,
                        child: Container(
                          width: 100,
                          height: 50,
                          child: TextButton(
                            child: const Text(
                              "STOP",
                              style: TextStyle(color: Colors.white),
                            ),
                            onPressed: () {
                              // 发送急停命令
                              Provider.of<RosChannel>(context, listen: false)
                                  .sendEmergencyStop();
                              // 显示提示
                              Toast.show("已触发急停！",
                                  duration: Toast.lengthShort,
                                  gravity: Toast.bottom);
                            },
                          ),
                        ),
                      ),
                      Container(
                        child: ValueListenableBuilder<ActionStatus>(
                            valueListenable:
                                Provider.of<RosChannel>(context, listen: true)
                                    .navStatus_,
                            builder: (context, navStatus, child) {
                              return Visibility(
                                visible: navStatus == ActionStatus.executing ||
                                    navStatus == ActionStatus.accepted,
                                child: Card(
                                  color: Colors.blue,
                                  child: Container(
                                    width: 50,
                                    height: 50,
                                    child: IconButton(
                                      icon: const Icon(
                                        Icons.stop_circle, // 使用警告图标
                                        size: 30,
                                        color: Colors.white,
                                      ),
                                      onPressed: () {
                                        // 发送停止导航指令
                                        Provider.of<RosChannel>(context,
                                                listen: false)
                                            .sendCancelNav();
                                        // 显示提示
                                        Toast.show("stop nav",
                                            duration: Toast.lengthShort,
                                            gravity: Toast.bottom);
                                      },
                                    ),
                                  ),
                                ),
                              );
                            }),
                      ),
                    ],
                  ),
                ),
              )),

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
                              .value = Mode.normal;
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
                                  : theme.iconTheme.color)),
                  IconButton(
                    onPressed: () {
                      // 退出操作
                      Navigator.pop(context); // 返回到上一个页面
                      // Provider.of<RosChannel>(context, listen: false).closeConnection();
                    },
                    icon: const Icon(Icons.exit_to_app),
                    tooltip: '退出',
                  ),
                ],
              ),
            ),
          ),
          Visibility(
            child: GamepadWidget(),
          )
        ],
      ),
    );
  }
}
