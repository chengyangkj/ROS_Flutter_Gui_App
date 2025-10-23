import 'dart:math';
import 'dart:typed_data';
import 'package:vector_math/vector_math_64.dart' as vm;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/basic/robot_path.dart';
import 'package:ros_flutter_gui_app/basic/tf2_dart.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/basic/transform.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ros_bridge_player.dart';
import 'package:roslibdart/roslibdart.dart';
import 'dart:async';
import "package:ros_flutter_gui_app/basic/occupancy_map.dart";
import 'package:ros_flutter_gui_app/basic/tf.dart';
import 'package:ros_flutter_gui_app/basic/laser_scan.dart';
import "package:ros_flutter_gui_app/basic/math.dart";
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/polygon_stamped.dart';
import 'package:ros_flutter_gui_app/basic/pointcloud2.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_array.dart';
import 'package:ros_flutter_gui_app/provider/diagnostic_manager.dart';
import 'package:oktoast/oktoast.dart';


class LaserData {
  RobotPose robotPose;
  List<vm.Vector2> laserPoseBaseLink;
  LaserData({required this.robotPose, required this.laserPoseBaseLink});
}

class RobotSpeed {
  double vx;
  double vy;
  double vw;
  RobotSpeed({required this.vx, required this.vy, required this.vw});
}

class RosChannel {
  late RosBridgePlayer rosBridgePlayer;

  String rosUrl_ = "";
  Timer? cmdVelTimer;
  bool isReconnect_ = false;

  bool manualCtrlMode_ = false;
  ValueNotifier<double> battery_ = ValueNotifier(78);
  ValueNotifier<Uint8List> imageData = ValueNotifier(Uint8List(0));
  RobotSpeed cmdVel_ = RobotSpeed(vx: 0, vy: 0, vw: 0);
  ValueNotifier<RobotSpeed> robotSpeed_ =
      ValueNotifier(RobotSpeed(vx: 0, vy: 0, vw: 0));
  String url_ = "";
  TF2Dart tf_ = TF2Dart();
  ValueNotifier<OccupancyMap> map_ =
      ValueNotifier<OccupancyMap>(OccupancyMap());
  ValueNotifier<TopologyMap> topologyMap_ =
      ValueNotifier<TopologyMap>(TopologyMap(points: []));
  Status rosConnectState_ = Status.none;
  ValueNotifier<RobotPose> robotPoseMap = ValueNotifier(RobotPose.zero());
  ValueNotifier<RobotPose> robotPoseScene = ValueNotifier(RobotPose.zero());
  ValueNotifier<List<vm.Vector2>> laserBasePoint_ = ValueNotifier([]);
  ValueNotifier<List<vm.Vector2>> localPath = ValueNotifier([]);
  ValueNotifier<List<vm.Vector2>> globalPath = ValueNotifier([]);
  ValueNotifier<List<vm.Vector2>> tracePath = ValueNotifier([]);
  ValueNotifier<LaserData> laserPointData = ValueNotifier(
      LaserData(robotPose: RobotPose(0, 0, 0), laserPoseBaseLink: []));
  ValueNotifier<ActionStatus> navStatus_ = ValueNotifier(ActionStatus.unknown);
  ValueNotifier<List<vm.Vector2>> robotFootprint = ValueNotifier([]);
  ValueNotifier<OccupancyMap> localCostmap = ValueNotifier(OccupancyMap());
  ValueNotifier<OccupancyMap> globalCostmap = ValueNotifier(OccupancyMap());
  ValueNotifier<List<Point3D>> pointCloud2Data = ValueNotifier([]);
  ValueNotifier<DiagnosticArray> diagnosticData = ValueNotifier(DiagnosticArray());
  late DiagnosticManager diagnosticManager;


  RosChannel() {
    diagnosticManager = DiagnosticManager();
    
    //启动定时器 获取机器人实时坐标
    globalSetting.init().then((success) {
       //监听链接状态

    //获取机器人实时坐标
    Timer.periodic(const Duration(milliseconds: 50), (timer) {
      if (rosConnectState_ != Status.connected) return;
      try {
        robotPoseMap.value = tf_.lookUpForTransform(
            globalSetting.mapFrameName, globalSetting.baseLinkFrameName);
                        vm.Vector2 poseScene = map_.value
        .xy2idx(vm.Vector2(robotPoseMap.value.x, robotPoseMap.value.y));
    robotPoseScene.value = RobotPose(
        poseScene.x, poseScene.y, robotPoseMap.value.theta);
      } catch (e) {
        print("get robot pose error:${e}");
      }
    });

    //重连
    Timer.periodic(const Duration(seconds: 5), (timer) async {
      if (isReconnect_ && rosConnectState_ != Status.connected){
        showToast(
          "lost connection to ${rosUrl_} try reconnect...",
          position: ToastPosition.bottom,
          backgroundColor: Colors.black.withOpacity(0.8),
          textStyle: const TextStyle( color: Colors.white),
        );
        String error = await connect(rosUrl_);
          if(error.isEmpty){
            showToast(
              "reconnect success to ${rosUrl_}!",
              position: ToastPosition.bottom,
              backgroundColor: Colors.green.withOpacity(0.8),
              textStyle: const TextStyle(color: Colors.white),
            );
          }else{
            showToast(
              "reconnect failed to ${rosUrl_} error: $error",
              position: ToastPosition.bottom,
              backgroundColor: Colors.red.withOpacity(0.8),
              textStyle: const TextStyle( color: Colors.white),
            );
        }
      }
    });
      
    });
  }


  Future<String> connect(String url) async {
    rosUrl_ = url;
    rosConnectState_ = Status.none;
    
    // 创建RosBridgePlayer实例
    rosBridgePlayer = RosBridgePlayer(
      url: url,
      id: DateTime.now().millisecondsSinceEpoch.toString(),
    );

    // 设置状态监听器
    rosBridgePlayer.setListener((presence, topics, datatypes) {
      switch (presence) {
        case PlayerPresence.notPresent:
          rosConnectState_ = Status.none;
          break;
        case PlayerPresence.initializing:
          rosConnectState_ = Status.connecting;
          break;
        case PlayerPresence.present:
          rosConnectState_ = Status.connected;
          break;
        case PlayerPresence.reconnecting:
          rosConnectState_ = Status.errored;
          break;
      }
    });

      if(!isReconnect_){
        isReconnect_ = true;
      }
      
    // 连接成功，初始化通道
    Timer(const Duration(seconds: 1), () async {
      await initChannel();
    });
    return "";
      
  }

  void closeConnection() {
    robotFootprint.value.clear();
    map_.value.data.clear();
    topologyMap_.value.points.clear();
    laserBasePoint_.value.clear();
    localPath.value.clear();
    globalPath.value.clear();
    tracePath.value.clear();
    laserPointData.value.laserPoseBaseLink.clear();
    laserPointData.value.robotPose = RobotPose.zero();
    robotSpeed_.value.vx = 0;
    robotSpeed_.value.vy = 0;
    robotSpeed_.value.vw = 0;
    robotPoseMap.value = RobotPose.zero();
    robotPoseScene.value = RobotPose.zero();
    navStatus_.value = ActionStatus.unknown;
    battery_.value = 0;
    imageData.value = Uint8List(0);
    pointCloud2Data.value.clear();
    localCostmap.value.data.clear();
    globalCostmap.value.data.clear();
    diagnosticData.value = DiagnosticArray();
    cmdVel_.vx = 0;
    cmdVel_.vy = 0;
    
    rosBridgePlayer.close();
  }

  ValueNotifier<OccupancyMap> get map => map_;
  
  // Topics相关getter
  List<TopicWithSchemaName> get topics => rosBridgePlayer.topics;
  Map<String, dynamic> get datatypes => rosBridgePlayer.datatypes;
  int get currentRosVersion => rosBridgePlayer.currentRosVersion;
  
  // 手动触发topics请求
  void refreshTopics() {
    rosBridgePlayer.refreshTopics();
  }
  Future<void> initChannel() async {
    // 使用RosBridgePlayer的订阅接口
    rosBridgePlayer.subscribe(globalSetting.topologyMapTopic, "topology_msgs/TopologyMap", topologyMapCallback);
    rosBridgePlayer.subscribe(globalSetting.mapTopic, "nav_msgs/OccupancyGrid", mapCallback);
    rosBridgePlayer.subscribe(globalSetting.navToPoseStatusTopic, "action_msgs/GoalStatusArray", navStatusCallback);
    rosBridgePlayer.subscribe(globalSetting.navThroughPosesStatusTopic, "action_msgs/GoalStatusArray", navStatusCallback);
    rosBridgePlayer.subscribe("/tf", "tf2_msgs/TFMessage", tfCallback);
    rosBridgePlayer.subscribe("/tf_static", "tf2_msgs/TFMessage", tfStaticCallback);
    rosBridgePlayer.subscribe(globalSetting.laserTopic, "sensor_msgs/LaserScan", laserCallback);
    rosBridgePlayer.subscribe(globalSetting.localPathTopic, "nav_msgs/Path", localPathCallback);
    rosBridgePlayer.subscribe(globalSetting.globalPathTopic, "nav_msgs/Path", globalPathCallback);
    rosBridgePlayer.subscribe(globalSetting.tracePathTopic, "nav_msgs/Path", tracePathCallback);
    rosBridgePlayer.subscribe(globalSetting.odomTopic, "nav_msgs/Odometry", odomCallback);
    rosBridgePlayer.subscribe(globalSetting.batteryTopic, "sensor_msgs/BatteryState", batteryCallback);
    rosBridgePlayer.subscribe(globalSetting.robotFootprintTopic, "geometry_msgs/PolygonStamped", robotFootprintCallback);
    rosBridgePlayer.subscribe(globalSetting.localCostmapTopic, "nav_msgs/OccupancyGrid", localCostmapCallback);
    rosBridgePlayer.subscribe(globalSetting.globalCostmapTopic, "nav_msgs/OccupancyGrid", globalCostmapCallback);
    rosBridgePlayer.subscribe(globalSetting.pointCloud2Topic, "sensor_msgs/PointCloud2", pointCloud2Callback);
    rosBridgePlayer.subscribe(globalSetting.diagnosticTopic, "diagnostic_msgs/DiagnosticArray", diagnosticCallback);

    // 注册发布者
    rosBridgePlayer.advertise(globalSetting.relocTopic, "geometry_msgs/PoseWithCovarianceStamped");
    rosBridgePlayer.advertise(globalSetting.navGoalTopic, "geometry_msgs/PoseStamped");
    rosBridgePlayer.advertise("${globalSetting.navGoalTopic}/cancel", "std_msgs/Empty");
    rosBridgePlayer.advertise("${globalSetting.topologyMapTopic}/update", "topology_msgs/TopologyMap");
    rosBridgePlayer.advertise(globalSetting.getConfig("SpeedCtrlTopic"), "geometry_msgs/Twist");
  }

  Future<Map<String, dynamic>> sendTopologyGoal(String name) async {
    Map<String, dynamic> msg = {
      "point_name": name
    };
    
    try {
      var result = await rosBridgePlayer.callService("/nav_to_topology_point", msg);
      print("result: $result");
      
      // 检查result是否为字符串（错误信息）
      if (result is String) {
        return {
          "is_success": false,
          "message": result
        };
      }
      
      Map<String, dynamic> resultMap = result;
      print("sendTopologyGoal result: $resultMap");
      return resultMap;
    } catch (e) {
      print("sendTopologyGoal error: $e");
      return {
        "is_success": false,
        "message": e.toString()
      };
    }
  }

  Future<void> sendNavigationGoal(RobotPose pose) async {
    vm.Quaternion quaternion = eulerToQuaternion(pose.theta, 0, 0);
    Map<String, dynamic> msg = {
      "header": {
        // "seq": 0,
        "stamp": {
          "secs": DateTime.now().second,
          "nsecs": DateTime.now().millisecond * 1000000
        },
        "frame_id": "map"
      },
      "pose": {
        "position": {"x": pose.x, "y": pose.y, "z": 0},
        "orientation": {
          "x": quaternion.x,
          "y": quaternion.y,
          "z": quaternion.z,
          "w": quaternion.w
        }
      }
    };

    try {
      rosBridgePlayer.publish(globalSetting.navGoalTopic, msg);
    } catch (e) {
      print("Failed to send navigation goal: $e");
    }
  }

  Future<void> sendEmergencyStop() async {
    await sendSpeed(0, 0, 0);
  }

  Future<void> sendCancelNav() async {
    rosBridgePlayer.publish("${globalSetting.navGoalTopic}/cancel", {});
  }

  void destroyConnection() async {
    rosBridgePlayer.close();
  }

  void setVx(double vx) {
    cmdVel_.vx = vx;
  }

  void setVy(double vy) {
    cmdVel_.vy = vy;
  }

  void setVw(double vw) {
    cmdVel_.vw = vw;
  }

  void startMunalCtrl() {
    cmdVelTimer =
        Timer.periodic(const Duration(milliseconds: 100), (timer) async {
      await sendSpeed(cmdVel_.vx, cmdVel_.vy, cmdVel_.vw);
    });
  }

  void stopMunalCtrl() {
    if (cmdVelTimer != null) {
      cmdVelTimer!.cancel();
      cmdVelTimer = null;
    }
    
    // 重置速度命令，确保机器人停止
    cmdVel_.vx = 0;
    cmdVel_.vy = 0;
    cmdVel_.vw = 0;
    
    // 发送一次停止命令
    sendSpeed(0, 0, 0);
  }

  Future<void> sendSpeed(double vx, double vy, double vw) async {
    Map<String, dynamic> msg = {
      "linear": {
        "x": vx, // 代表线速度x分量
        "y": vy, // 代表线速度y分量
        "z": 0.0 // 代表线速度z分量
      },
      "angular": {
        "x": 0.0, // 代表角速度x分量
        "y": 0.0, // 代表角速度y分量
        "z": vw // 代表角速度z分量
      }
    };
    rosBridgePlayer.publish(globalSetting.getConfig("SpeedCtrlTopic"), msg);
  }

  Future<void> sendRelocPose(RobotPose pose) async {
    vm.Quaternion quation = eulerToQuaternion(pose.theta, 0, 0);
    Map<String, dynamic> msg = {
      "header": {
        // "seq": 0,
        "stamp": {
          "secs": DateTime.now().second,
          "nsecs": DateTime.now().millisecond * 1000000
        },
        "frame_id": "map"
      },
      "pose": {
        "pose": {
          "position": {"x": pose.x, "y": pose.y, "z": 0},
          "orientation": {
            "x": quation.x,
            "y": quation.y,
            "z": quation.z,
            "w": quation.w
          }
        },
        "covariance": [
          0.1,
          0,
          0,
          0,
          0,
          0,
          0,
          0.1,
          0,
          0,
          0,
          0,
          0,
          0,
          0.1,
          0,
          0,
          0,
          0,
          0,
          0,
          0.1,
          0,
          0,
          0,
          0,
          0,
          0,
          0.1,
          0,
          0,
          0,
          0,
          0,
          0,
          0.1
        ]
      }
    };
    try {
      rosBridgePlayer.publish(globalSetting.relocTopic, msg);
    } catch (e) {
      print("send reloc pose error:$e");
    }
  }

  Future<void> batteryCallback(Map<String, dynamic> message) async {
    double percentage = message['percentage'] * 100; // 假设电量百分比在 0-1 范围内
    battery_.value = percentage;
    // print("battery:$percentage");
  }

  Future<void> odomCallback(Map<String, dynamic> message) async {
    // 解析线速度 (vx, vy)
    double vx = message['twist']['twist']['linear']['x'];
    double vy = message['twist']['twist']['linear']['y'];

    // 解析角速度 (vw)
    double vw = message['twist']['twist']['angular']['z'];
    RobotSpeed speed = RobotSpeed(vx: vx, vy: vy, vw: vw);
    robotSpeed_.value = speed;
    // print("vx:$vx,vy:$vy,vw:$vw");
  }

  Future<void> robotFootprintCallback(Map<String, dynamic> message) async {
    try {
      PolygonStamped polygonStamped = PolygonStamped.fromJson(message);

      String framId = polygonStamped.header!.frameId!;
      RobotPose transPose = RobotPose(0, 0, 0);
      try {
        transPose = tf_.lookUpForTransform(globalSetting.mapFrameName, framId);
      } catch (e) {
        print("not find robot footprint transfrom form:map to:$framId");
        return;
      }
      
      List<vm.Vector2> newPoints = [];
      
      if (polygonStamped.polygon != null) {
        for (int i = 0; i < polygonStamped.polygon!.points.length; i++) {
          Point32 point = polygonStamped.polygon!.points[i];
          RobotPose pose = RobotPose(point.x, point.y, 0);
          RobotPose poseMap = absoluteSum(transPose, pose);
          vm.Vector2 poseScene = map_.value.xy2idx(vm.Vector2(poseMap.x, poseMap.y));
          newPoints.add(poseScene);
        }
      }
      robotFootprint.value = newPoints;
    } catch (e) {
      print("Error parsing robot footprint: $e");
    }
  }

  Future<void> localCostmapCallback(Map<String, dynamic> msg) async {
    DateTime currentTime = DateTime.now(); // 获取当前时间

    if (_lastMapCallbackTime != null) {
      Duration difference = currentTime.difference(_lastMapCallbackTime!);
      if (difference.inSeconds < 5) {
        return;
      }
    }

    _lastMapCallbackTime = currentTime; // 更新上一次回调时间

    try {
      // 解析局部代价地图数据
      int width = msg["info"]["width"];
      int height = msg["info"]["height"];
      double resolution = msg["info"]["resolution"];
      double originX = msg["info"]["origin"]["position"]["x"];
      double originY = msg["info"]["origin"]["position"]["y"];
      
      // 解析四元数获取旋转角度
      Map<String, dynamic> orientation = msg["info"]["origin"]["orientation"];
      double qx = orientation["x"]?.toDouble() ?? 0.0;
      double qy = orientation["y"]?.toDouble() ?? 0.0;
      double qz = orientation["z"]?.toDouble() ?? 0.0;
      double qw = orientation["w"]?.toDouble() ?? 1.0;
      
      // 四元数转欧拉角
      vm.Quaternion quaternion = vm.Quaternion(qx, qy, qz, qw);
      List<double> euler = quaternionToEuler(quaternion);
      double originTheta = euler[0]; // yaw 角
      
      // 创建局部代价地图
      OccupancyMap costmap = OccupancyMap();
      costmap.mapConfig.resolution = resolution;
      costmap.mapConfig.width = width;
      costmap.mapConfig.height = height;
      costmap.mapConfig.originX = originX;
      costmap.mapConfig.originY = originY;
      
      List<int> dataList = List<int>.from(msg["data"]);
      costmap.data = List.generate(
        height,
        (i) => List.generate(width, (j) => 0),
      );
      
      for (int i = 0; i < dataList.length; i++) {
        int x = i ~/ width;
        int y = i % width;
        costmap.data[x][y] = dataList[i];
      }
      costmap.setFlip();
      
      // 坐标系转换：将局部代价地图的基础坐标转换为 map 坐标系
      String frameId = msg["header"]["frame_id"];
      RobotPose originPose = RobotPose(0, 0, 0);
      
      try {
        // 获取从局部坐标系到 map 坐标系的变换
        RobotPose transPose = tf_.lookUpForTransform(globalSetting.mapFrameName, frameId);
        
        // 计算局部代价地图原点在 map 坐标系中的位置
        RobotPose localOrigin = RobotPose(originX, originY, originTheta);
        RobotPose mapOrigin = absoluteSum(transPose, localOrigin);
        
        // 调整 Y 坐标（考虑地图翻转）
        mapOrigin.y += costmap.heightMap();
        originPose = mapOrigin;

        
      } catch (e) {
        print("getTransform localCostMapCallback error: $e");
        return;
      }
      
      // 将局部代价地图叠加到使用全局地图的size进行叠加
      OccupancyMap sizedCostMap = map_.value.copy();
      sizedCostMap.setZero();
      
      // 使用 xy2idx 方法将代价地图左上角的世界坐标转换为栅格坐标
      vm.Vector2 occPoint = map_.value.xy2idx(vm.Vector2(originPose.x, originPose.y));
      double mapOX = occPoint.x;
      double mapOY = occPoint.y;
      
      // 清空目标区域
      for (int x = 0; x < sizedCostMap.mapConfig.height; x++) {
        for (int y = 0; y < sizedCostMap.mapConfig.width; y++) {
          if (x > mapOX && y > mapOY && 
              y < mapOY + costmap.mapConfig.height &&
              x < mapOX + costmap.mapConfig.width) {
            // 在局部代价地图范围内，使用局部代价地图的值
            int localX = x - mapOX.toInt();
            int localY = y - mapOY.toInt();
            if (localX >= 0 && localX < costmap.mapConfig.height &&
                localY >= 0 && localY < costmap.mapConfig.width) {
              // 添加正确的边界检查
              if (y < sizedCostMap.data.length && 
                  x < sizedCostMap.data[y].length) {
                sizedCostMap.data[y][x] = costmap.data[localY][localX];
              } 
            }
          }
          // 不在范围内，保持原值
        }
      }
      
      // 更新局部代价地图
      localCostmap.value = sizedCostMap;
    } catch (e) {
      print("Error processing local costmap: $e");
    }
  }

  Future<void> tfCallback(Map<String, dynamic> msg) async {
    // print("${json.encode(msg)}");
    tf_.updateTF(TF.fromJson(msg));
  }

  Future<void> tfStaticCallback(Map<String, dynamic> msg) async {
    // print("${json.encode(msg)}");
    tf_.updateTF(TF.fromJson(msg));
  }

  Future<void> localPathCallback(Map<String, dynamic> msg) async {
    List<vm.Vector2> newPath = [];
    // print("${json.encode(msg)}");
    RobotPath path = RobotPath.fromJson(msg);
    String framId = path.header!.frameId!;
    RobotPose transPose = RobotPose(0, 0, 0);
    try {
      transPose = tf_.lookUpForTransform(globalSetting.mapFrameName, framId);
    } catch (e) {
      print("not find local path transfrom form:map to:$framId");
      return;
    }

    for (var pose in path.poses!) {
      RosTransform tran = RosTransform(
          translation: pose.pose!.position!, rotation: pose.pose!.orientation!);
      var poseFrame = tran.getRobotPose();
      var poseMap = absoluteSum(transPose, poseFrame);
      vm.Vector2 poseScene = map_.value.xy2idx(vm.Vector2(poseMap.x, poseMap.y));
      newPath.add(vm.Vector2(poseScene.x, poseScene.y));
    }
    
    // 使用新的列表赋值来触发监听器
    localPath.value = newPath;
  }

  Future<void> globalPathCallback(Map<String, dynamic> msg) async {
    List<vm.Vector2> newPath = [];
    RobotPath path = RobotPath.fromJson(msg);
    String framId = path.header!.frameId!;
    RobotPose transPose = RobotPose(0, 0, 0);
    try {
      transPose = tf_.lookUpForTransform("map", framId);
    } catch (e) {
      print("not find global path transfrom form:map to:$framId");
      return;
    }

    for (var pose in path.poses!) {
      RosTransform tran = RosTransform(
          translation: pose.pose!.position!, rotation: pose.pose!.orientation!);
      var poseFrame = tran.getRobotPose();
      var poseMap = absoluteSum(transPose, poseFrame);
      vm.Vector2 poseScene = map_.value.xy2idx(vm.Vector2(poseMap.x, poseMap.y));
      newPath.add(vm.Vector2(poseScene.x, poseScene.y));
    }
    
    // 使用新的列表赋值来触发监听器
    globalPath.value = newPath;
  }

  Future<void> tracePathCallback(Map<String, dynamic> msg) async {
    List<vm.Vector2> newPath = [];
    RobotPath path = RobotPath.fromJson(msg);
    String framId = path.header!.frameId!;
    RobotPose transPose = RobotPose(0, 0, 0);
    try {
      transPose = tf_.lookUpForTransform("map", framId);
    } catch (e) {
      print("not find trace path transfrom form:map to:$framId");
      return;
    }

    for (var pose in path.poses!) {
      RosTransform tran = RosTransform(
          translation: pose.pose!.position!, rotation: pose.pose!.orientation!);
      var poseFrame = tran.getRobotPose();
      var poseMap = absoluteSum(transPose, poseFrame);
      vm.Vector2 poseScene = map_.value.xy2idx(vm.Vector2(poseMap.x, poseMap.y));
      newPath.add(vm.Vector2(poseScene.x, poseScene.y));
    }
    
    // 使用新的列表赋值来触发监听器
    tracePath.value = newPath;
  }


  Future<void> laserCallback(Map<String, dynamic> msg) async {
    // print("${json.encode(msg)}");
    LaserScan laser = LaserScan.fromJson(msg);
    RobotPose laserPoseBase = RobotPose(0, 0, 0);
    try {
      laserPoseBase = tf_.lookUpForTransform(
          globalSetting.baseLinkFrameName, laser.header!.frameId!);
    } catch (e) {
      print("not find transform from:map to ${laser.header!.frameId!}");
      return;
    }
    // print("find laser size:${laser.ranges!.length}");
    double angleMin = laser.angleMin!.toDouble();
    double angleIncrement = laser.angleIncrement!;
    List<vm.Vector2> newLaserPoints = [];
    for (int i = 0; i < laser.ranges!.length; i++) {
      double angle = angleMin + i * angleIncrement;
      // print("${laser.ranges![i]}");
      if (laser.ranges![i].isInfinite || laser.ranges![i].isNaN) continue;
      double dist = laser.ranges![i];
      //null数据处理
      if (dist == -1) continue;
      RobotPose poseLaser = RobotPose(dist * cos(angle), dist * sin(angle), 0);

      //转换到map坐标系
      RobotPose poseBaseLink = absoluteSum(laserPoseBase, poseLaser);

      newLaserPoints.add(vm.Vector2(poseBaseLink.x, poseBaseLink.y));
    }
    
    // 使用新的列表赋值来触发监听器
    laserBasePoint_.value = newLaserPoints;
    laserPointData.value = LaserData(
        robotPose: robotPoseMap.value, laserPoseBaseLink: newLaserPoints);
  }

  DateTime? _lastMapCallbackTime;

  Future<void> mapCallback(Map<String, dynamic> msg) async {
    
    OccupancyMap map = OccupancyMap();
    map.mapConfig.resolution = msg["info"]["resolution"];
    map.mapConfig.width = msg["info"]["width"];
    map.mapConfig.height = msg["info"]["height"];
    map.mapConfig.originX = msg["info"]["origin"]["position"]["x"];
    map.mapConfig.originY = msg["info"]["origin"]["position"]["y"];
    List<int> dataList = List<int>.from(msg["data"]);
    map.data = List.generate(
      map.mapConfig.height, // 外层列表的长度
      (i) => List.generate(
        map.mapConfig.width, // 内层列表的长度
        (j) => 0, // 初始化值
      ),
    );
    for (int i = 0; i < dataList.length; i++) {
      int x = i ~/ map.mapConfig.width;
      int y = i % map.mapConfig.width;
      map.data[x][y] = dataList[i];
    }
    map.setFlip();
    map_.value = map;
  }

  Future<void> topologyMapCallback(Map<String, dynamic> msg) async {
    // 延迟1秒执行 避免地图还未加载，点位就发过来了（只发送一次）
    await Future.delayed(Duration(seconds: 1));
    
    // print("收到拓扑地图数据: $msg");
    
    final map = TopologyMap.fromJson(msg);
    // print("解析后的拓扑地图 - 点数量: ${map.points.length}, 路径数量: ${map.routes.length}");

    // 创建新的 points 列表
    final updatedPoints = map.points.map((point) {
      // 创建新的 NavPoint 对象
      return NavPoint(
        x: point.x,
        y: point.y,
        theta: point.theta,
        name: point.name,
        type: point.type,
      );
    }).toList();

    // 创建新的 TopologyMap 对象，包含转换后的点和原始路径信息
    final updatedMap = TopologyMap(
      points: updatedPoints, 
      routes: map.routes,
      mapName: map.mapName,
      mapProperty: map.mapProperty,
    );

    print("更新后的拓扑地图 - 点数量: ${updatedMap.points.length}, 路径数量: ${updatedMap.routes.length}");
    
    // 更新 ValueNotifier
    topologyMap_.value = updatedMap;
  }

  Future<void> updateTopologyMap(TopologyMap updatedMap) async {
    // 转换为JSON并通过ROS发布
    try {
      final jsonData = updatedMap.toJson();
      rosBridgePlayer.publish("${globalSetting.topologyMapTopic}/update", jsonData);
      print("拓扑地图已发布到ROS: ${updatedMap.points.length}个点, ${updatedMap.routes.length}条路径");
    } catch (e) {
      print("发布拓扑地图失败: $e");
    }
  }

  Future<void> navStatusCallback(Map<String, dynamic> msg) async {
    GoalStatusArray goalStatusArray = GoalStatusArray.fromJson(msg);
    navStatus_.value = goalStatusArray.statusList.last.status;
  }

  Future<void> pointCloud2Callback(Map<String, dynamic> msg) async {
    try {
      PointCloud2 pointCloud = PointCloud2.fromJson(msg);
      
      // 获取点云数据
      List<Point3D> points = pointCloud.getPoints();
      
      // 转换坐标系：从点云坐标系到map坐标系
      String frameId = pointCloud.header!.frameId!;
      RobotPose transPose = RobotPose(0, 0, 0);
      
      try {
        transPose = tf_.lookUpForTransform(globalSetting.mapFrameName, frameId);
      } catch (e) {
        print("not find pointcloud transform from:map to:$frameId");
        return;
      }
      
      // 转换所有点到map坐标系
      List<Point3D> transformedPoints = [];
      for (Point3D point in points) {
        RobotPose pointPose = RobotPose(point.x, point.y, 0);
        RobotPose mapPose = absoluteSum(transPose, pointPose);
        transformedPoints.add(Point3D(mapPose.x, mapPose.y, point.z));
      }
      
      
      // 更新点云数据
      pointCloud2Data.value = transformedPoints;
      
    } catch (e) {
      print("Error processing PointCloud2 data: $e");
    }
  }

  Future<void> globalCostmapCallback(Map<String, dynamic> msg) async {
    DateTime currentTime = DateTime.now(); // 获取当前时间

    if (_lastMapCallbackTime != null) {
      Duration difference = currentTime.difference(_lastMapCallbackTime!);
      if (difference.inSeconds < 5) {
        return;
      }
    }

    _lastMapCallbackTime = currentTime; // 更新上一次回调时间

    try {
      // 解析全局代价地图数据
      int width = msg["info"]["width"];
      int height = msg["info"]["height"];
      double resolution = msg["info"]["resolution"];
      double originX = msg["info"]["origin"]["position"]["x"];
      double originY = msg["info"]["origin"]["position"]["y"];
      
      // 创建全局代价地图
      OccupancyMap costmap = OccupancyMap();
      costmap.mapConfig.resolution = resolution;
      costmap.mapConfig.width = width;
      costmap.mapConfig.height = height;
      costmap.mapConfig.originX = originX;
      costmap.mapConfig.originY = originY;
      
      List<int> dataList = List<int>.from(msg["data"]);
      costmap.data = List.generate(
        height,
        (i) => List.generate(width, (j) => 0),
      );
      
      for (int i = 0; i < dataList.length; i++) {
        int x = i ~/ width;
        int y = i % width;
        costmap.data[x][y] = dataList[i];
      }
      costmap.setFlip();
      
      // 直接更新全局代价地图，不需要resize
      globalCostmap.value = costmap;
    } catch (e) {
      print("Error processing global costmap: $e");
    }
  }

  Future<void> diagnosticCallback(Map<String, dynamic> msg) async {
    try {
      DiagnosticArray diagnosticArray = DiagnosticArray.fromJson(msg);
      
      // 更新诊断数据（保持向后兼容）
      diagnosticData.value = diagnosticArray;
      
      // 使用DiagnosticManager管理诊断状态
      diagnosticManager.updateDiagnosticStates(diagnosticArray);
      
    } catch (e) {
      print("Error processing diagnostic data: $e");
    }
  }

}
