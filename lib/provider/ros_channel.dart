import 'dart:math';
import 'dart:typed_data';
import 'dart:ui';
import 'dart:io';
import 'package:image/image.dart' as img;
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
import 'package:roslibdart/roslibdart.dart';
import 'dart:async';
import 'dart:convert';
import 'package:provider/provider.dart';
import "package:ros_flutter_gui_app/basic/occupancy_map.dart";
import 'package:ros_flutter_gui_app/basic/tf.dart';
import 'package:ros_flutter_gui_app/basic/laser_scan.dart';
import "package:ros_flutter_gui_app/basic/math.dart";
import 'package:vector_math/vector_math_64.dart' as vm;
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:image/image.dart' as img;
import 'package:fluttertoast/fluttertoast.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/basic/polygon_stamped.dart';
import 'package:ros_flutter_gui_app/basic/pointcloud2.dart';

class LaserData {
  RobotPose robotPose;
  List<Offset> laserPoseBaseLink;
  LaserData({required this.robotPose, required this.laserPoseBaseLink});
}

class RobotSpeed {
  double vx;
  double vy;
  double vw;
  RobotSpeed({required this.vx, required this.vy, required this.vw});
}

class RosChannel {
  late Ros ros;
  late Topic mapChannel_;
  late Topic topologyMapChannel_;
  late Topic tfChannel_;
  late Topic tfStaticChannel_;
  late Topic laserChannel_;
  late Topic localPathChannel_;
  late Topic globalPathChannel_;
  late Topic relocChannel_;
  late Topic navGoalChannel_;
  late Topic navGoalCancelChannel_;
  late Topic speedCtrlChannel_;
  late Topic odomChannel_;
  late Topic batteryChannel_;
  late Topic imageTopic_;
  late Topic navToPoseStatusChannel_;
  late Topic navThroughPosesStatusChannel_;
  late Topic robotFootprintChannel_;
  late Topic localCostmapChannel_;
  late Topic globalCostmapChannel_;
  late Topic pointCloud2Channel_;
  late Service topologyGoalService_;

  String rosUrl_ = "";
  Timer? cmdVelTimer;
  bool isReconnect_ = false;

  bool manualCtrlMode_ = false;
  ValueNotifier<double> battery_ = ValueNotifier(78);
  ValueNotifier<Uint8List> imageData = ValueNotifier(Uint8List(0));
  RobotSpeed cmdVel_ = RobotSpeed(vx: 0, vy: 0, vw: 0);
  double vxLeft_ = 0;
  ValueNotifier<RobotSpeed> robotSpeed_ =
      ValueNotifier(RobotSpeed(vx: 0, vy: 0, vw: 0));
  String url_ = "";
  TF2Dart tf_ = TF2Dart();
  ValueNotifier<OccupancyMap> map_ =
      ValueNotifier<OccupancyMap>(OccupancyMap());
  ValueNotifier<TopologyMap> topologyMap_ =
      ValueNotifier<TopologyMap>(TopologyMap(points: []));
  Status rosConnectState_ = Status.none;
  ValueNotifier<RobotPose> currRobotPose_ = ValueNotifier(RobotPose.zero());
  ValueNotifier<RobotPose> robotPoseScene = ValueNotifier(RobotPose.zero());
  ValueNotifier<List<Offset>> laserPoint_ = ValueNotifier([]);
  ValueNotifier<List<Offset>> localPath = ValueNotifier([]);
  ValueNotifier<List<Offset>> globalPath = ValueNotifier([]);
  ValueNotifier<LaserData> laserPointData = ValueNotifier(
      LaserData(robotPose: RobotPose(0, 0, 0), laserPoseBaseLink: []));
  ValueNotifier<ActionStatus> navStatus_ = ValueNotifier(ActionStatus.unknown);
  ValueNotifier<List<Offset>> robotFootprint = ValueNotifier([]);
  ValueNotifier<OccupancyMap> localCostmap = ValueNotifier(OccupancyMap());
  ValueNotifier<OccupancyMap> globalCostmap = ValueNotifier(OccupancyMap());
  ValueNotifier<List<Point3D>> pointCloud2Data = ValueNotifier([]);

  RosChannel() {
    //启动定时器 获取机器人实时坐标
    globalSetting.init().then((success) {
       //监听链接状态

        //获取机器人实时坐标
        Timer.periodic(const Duration(milliseconds: 50), (timer) {
          if (rosConnectState_ != Status.connected) return;
          try {
            currRobotPose_.value = tf_.lookUpForTransform(
                globalSetting.mapFrameName, globalSetting.baseLinkFrameName);
            Offset poseScene = map_.value
                .xy2idx(Offset(currRobotPose_.value.x, currRobotPose_.value.y));
            robotPoseScene.value = RobotPose(
                poseScene.dx, poseScene.dy, currRobotPose_.value.theta);
          } catch (e) {
            print("get robot pose error:${e}");
          }
        });

        //重连
        Timer.periodic(const Duration(seconds: 5), (timer) async {
          if (isReconnect_ && rosConnectState_ != Status.connected){
            Fluttertoast.showToast(
              msg: "lost connection to ${rosUrl_} try reconnect...",
              toastLength: Toast.LENGTH_SHORT,
              gravity: ToastGravity.TOP,
            );
            String error = await connect(rosUrl_);
            if(error.isEmpty){
              Fluttertoast.showToast(
                msg: "reconnect success to ${rosUrl_}!",
                toastLength: Toast.LENGTH_SHORT,
                gravity: ToastGravity.TOP,
              );
            }else{
              Fluttertoast.showToast(
                msg: "reconnect failed to ${rosUrl_} error: $error",
                toastLength: Toast.LENGTH_LONG,
                gravity: ToastGravity.TOP,
              );
            }
          }
        });
      
    });
  }

  RobotPose get robotPoseMap {
    return currRobotPose_.value;
  }

  Future<String> connect(String url) async {
    rosUrl_ = url;
    rosConnectState_ = Status.none;
    ros = Ros(url: url);

    // 设置状态监听器
    ros.statusStream.listen(
      (Status data) {
        rosConnectState_ = data;
      },
      onError: (error) {
        rosConnectState_ = Status.errored;
      },
      onDone: () {
        rosConnectState_ = Status.closed;
      },
      cancelOnError: false, // 改为 false，让监听器继续工作
    );

      // 尝试连接
      String error = await ros.connect();
    
      if (error != "") {
        return error;
      }

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
    laserPoint_.value.clear();
    localPath.value.clear();
    globalPath.value.clear();
    laserPointData.value.laserPoseBaseLink.clear();
    laserPointData.value.robotPose = RobotPose.zero();
    robotSpeed_.value.vx = 0;
    robotSpeed_.value.vy = 0;
    robotSpeed_.value.vw = 0;
    currRobotPose_.value = RobotPose.zero();
    robotPoseScene.value = RobotPose.zero();
    navStatus_.value = ActionStatus.unknown;
    battery_.value = 0;
    imageData.value = Uint8List(0);
    pointCloud2Data.value.clear();
    localCostmap.value.data.clear();
    globalCostmap.value.data.clear();
    cmdVel_.vx = 0;
    cmdVel_.vy = 0;
    ros.close();
  }

  ValueNotifier<OccupancyMap> get map => map_;
  Future<void> initChannel() async {
    mapChannel_ = Topic(
        ros: ros,
        name: globalSetting.mapTopic,
        type: "nav_msgs/OccupancyGrid",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);
    mapChannel_.subscribe(mapCallback);

    topologyMapChannel_ = Topic(
        ros: ros,
        name: globalSetting.topologyMapTopic,
        type: "topology_msgs/TopologyMap",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);
    topologyMapChannel_.subscribe(topologyMapCallback);

    navToPoseStatusChannel_ = Topic(
      ros: ros,
      name: globalSetting.navToPoseStatusTopic,
      type: "action_msgs/GoalStatusArray",
      queueSize: 1,
      reconnectOnClose: true,
    );
    navToPoseStatusChannel_.subscribe(navStatusCallback);

    navThroughPosesStatusChannel_ = Topic(
      ros: ros,
      name: globalSetting.navThroughPosesStatusTopic,
      type: "action_msgs/GoalStatusArray",
      queueSize: 1,
      reconnectOnClose: true,
    );
    navThroughPosesStatusChannel_.subscribe(navStatusCallback);

    tfChannel_ = Topic(
      ros: ros,
      name: "/tf",
      type: "tf2_msgs/TFMessage",
      queueSize: 1,
      reconnectOnClose: true,
    );
    tfChannel_.subscribe(tfCallback);

    tfStaticChannel_ = Topic(
      ros: ros,
      name: "/tf_static",
      type: "tf2_msgs/TFMessage",
      queueSize: 1,
      reconnectOnClose: true,
    );
    tfStaticChannel_.subscribe(tfStaticCallback);

    laserChannel_ = Topic(
      ros: ros,
      name: globalSetting.laserTopic,
      type: "sensor_msgs/LaserScan",
      queueSize: 1,
      reconnectOnClose: true,
    );

    laserChannel_.subscribe(laserCallback);

    localPathChannel_ = Topic(
      ros: ros,
      name: globalSetting.localPathTopic,
      type: "nav_msgs/Path",
      queueSize: 1,
      reconnectOnClose: true,
    );
    localPathChannel_.subscribe(localPathCallback);

    globalPathChannel_ = Topic(
      ros: ros,
      name: globalSetting.globalPathTopic,
      type: "nav_msgs/Path",
      queueSize: 1,
      reconnectOnClose: true,
    );
    globalPathChannel_.subscribe(globalPathCallback);

    odomChannel_ = Topic(
      ros: ros,
      name: globalSetting.odomTopic,
      type: 'nav_msgs/Odometry',
      queueSize: 10,
      queueLength: 10,
    );
    odomChannel_.subscribe(odomCallback);

    batteryChannel_ = Topic(
      ros: ros,
      name: globalSetting.batteryTopic, // ROS 中的电池电量话题名称
      type: 'sensor_msgs/BatteryState', // 消息类型
      queueSize: 10,
      queueLength: 10,
    );

    batteryChannel_.subscribe(batteryCallback);

    robotFootprintChannel_ = Topic(
      ros: ros,
      name: globalSetting.robotFootprintTopic,
      type: "geometry_msgs/PolygonStamped",
      queueSize: 1,
      reconnectOnClose: true,
    );
    robotFootprintChannel_.subscribe(robotFootprintCallback);

    localCostmapChannel_ = Topic(
      ros: ros,
      name: globalSetting.localCostmapTopic,
      type: "nav_msgs/OccupancyGrid",
      queueSize: 1,
      reconnectOnClose: true,
    );
    localCostmapChannel_.subscribe(localCostmapCallback);

    globalCostmapChannel_ = Topic(
      ros: ros,
      name: globalSetting.globalCostmapTopic,
      type: "nav_msgs/OccupancyGrid",
      queueSize: 1,
      reconnectOnClose: true,
    );
    globalCostmapChannel_.subscribe(globalCostmapCallback);
    
    pointCloud2Channel_ = Topic(
      ros: ros,
      name: globalSetting.pointCloud2Topic,
      type: "sensor_msgs/PointCloud2",
      queueSize: 1,
      reconnectOnClose: true,
    );
    pointCloud2Channel_.subscribe(pointCloud2Callback);

//发布者
    relocChannel_ = Topic(
      ros: ros,
      name: globalSetting.relocTopic,
      type: "geometry_msgs/PoseWithCovarianceStamped",
      queueSize: 1,
      reconnectOnClose: true,
    );
    navGoalChannel_ = Topic(
      ros: ros,
      name: globalSetting.navGoalTopic,
      type: "geometry_msgs/PoseStamped",
      queueSize: 1,
      reconnectOnClose: true,
    );
    navGoalCancelChannel_ = Topic(
      ros: ros,
      name: "${globalSetting.navGoalTopic}/cancel",
      type: "std_msgs/Empty",
      queueSize: 1,
      reconnectOnClose: true,
    );
    speedCtrlChannel_ = Topic(
      ros: ros,
      name: globalSetting.getConfig("SpeedCtrlTopic"),
      type: "geometry_msgs/Twist",
      queueSize: 1,
      reconnectOnClose: true,
    );

    topologyGoalService_ = Service(
      ros: ros,
      name: "/nav_to_topology_point",
      type: "topology_msgs/srv/NavToTopologyPoint"
    );

  }

  Future<Map<String, dynamic>> sendTopologyGoal(String name) async {
    Map<String, dynamic> msg = {
      "point_name": name
    };
    
    try {
      var result = await topologyGoalService_.call(msg);
      print("result: $result");
      
      // 检查result是否为字符串（错误信息）
      if (result is String) {
        return {
          "is_success": false,
          "message": result
        };
      }
      
      Map<String, dynamic> resultMap = result.toJson();
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
    Offset p = map_.value.idx2xy(Offset(pose.x, pose.y));
    pose.x = p.dx;
    pose.y = p.dy;
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

    // Assuming `channel` is a pre-configured MethodChannel connected to ROS
    try {
      await navGoalChannel_.publish(msg);
    } catch (e) {
      print("Failed to send navigation goal: $e");
    }
  }

  Future<void> sendEmergencyStop() async {
    await sendSpeed(0, 0, 0);
  }

  Future<void> sendCancelNav() async {
    await navGoalCancelChannel_.publish({});
  }

  void destroyConnection() async {
    await mapChannel_.unsubscribe();
    await ros.close();
  }

  void setVxRight(double vx) {
    if (vxLeft_ == 0) {
      cmdVel_.vx = vx;
    }
  }

  void setVx(double vx) {
    vxLeft_ = vx;
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
    await speedCtrlChannel_.publish(msg);
  }

  Future<void> sendRelocPoseScene(RobotPose pose) async {
    Offset p = map_.value.idx2xy(Offset(pose.x, pose.y));
    pose.x = p.dx;
    pose.y = p.dy;
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
      await relocChannel_.publish(msg);
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
      
      // 清空之前的点列表
      robotFootprint.value.clear();
      
      if (polygonStamped.polygon != null) {
        for (int i = 0; i < polygonStamped.polygon!.points.length; i++) {
          Point32 point = polygonStamped.polygon!.points[i];
          RobotPose pose = RobotPose(point.x, point.y, 0);
          RobotPose poseMap = absoluteSum(transPose, pose);
          Offset poseScene = map_.value.xy2idx(Offset(poseMap.x, poseMap.y));
          robotFootprint.value.add(Offset(poseScene.dx, poseScene.dy));
        }
      }
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
      Offset occPoint = map_.value.xy2idx(Offset(originPose.x, originPose.y));
      double mapOX = occPoint.dx;
      double mapOY = occPoint.dy;
      
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
              sizedCostMap.data[y][x] = costmap.data[localY][localX];
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
    localPath.value.clear();
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
      Offset poseScene = map_.value.xy2idx(Offset(poseMap.x, poseMap.y));
      localPath.value.add(Offset(poseScene.dx, poseScene.dy));
    }
  }

  Future<void> globalPathCallback(Map<String, dynamic> msg) async {
    globalPath.value.clear();
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
      Offset poseScene = map_.value.xy2idx(Offset(poseMap.x, poseMap.y));
      globalPath.value.add(Offset(poseScene.dx, poseScene.dy));
    }
  }

  Uint8List _hexToBytes(String hex) {
    final length = hex.length;
    final buffer = Uint8List(length ~/ 2);
    for (int i = 0; i < length; i += 2) {
      buffer[i ~/ 2] = int.parse(hex.substring(i, i + 2), radix: 16);
    }
    return buffer;
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
    double angleMax = laser.angleMax!.toDouble();
    double angleIncrement = laser.angleIncrement!;
    laserPoint_.value.clear();
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

      laserPoint_.value.add(Offset(poseBaseLink.x, poseBaseLink.y));
    }
    laserPointData.value = LaserData(
        robotPose: currRobotPose_.value, laserPoseBaseLink: laserPoint_.value);
  }

  DateTime? _lastMapCallbackTime;

  Future<void> mapCallback(Map<String, dynamic> msg) async {
    DateTime currentTime = DateTime.now(); // 获取当前时间

    if (_lastMapCallbackTime != null) {
      Duration difference = currentTime.difference(_lastMapCallbackTime!);
      if (difference.inSeconds < 5) {
        return;
      }
    }

    _lastMapCallbackTime = currentTime; // 更新上一次回调时间

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

  String msgReceived = '';
  Future<void> subscribeHandler(Map<String, dynamic> msg) async {
    msgReceived = json.encode(msg);
    print("recv ${msgReceived}");
  }

  Future<void> topologyMapCallback(Map<String, dynamic> msg) async {
    // 延迟1秒执行 避免地图还未加载，点位就发过来了（只发送一次）
    await Future.delayed(Duration(seconds: 1));
    
    print("收到拓扑地图数据: $msg");
    
    final map = TopologyMap.fromJson(msg);
    print("解析后的拓扑地图 - 点数量: ${map.points.length}, 路径数量: ${map.routes.length}");

    // 创建新的 points 列表
    final updatedPoints = map.points.map((point) {
      Offset pointScene = map_.value.xy2idx(Offset(point.x, point.y));
      // 创建新的 NavPoint 对象
      return NavPoint(
        x: pointScene.dx,
        y: pointScene.dy,
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
}
