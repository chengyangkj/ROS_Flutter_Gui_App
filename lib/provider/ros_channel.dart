import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/tf2_dart.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:roslibdart/roslibdart.dart';
import 'dart:async';
import 'dart:convert';
import 'package:provider/provider.dart';
import "package:ros_flutter_gui_app/basic/occupancy_map.dart";
import 'package:ros_flutter_gui_app/basic/tf.dart';
import 'package:ros_flutter_gui_app/basic/laser_scan.dart';

class RosChannel extends ChangeNotifier {
  late Ros ros;
  late Topic mapChannel_;
  late Topic tfChannel_;
  late Topic tfStaticChannel_;
  late Topic laserChannel_;
  TF2Dart tf_ = TF2Dart();
  ValueNotifier<OccupancyMap> map_ =
      ValueNotifier<OccupancyMap>(OccupancyMap());
  Status rosConnectState_ = Status.none;
  RobotPose currRobotPose_ = RobotPose(0, 0, 0);
  List<Offset> laserPoint_ = [];

  RosChannel() {
    //启动定时器 获取机器人实时坐标

    Timer.periodic(const Duration(milliseconds: 50), (timer) {
      try {
        currRobotPose_ = tf_.lookUpForTransform("map", "base_link");
        notifyListeners();
      } catch (e) {
        print("get robot pose error:${e}");
      }
    });
  }

  List<Offset> get laserPointScene {
    return laserPoint_;
  }

  RobotPose get robotPoseScene {
    Offset poseScene =
        map_.value.xy2idx(Offset(currRobotPose_.x, currRobotPose_.y));
    return RobotPose(poseScene.dx, poseScene.dy, currRobotPose_.theta);
  }

  Future<bool> connect(String url) async {
    rosConnectState_ = Status.none;
    ros = Ros(url: url);

    ros.statusStream.listen(
      (Status data) {
        print("connect state: $data");
        rosConnectState_ = data;
      },
      onError: (error) {
        print('Error occurred: $error'); // 打印错误信息
      },
      onDone: () {
        print('Stream closed'); // 打印流关闭信息
      },
      cancelOnError: false, // 是否在遇到错误时取消订阅，默认为false);
    );
    ros.connect();

    // 等待连接成功
    while (true) {
      if (rosConnectState_ == Status.connected) {
        print("ros connect success!");
        break;
      } else if (rosConnectState_ == Status.closed ||
          rosConnectState_ == Status.errored) {
        print("ros connect failed!");
        return false;
      }
      await Future.delayed(Duration(milliseconds: 100));
    }
    Timer(const Duration(seconds: 1), () async {
      await initChannel();
      // await chatter.subscribe();
    });
    return true;
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
    tfStaticChannel_.subscribe(tfCallback);

    laserChannel_ = Topic(
      ros: ros,
      name: globalSetting.laserTopic,
      type: "sensor_msgs/LaserScan",
      queueSize: 1,
      reconnectOnClose: true,
    );

    laserChannel_.subscribe(laserCallback);
  }

  void destroyConnection() async {
    await mapChannel_.unsubscribe();
    await ros.close();
  }

  Future<void> tfCallback(Map<String, dynamic> msg) async {
    // print("${json.encode(msg)}");
    tf_.updateTF(TF.fromJson(msg));
    // notifyListeners();
  }

  Future<void> laserCallback(Map<String, dynamic> msg) async {
    // print("${json.encode(msg)}");
    LaserScan laser = LaserScan.fromJson(msg);
    RobotPose laserPoseMap = RobotPose(0, 0, 0);
    try {
      laserPoseMap = tf_.lookUpForTransform("map", laser.header!.frameId!);
    } catch (e) {
      print("not find transform from:map to ${laser.header!.frameId!}");
      return;
    }
    // print("find laser size:${laser.ranges!.length}");
    double angleMin = laser.angleMin!.toDouble();
    double angleMax = laser.angleMax!;
    double angleIncrement = laser.angleIncrement!;
    laserPoint_.clear();
    for (int i = 0; i < laser.ranges!.length; i++) {
      double angle = angleMin + i * angleIncrement;
      // print("${laser.ranges![i]}");
      if (laser.ranges![i].isInfinite || laser.ranges![i].isNaN) continue;
      double dist = laser.ranges![i];
      //null数据处理
      if (dist == -1) continue;
      RobotPose poseLaser = RobotPose(dist * cos(angle), dist * sin(angle), 0);

      //转换到map坐标系
      RobotPose poseMap = absoluteSum(laserPoseMap, poseLaser);
      Offset poseScene = map_.value.xy2idx(Offset(poseMap.x, poseMap.y));
      laserPoint_.add(Offset(poseScene.dx, poseScene.dy));
    }
    notifyListeners();
  }

  Future<void> mapCallback(Map<String, dynamic> msg) async {
    // print("recv ${json.encode(msg)}");

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
}
