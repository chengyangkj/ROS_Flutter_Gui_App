import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:roslibdart/roslibdart.dart';
import 'dart:async';
import 'dart:convert';
import 'package:provider/provider.dart';
import "package:ros_flutter_gui_app/basic/occupancy_map.dart";

class RosChannel extends ChangeNotifier {
  late Ros ros;
  late Topic mapTopic_;
  OccupancyMap map_ = OccupancyMap();
  Status rosConnectState_ = Status.none;
  @override
  void initState() {}

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

  Future<void> initChannel() async {
    mapTopic_ = Topic(
        ros: ros,
        name: '/map',
        type: "nav_msgs/OccupancyGrid",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);
    mapTopic_.subscribe(mapCallback);
  }

  void destroyConnection() async {
    await mapTopic_.unsubscribe();
    await ros.close();
  }

  Future<void> mapCallback(Map<String, dynamic> msg) async {
    // print("recv ${json.encode(msg)}");
    map_.mapConfig.resolution = msg["info"]["resolution"];
    map_.mapConfig.width = msg["info"]["width"];
    map_.mapConfig.height = msg["info"]["height"];
    map_.mapConfig.originX = msg["info"]["origin"]["position"]["x"];
    map_.mapConfig.originY = msg["info"]["origin"]["position"]["y"];
    List<int> dataList = List<int>.from(msg["data"]);
    map_.data = List.generate(
      map_.mapConfig.height, // 外层列表的长度
      (i) => List.generate(
        map_.mapConfig.width, // 内层列表的长度
        (j) => 0, // 初始化值
      ),
    );
    for (int i = 0; i < dataList.length; i++) {
      int x = i ~/ map_.mapConfig.width;
      int y = i % map_.mapConfig.width;
      map_.data[x][y] = dataList[i];
    }
    notifyListeners();
  }

  String msgReceived = '';
  Future<void> subscribeHandler(Map<String, dynamic> msg) async {
    msgReceived = json.encode(msg);
    print("recv ${msgReceived}");
  }
}
