import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

class Setting {
  late SharedPreferences prefs;
  Future<bool> init() async {
    prefs = await SharedPreferences.getInstance();
    return true;
  }

  void setRobotIp(String ip) {
    prefs.setString('robotIp', ip);
  }

  String get robotIp {
    return prefs.getString("robotIp") ?? "127.0.0.1";
  }

  void setRobotPort(String port) {
    prefs.setString('robotPort', port);
  }

  String get robotPort {
    return prefs.getString("robotPort") ?? "9090";
  }

  void setMapTopic(String topic) {
    prefs.setString('mapTopic', topic);
  }

  String get mapTopic {
    return prefs.getString("mapTopic") ?? "map";
  }

  void setLaserTopic(String topic) {
    prefs.setString('laserTopic', topic);
  }

  String get laserTopic {
    return prefs.getString("laserTopic") ?? "scan";
  }

  void setGloalPathTopic(String topic) {
    prefs.setString('globalPathTopic', topic);
  }

  String get globalPathTopic {
    return prefs.getString("globalPathTopic") ??
        "/move_base/DWAPlannerROS/global_plan";
  }

  void setLocalPathTopic(String topic) {
    prefs.setString('localPathTopic', topic);
  }

  String get localPathTopic {
    return prefs.getString("localPathTopic") ??
        "/move_base/DWAPlannerROS/local_plan";
  }

  void setMapFrame(String frame) {
    prefs.setString('mapFrame', frame);
  }

  String get mapFrame {
    return prefs.getString("mapFrame") ?? "map";
  }
}

Setting globalSetting = Setting();

// 初始化全局配置
Future<bool> initGlobalSetting() async {
  return globalSetting.init();
}
