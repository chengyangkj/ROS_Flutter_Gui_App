import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

class Setting {
  late SharedPreferences prefs;
  Future<bool> init() async {
    prefs = await SharedPreferences.getInstance();
    if (!prefs.containsKey("init") ||
        (prefs.containsKey("init") && prefs.getString("init") != "1")) {
      setDefaultCfgRos1();
    }
    return true;
  }

  void setDefaultCfgRos1() {
    prefs.setString('init', "1");
    prefs.setString('mapTopic', "map");
    prefs.setString('laserTopic', "scan");
    prefs.setString('globalPathTopic', "/move_base/DWAPlannerROS/global_plan");
    prefs.setString('localPathTopic', "/move_base/DWAPlannerROS/local_plan");
    prefs.setString('relocTopic', "/initialpose");
    prefs.setString('navGoalTopic', "move_base_simple/goal");
    prefs.setString('OdometryTopic', "/odom");
    prefs.setString('SpeedCtrlTopic', "/cmd_vel");
    prefs.setString('BatteryTopic', "/battery");
    prefs.setString('MaxVx', "0.1");
    prefs.setString('MaxVy', "0.1");
    prefs.setString('MaxVw', "0.3");
  }

  SharedPreferences get config {
    return prefs;
  }

  // 设置机器人IP
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

  void setRelocTopic(String topic) {
    prefs.setString('relocTopic', topic);
  }

  String get relocTopic {
    return prefs.getString("relocTopic") ?? "/initialpose";
  }

  void setNavGoalTopic(String topic) {
    prefs.setString('navGoalTopic', topic);
  }

  String get navGoalTopic {
    return prefs.getString("navGoalTopic") ?? "/move_base/goal";
  }

  void setMapFrame(String frame) {
    prefs.setString('mapFrame', frame);
  }

  String get mapFrame {
    return prefs.getString("mapFrame") ?? "map";
  }

  String getConfig(String key) {
    return prefs.getString(key) ?? "";
  }
  //实列
}

Setting globalSetting = Setting();

// 初始化全局配置
Future<bool> initGlobalSetting() async {
  return globalSetting.init();
}
