import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

enum KeyName {
  None,
  leftAxisX,
  leftAxisY,
  rightAxisX,
  rightAxisY,
  lS,
  rS,
  triggerLeft,
  triggerRight,
  buttonUpDown,
  buttonLeftRight,
  buttonA,
  buttonB,
  buttonX,
  buttonY,
  buttonLB,
  buttonRB,
}

class JoyStickEvent {
  late KeyName keyName;
  bool reverse = false; //是否反转(反转填-1)
  int maxValue = 32767;
  int minValue = -32767;
  double value = 0;
  JoyStickEvent(this.keyName,
      {this.reverse = false, this.maxValue = 32767, this.minValue = -32767});
}

// 定义一个映射关系，将Dart中的类名映射到JavaScript中的类名
Map<String, JoyStickEvent> axisMapping = {
  "AXIS_X": JoyStickEvent(KeyName.leftAxisX),
  "AXIS_Y": JoyStickEvent(KeyName.leftAxisY),
  "AXIS_Z": JoyStickEvent(KeyName.rightAxisX),
  "AXIS_RZ": JoyStickEvent(KeyName.rightAxisY),
  "triggerRight": JoyStickEvent(KeyName.triggerRight),
  "triggerLeft": JoyStickEvent(KeyName.triggerLeft),
  "buttonLeftRight": JoyStickEvent(KeyName.buttonLeftRight),
  "buttonUpDown": JoyStickEvent(KeyName.buttonUpDown),
};
Map<String, JoyStickEvent> buttonMapping = {
  "KEYCODE_BUTTON_A":
      JoyStickEvent(KeyName.buttonA, maxValue: 1, minValue: 0, reverse: true),
  "KEYCODE_BUTTON_B":
      JoyStickEvent(KeyName.buttonB, maxValue: 1, minValue: 0, reverse: true),
  "KEYCODE_BUTTON_X":
      JoyStickEvent(KeyName.buttonX, maxValue: 1, minValue: 0, reverse: true),
  "KEYCODE_BUTTON_Y":
      JoyStickEvent(KeyName.buttonY, maxValue: 1, minValue: 0, reverse: true),
  "KEYCODE_BUTTON_L1":
      JoyStickEvent(KeyName.buttonLB, maxValue: 1, minValue: 0, reverse: true),
  "KEYCODE_BUTTON_R1":
      JoyStickEvent(KeyName.buttonRB, maxValue: 1, minValue: 0, reverse: true),
};

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
    prefs.setString('mapFrameName', "map");
    prefs.setString('baseLinkFrameName', "base_link");
    prefs.setString('imagePort', "8080");
    prefs.setString('imageTopic', "/camera/rgb/image_raw");
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

  String get imagePort {
    return prefs.getString("imagePort") ?? "8080";
  }

  String get imageTopic {
    return prefs.getString("imageTopic") ?? "/camera/rgb/image_raw";
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

  String get mapFrameName {
    return prefs.getString("mapFrameName") ?? "map";
  }

  String get baseLinkFrameName {
    return prefs.getString("baseLinkFrameName") ?? "base_link";
  }

  void setNavGoalTopic(String topic) {
    prefs.setString('navGoalTopic', topic);
  }

  String get batteryTopic {
    return prefs.getString("batteryTopic") ?? "/battery_state";
  }

  void setBatteryTopic(String topic) {
    prefs.setString('batteryTopic', topic);
  }

  String get navGoalTopic {
    return prefs.getString("navGoalTopic") ?? "/move_base_simple/goal";
  }

  String getConfig(String key) {
    return prefs.getString(key) ?? "";
  }

  String get odomTopic {
    return prefs.getString("OdomTopic") ?? "/odom";
  }

  void set odomTopic(String topic) {
    prefs.setString('OdomTopic', topic);
  }
  //实列
}

Setting globalSetting = Setting();

// 初始化全局配置
Future<bool> initGlobalSetting() async {
  return globalSetting.init();
}
