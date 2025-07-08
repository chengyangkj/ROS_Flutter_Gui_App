import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:package_info_plus/package_info_plus.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:package_info_plus/package_info_plus.dart';
import 'dart:convert';

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
  bool reverse = false; //是否反转(反转填-1)Q
  double maxValue = 32767;
  double minValue = -32767;
  double value = 0;

  JoyStickEvent(this.keyName,
      {this.reverse = false, this.maxValue = 32767, this.minValue = -32767});
}

enum TempConfigType {
  ROS2Default,
  ROS1,
  TurtleBot3,
  TurtleBot4,
  Jackal,
}

String tempConfigTypeToString(TempConfigType type) {
  return type.toString().split('.').last;
}

class Setting {
  late SharedPreferences prefs;

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
    "KEYCODE_BUTTON_L1": JoyStickEvent(KeyName.buttonLB,
        maxValue: 1, minValue: 0, reverse: true),
    "KEYCODE_BUTTON_R1": JoyStickEvent(KeyName.buttonRB,
        maxValue: 1, minValue: 0, reverse: true),
  };

  Future<bool> init() async {
    prefs = await SharedPreferences.getInstance();

    // 获取应用版本
    PackageInfo packageInfo = await PackageInfo.fromPlatform();
    String currentVersion = packageInfo.version;

    if (!prefs.containsKey("version") ||
        prefs.getString("version") != currentVersion) {
      setDefaultCfgRos2();
      prefs.setString("version", currentVersion);
    }

    // 从配置中加载手柄映射
    await _loadGamepadMapping();

    return true;
  }

  //设置语言回调
  late Function(Locale locale) setLanguage;

  Future<void> _loadGamepadMapping() async {
    final mappingStr = prefs.getString('gamepadMapping');
    print(mappingStr);
    if (mappingStr != null) {
      try {
        final mapping = jsonDecode(mappingStr);

        // 清空现有映射
        axisMapping.clear();
        buttonMapping.clear();

        // 加载 axisMapping
        if (mapping['axisMapping'] != null) {
          (mapping['axisMapping'] as Map<String, dynamic>)
              .forEach((key, value) {
            final keyName = _parseKeyName(value['keyName']);
            axisMapping[key] = JoyStickEvent(
              keyName,
              maxValue: value['maxValue'] ?? 32767,
              minValue: value['minValue'] ?? -32767,
              reverse: value['reverse'] ?? false,
            );
          });
        }

        // 加载 buttonMapping
        if (mapping['buttonMapping'] != null) {
          (mapping['buttonMapping'] as Map<String, dynamic>)
              .forEach((key, value) {
            final keyName = _parseKeyName(value['keyName']);
            buttonMapping[key] = JoyStickEvent(
              keyName,
              maxValue: value['maxValue'] ?? 1,
              minValue: value['minValue'] ?? 0,
              reverse: value['reverse'] ?? true,
            );
          });
        }
      } catch (e) {
        print('Error loading gamepad mapping: $e');
        // 如果加载失败，使用默认映射
        resetGamepadMapping();
      }
    }
  }

  KeyName _parseKeyName(String keyNameStr) {
    // 移除 'KeyName.' 前缀
    final enumStr = keyNameStr.replaceAll('KeyName.', '');
    return KeyName.values.firstWhere(
      (e) => e.toString() == 'KeyName.$enumStr',
      orElse: () => KeyName.None,
    );
  }

  Future<void> saveGamepadMapping() async {
    // 将默认映射保存到配置中
    final mapping = {
      'axisMapping': axisMapping.map((key, value) => MapEntry(key, {
            'keyName': value.keyName.toString(),
            'maxValue': value.maxValue,
            'minValue': value.minValue,
            'reverse': value.reverse,
          })),
      'buttonMapping': buttonMapping.map((key, value) => MapEntry(key, {
            'keyName': value.keyName.toString(),
            'maxValue': value.maxValue,
            'minValue': value.minValue,
            'reverse': value.reverse,
          })),
    };
    print(jsonEncode(mapping));
    await prefs.setString('gamepadMapping', jsonEncode(mapping));
  }

  Future<void> resetGamepadMapping() async {
    axisMapping.clear();
    buttonMapping.clear();

    // 恢复默认的轴映射
    axisMapping.addAll({
      "AXIS_X": JoyStickEvent(KeyName.leftAxisX),
      "AXIS_Y": JoyStickEvent(KeyName.leftAxisY),
      "AXIS_Z": JoyStickEvent(KeyName.rightAxisX),
      "AXIS_RZ": JoyStickEvent(KeyName.rightAxisY),
      "triggerRight": JoyStickEvent(KeyName.triggerRight),
      "triggerLeft": JoyStickEvent(KeyName.triggerLeft),
      "buttonLeftRight": JoyStickEvent(KeyName.buttonLeftRight),
      "buttonUpDown": JoyStickEvent(KeyName.buttonUpDown),
    });

    // 恢复默认的按钮映射
    buttonMapping.addAll({
      "KEYCODE_BUTTON_A": JoyStickEvent(KeyName.buttonA,
          maxValue: 1, minValue: 0, reverse: true),
      "KEYCODE_BUTTON_B": JoyStickEvent(KeyName.buttonB,
          maxValue: 1, minValue: 0, reverse: true),
      "KEYCODE_BUTTON_X": JoyStickEvent(KeyName.buttonX,
          maxValue: 1, minValue: 0, reverse: true),
      "KEYCODE_BUTTON_Y": JoyStickEvent(KeyName.buttonY,
          maxValue: 1, minValue: 0, reverse: true),
      "KEYCODE_BUTTON_L1": JoyStickEvent(KeyName.buttonLB,
          maxValue: 1, minValue: 0, reverse: true),
      "KEYCODE_BUTTON_R1": JoyStickEvent(KeyName.buttonRB,
          maxValue: 1, minValue: 0, reverse: true),
    });

    // 将默认映射保存到配置中
    final mapping = {
      'axisMapping': axisMapping.map((key, value) => MapEntry(key, {
            'keyName': value.keyName.toString(),
            'maxValue': value.maxValue,
            'minValue': value.minValue,
            'reverse': value.reverse,
          })),
      'buttonMapping': buttonMapping.map((key, value) => MapEntry(key, {
            'keyName': value.keyName.toString(),
            'maxValue': value.maxValue,
            'minValue': value.minValue,
            'reverse': value.reverse,
          })),
    };

    await prefs.setString('gamepadMapping', jsonEncode(mapping));
  }

  void setDefaultCfgRos2Jackal() {
    prefs.setInt("tempConfig", TempConfigType.Jackal.index);
    prefs.setString('mapTopic', "map");
    prefs.setString('laserTopic', "/sensors/lidar_0/scan");
    prefs.setString('pointCloud2Topic', "/sensors/lidar_0/points");
    prefs.setString('globalPathTopic', "/plan");
    prefs.setString('localPathTopic', "/plan");
    prefs.setString('relocTopic', "/initialpose");
    prefs.setString('navGoalTopic', "/goal_pose");
    prefs.setString('OdometryTopic', "/platform/odom/filtered");
    prefs.setString('SpeedCtrlTopic', "/cmd_vel");
    prefs.setString('BatteryTopic', "/battery_status");
    prefs.setString('robotFootprintTopic', "/local_costmap/published_footprint");
    prefs.setString('localCostmapTopic', "/local_costmap/costmap");
    prefs.setString('MaxVx', "0.9");
    prefs.setString('MaxVy', "0.9");
    prefs.setString('MaxVw', "0.9");
    prefs.setString('mapFrameName', "map");
    prefs.setString('baseLinkFrameName', "base_link");
    prefs.setString('imagePort', "8080");
    prefs.setString('imageTopic', "/camera/image_raw");
    prefs.setDouble('imageWidth', 640);
    prefs.setDouble('imageHeight', 480);
  }

  void setDefaultCfgRos2TB4() {
    prefs.setInt("tempConfig", TempConfigType.TurtleBot4.index);
    prefs.setString('mapTopic', "map");
    prefs.setString('laserTopic', "scan");
    prefs.setString('pointCloud2Topic', "points");
    prefs.setString('globalPathTopic', "/plan");
    prefs.setString('localPathTopic', "/local_plan");
    prefs.setString('relocTopic', "/initialpose");
    prefs.setString('navGoalTopic', "/goal_pose");
    prefs.setString('OdometryTopic', "/odom");
    prefs.setString('SpeedCtrlTopic', "/cmd_vel");
    prefs.setString('BatteryTopic', "/battery_status");
    prefs.setString('robotFootprintTopic', "/local_costmap/published_footprint");
    prefs.setString('localCostmapTopic', "/local_costmap/costmap");
    prefs.setString('MaxVx', "0.9");
    prefs.setString('MaxVy', "0.9");
    prefs.setString('MaxVw', "0.9");
    prefs.setString('mapFrameName', "map");
    prefs.setString('baseLinkFrameName', "base_link");
    prefs.setString('imagePort', "8080");
    prefs.setString('imageTopic', "/camera/image_raw");
    prefs.setDouble('imageWidth', 640);
    prefs.setDouble('imageHeight', 480);
  }

  void setDefaultCfgRos2TB3() {
    prefs.setInt("tempConfig", TempConfigType.TurtleBot3.index);
    prefs.setString('mapTopic', "map");
    prefs.setString('laserTopic', "scan");
    prefs.setString('pointCloud2Topic', "points");
    prefs.setString('globalPathTopic', "/plan");
    prefs.setString('localPathTopic', "/local_plan");
    prefs.setString('relocTopic', "/initialpose");
    prefs.setString('navGoalTopic', "/goal_pose");
    prefs.setString('OdometryTopic', "/odom");
    prefs.setString('SpeedCtrlTopic', "/cmd_vel");
    prefs.setString('BatteryTopic', "/battery_status");
    prefs.setString('robotFootprintTopic', "/local_costmap/published_footprint");
    prefs.setString('localCostmapTopic', "/local_costmap/costmap");
    prefs.setString('MaxVx', "0.9");
    prefs.setString('MaxVy', "0.9");
    prefs.setString('MaxVw', "0.9");
    prefs.setString('mapFrameName', "map");
    prefs.setString('baseLinkFrameName', "base_link");
    prefs.setString('imagePort', "8080");
    prefs.setString('imageTopic', "/camera/image_raw");
    prefs.setDouble('imageWidth', 640);
    prefs.setDouble('imageHeight', 480);
  }

  void setDefaultCfgRos2() {
    prefs.setInt("tempConfig", TempConfigType.ROS2Default.index);
    prefs.setString('mapTopic', "map");
    prefs.setString('laserTopic', "scan");
    prefs.setString('pointCloud2Topic', "points");
    prefs.setString('globalPathTopic', "/plan");
    prefs.setString('localPathTopic', "/local_plan");
    prefs.setString('relocTopic', "/initialpose");
    prefs.setString('navGoalTopic', "/goal_pose");
    prefs.setString('OdometryTopic', "/wheel/odometry");
    prefs.setString('SpeedCtrlTopic', "/cmd_vel");
    prefs.setString('BatteryTopic', "/battery_status");
    prefs.setString('robotFootprintTopic', "/local_costmap/published_footprint");
    prefs.setString('localCostmapTopic', "/local_costmap/costmap");
    prefs.setString('MaxVx', "0.9");
    prefs.setString('MaxVy', "0.9");
    prefs.setString('MaxVw', "0.9");
    prefs.setString('mapFrameName', "map");
    prefs.setString('baseLinkFrameName', "base_link");
    prefs.setString('imagePort', "8080");
    prefs.setString('imageTopic', "/camera/image_raw");
    prefs.setDouble('imageWidth', 640);
    prefs.setDouble('imageHeight', 480);
  }

  void setDefaultCfgRos1() {
    prefs.setInt("tempConfig", TempConfigType.ROS1.index);
    prefs.setString('mapTopic', "map");
    prefs.setString('laserTopic', "scan");
    prefs.setString('pointCloud2Topic', "points");
    prefs.setString('globalPathTopic', "/move_base/DWAPlannerROS/global_plan");
    prefs.setString('localPathTopic', "/move_base/DWAPlannerROS/local_plan");
    prefs.setString('relocTopic', "/initialpose");
    prefs.setString('navGoalTopic', "move_base_simple/goal");
    prefs.setString('OdometryTopic', "/odom");
    prefs.setString('SpeedCtrlTopic', "/cmd_vel");
    prefs.setString('BatteryTopic', "/battery_status");
    prefs.setString('robotFootprintTopic', "/local_costmap/published_footprint");
    prefs.setString('localCostmapTopic', "/local_costmap/costmap");
    prefs.setString('MaxVx', "0.9");
    prefs.setString('MaxVy', "0.9");
    prefs.setString('MaxVw', "0.9");
    prefs.setString('mapFrameName', "map");
    prefs.setString('baseLinkFrameName', "base_link");
    prefs.setString('imagePort', "8080");
    prefs.setString('imageTopic', "/camera/rgb/image_raw");
    prefs.setDouble('imageWidth', 640);
    prefs.setDouble('imageHeight', 480);
  }

  SharedPreferences get config {
    return prefs;
  }

  double get imageWidth {
    return prefs.getDouble("imageWidth") ?? 640;
  }

  double get imageHeight {
    return prefs.getDouble("imageHeight") ?? 480;
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

  String get robotPort {
    return prefs.getString("robotPort") ?? "9090";
  }

  String get robotFootprintTopic {
    return prefs.getString("robotFootprintTopic") ?? "/local_costmap/published_footprint";
  }

  void setRobotFootprintTopic(String topic) {
    prefs.setString('robotFootprintTopic', topic);
  }

  String get localCostmapTopic {
    return prefs.getString("localCostmapTopic") ?? "/local_costmap/costmap";
  } 

  void setLocalCostmapTopic(String topic) {
    prefs.setString('localCostmapTopic', topic);
  }

  void setMapTopic(String topic) {
    prefs.setString('mapTopic', topic);
  }

  String get mapTopic {
    return prefs.getString("mapTopic") ?? "map";
  }

  String get topologyMapTopic {
    return prefs.getString("topologyMapTopic") ?? "map/topology";
  }

  String get navToPoseStatusTopic {
    return prefs.getString("navToPoseStatusTopic") ??
        "navigate_to_pose/_action/status";
  }

  String get navThroughPosesStatusTopic {
    return prefs.getString("navThroughPosesStatusTopic") ??
        "navigate_through_poses/_action/status";
  }

  void setLaserTopic(String topic) {
    prefs.setString('laserTopic', topic);
  }

  String get laserTopic {
    return prefs.getString("laserTopic") ?? "scan";
  }

  void setPointCloud2Topic(String topic) {
    prefs.setString('pointCloud2Topic', topic);
  }

  String get pointCloud2Topic {
    return prefs.getString("pointCloud2Topic") ?? "points";
  }

  void setGloalPathTopic(String topic) {
    prefs.setString('globalPathTopic', topic);
  }

  String get globalPathTopic {
    return prefs.getString("globalPathTopic") ?? "plan";
  }

  void setLocalPathTopic(String topic) {
    prefs.setString('localPathTopic', topic);
  }

  String get localPathTopic {
    return prefs.getString("localPathTopic") ?? "/local_plan";
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

  String get navGoalTopic {
    return prefs.getString("navGoalTopic") ?? "/goal_pose";
  }

  void setNavGoalTopic(String topic) {
    prefs.setString('navGoalTopic', topic);
  }

  String get batteryTopic {
    return prefs.getString("BatteryTopic") ?? "/battery_status";
  }

  void setBatteryTopic(String topic) {
    prefs.setString('BatteryTopic', topic);
  }

  String getConfig(String key) {
    return prefs.getString(key) ?? "";
  }

  String get odomTopic {
    return prefs.getString("OdometryTopic") ?? "/wheel/odometry";
  }

  void setOdomTopic(String topic) {
    prefs.setString('OdometryTopic', topic);
  }

  // 添加速度控制相关的方法
  void setSpeedCtrlTopic(String topic) {
    prefs.setString('SpeedCtrlTopic', topic);
  }

  String get speedCtrlTopic {
    return prefs.getString("SpeedCtrlTopic") ?? "/cmd_vel";
  }

  // 添加最大速度设置方法
  void setMaxVx(String value) {
    prefs.setString('MaxVx', value);
  }

  void setMaxVy(String value) {
    prefs.setString('MaxVy', value);
  }

  void setMaxVw(String value) {
    prefs.setString('MaxVw', value);
  }

  TempConfigType get tempConfig {
    return TempConfigType.values[prefs.getInt("tempConfig") ?? 0];
  }

  // 添加最大速度获取方法
  double get maxVx {
    return double.parse(prefs.getString("MaxVx") ?? "0.1");
  }

  double get maxVy {
    return double.parse(prefs.getString("MaxVy") ?? "0.1");
  }

  double get maxVw {
    return double.parse(prefs.getString("MaxVw") ?? "0.3");
  }

  // 添加图像设置方法
  void setImagePort(String port) {
    prefs.setString('imagePort', port);
  }

  void setImageTopic(String topic) {
    prefs.setString('imageTopic', topic);
  }

  void setImageWidth(double width) {
    prefs.setDouble('imageWidth', width);
  }

  void setImageHeight(double height) {
    prefs.setDouble('imageHeight', height);
  }

  // 添加框架名称设置方法
  void setMapFrameName(String name) {
    prefs.setString('mapFrameName', name);
  }

  void setBaseLinkFrameName(String name) {
    prefs.setString('baseLinkFrameName', name);
  }

  // 添加通用配置设置方法
  void setConfig(String key, String value) {
    prefs.setString(key, value);
  }

  // 基本设置相关方法
  void setRobotIp(String ip) {
    prefs.setString('robotIp', ip);
  }

  void setRobotPort(String port) {
    prefs.setString('robotPort', port);
  }

  // 地图相关方法

  void setMapMetadataTopic(String topic) {
    prefs.setString('mapMetadataTopic', topic);
  }

  // 定位相关方法

  void setInitPoseTopic(String topic) {
    prefs.setString('initPoseTopic', topic);
  }

  void setAmclPoseTopic(String topic) {
    prefs.setString('amclPoseTopic', topic);
  }

  // 导航相关方法
  void setMoveBaseTopic(String topic) {
    prefs.setString('moveBaseTopic', topic);
  }

  void setCmdVelTopic(String topic) {
    prefs.setString('cmdVelTopic', topic);
  }

  void setGlobalPlanTopic(String topic) {
    prefs.setString('globalPlanTopic', topic);
  }

  void setLocalPlanTopic(String topic) {
    prefs.setString('localPlanTopic', topic);
  }

  void setGlobalCostmapTopic(String topic) {
    prefs.setString('globalCostmapTopic', topic);
  }

  void setGlobalPathTopic(String topic) {
    prefs.setString('globalPathTopic', topic);
  }

  // 状态监控相关方法
  void setRobotStatusTopic(String topic) {
    prefs.setString('robotStatusTopic', topic);
  }

  void setJointStatesTopic(String topic) {
    prefs.setString('jointStatesTopic', topic);
  }
  
}

Setting globalSetting = Setting();

// 初始化全局配置
Future<bool> initGlobalSetting() async {
  return globalSetting.init();
}
