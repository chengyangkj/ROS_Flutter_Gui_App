import 'package:flutter/cupertino.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'dart:convert';

enum Mode {
  normal,
  reloc, //重定位模式
  addNavPoint, //添加导航点模式
  robotFixedCenter, //机器人固定屏幕中心模式
  mapEdit, //地图编辑模式
}

class GlobalState extends ChangeNotifier {
  ValueNotifier<bool> isManualCtrl = ValueNotifier(false);
  ValueNotifier<Mode> mode = ValueNotifier(Mode.normal);
  // 图层开关状态 - 使用Map存储
  final Map<String, ValueNotifier<bool>> _layerStates = {
    'showGrid': ValueNotifier(true),
    'showGlobalCostmap': ValueNotifier(false),
    'showLocalCostmap': ValueNotifier(false),
    'showLaser': ValueNotifier(false),
    'showPointCloud': ValueNotifier(false),
    'showGlobalPath': ValueNotifier(true),
    'showLocalPath': ValueNotifier(true),
    'showTracePath': ValueNotifier(true),
    'showTopology': ValueNotifier(true),
    'showRobotFootprint': ValueNotifier(true),
  };
  


  // 图层状态管理
  static const String _layerSettingsKey = 'layer_settings';
  
  // 获取图层状态
  ValueNotifier<bool> getLayerState(String layerName) {
    return _layerStates[layerName] ?? ValueNotifier(false);
  }
  
  // 设置图层状态
  void setLayerState(String layerName, bool value) {
    final state = _layerStates[layerName];
    if (state != null) {
      state.value = value;
      saveLayerSettings();
    }
  }
  
  // 切换图层状态
  void toggleLayer(String layerName) {
    final state = _layerStates[layerName];
    if (state != null) {
      state.value = !state.value;
      saveLayerSettings();
    }
  }

  // 保存所有图层状态到设置
  Future<void> saveLayerSettings() async {
    final prefs = await SharedPreferences.getInstance();
    final layerSettings = <String, bool>{};
    
    _layerStates.forEach((key, valueNotifier) {
      layerSettings[key] = valueNotifier.value;
    });
    
    await prefs.setString(_layerSettingsKey, jsonEncode(layerSettings));
  }
  
  // 从设置加载图层状态
  Future<void> loadLayerSettings() async {
    final prefs = await SharedPreferences.getInstance();
    final layerSettingsStr = prefs.getString(_layerSettingsKey);
    
    if (layerSettingsStr != null) {
      try {
        final layerSettings = Map<String, dynamic>.from(
          jsonDecode(layerSettingsStr)
        );
        
        _layerStates.forEach((key, valueNotifier) {
          if (layerSettings.containsKey(key)) {
            valueNotifier.value = layerSettings[key] ?? false;
          }
        });
      } catch (e) {
        print('加载图层设置失败: $e');
      }
    }
  }
  
  // 获取所有图层名称
  List<String> get layerNames => _layerStates.keys.toList();
  
  // 检查图层是否可见
  bool isLayerVisible(String layerName) {
    return _layerStates[layerName]?.value ?? false;
  }
}
