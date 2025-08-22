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
  ValueNotifier<Mode> mode = ValueNotifier(Mode.normal);
  ValueNotifier<bool> isManualCtrl = ValueNotifier(false);
  
  // 图层开关状态 - 使用Map存储
  final Map<String, ValueNotifier<bool>> _layerStates = {
    'showGrid': ValueNotifier(true),
    'showGlobalCostmap': ValueNotifier(false),
    'showLocalCostmap': ValueNotifier(false),
    'showLaser': ValueNotifier(false),
    'showPointCloud': ValueNotifier(false),
    'showGlobalPath': ValueNotifier(true),
    'showLocalPath': ValueNotifier(true),
    'showTopology': ValueNotifier(true),
    'showRobotFootprint': ValueNotifier(true),
  };
  
  // 保存进入编辑模式前的图层状态
  Map<String, bool>? _previousLayerStates;

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
  
  // 进入地图编辑模式
  void enterMapEditMode() {
    mode.value = Mode.mapEdit;
    
    // 保存当前图层状态
    _previousLayerStates = <String, bool>{};
    _layerStates.forEach((key, valueNotifier) {
      _previousLayerStates![key] = valueNotifier.value;
    });
    
    // 关闭所有图层，只保留地图和网格
    setLayerState('showGlobalCostmap', false);
    setLayerState('showLocalCostmap', false);
    setLayerState('showLaser', false);
    setLayerState('showPointCloud', false);
    setLayerState('showGlobalPath', false);
    setLayerState('showLocalPath', false);
    setLayerState('showTopology', false);
  }
  
  // 退出地图编辑模式
  void exitMapEditMode() {
    mode.value = Mode.normal;
    
    // 恢复进入编辑模式前的图层状态
    if (_previousLayerStates != null) {
      _previousLayerStates!.forEach((key, value) {
        setLayerState(key, value);
      });
      _previousLayerStates = null; // 清除保存的状态
    } else {
      // 如果没有保存的状态，使用默认值
      setLayerState('showGlobalCostmap', false);
      setLayerState('showLocalCostmap', false);
      setLayerState('showLaser', false);
      setLayerState('showPointCloud', false);
      setLayerState('showGlobalPath', true);
      setLayerState('showLocalPath', true);
      setLayerState('showTopology', true);
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
