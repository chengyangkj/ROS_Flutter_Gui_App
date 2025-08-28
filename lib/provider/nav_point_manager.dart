import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:flutter/foundation.dart';

class NavPointManager extends ChangeNotifier {
  static const String _navPointsKey = 'topology_points';
  
  // 单例模式
  static final NavPointManager _instance = NavPointManager._internal();
  factory NavPointManager() => _instance;
  NavPointManager._internal();

  // 导航点列表
  List<NavPoint> _navPoints = [];
  
  // 获取导航点列表
  List<NavPoint> get navPoints => List.unmodifiable(_navPoints);
  
  // 添加导航点
  Future<NavPoint> addNavPoint(double x, double y, double theta, String name) async {
    final counter = await getNextId();
    final navPoint = NavPoint(
      x: x,
      y: y,
      theta: theta,
      name: name,
      type: NavPointType.navGoal,
    );
    
    _navPoints.add(navPoint);
    await saveNavPoints(_navPoints);
    notifyListeners(); // 通知监听器数据已变化
    return navPoint;
  }
  
  // 删除导航点
  Future<void> removeNavPoint(String name) async {
    _navPoints.removeWhere((point) => point.name == name);
    await saveNavPoints(_navPoints);
    notifyListeners(); // 通知监听器数据已变化
  }
  

  
  // 获取下一个ID
  Future<int> getNextId() async {
    // 查找可用的最小ID
    int nextId = 0;
    final existingIds = navPoints.map((point) => int.tryParse(point.name.split('_')[1]) ?? -1).toSet();
    
    while (existingIds.contains(nextId)) {
      nextId++;
    }
    return nextId;
  }


  
  // 保存导航点到本地
  Future<void> saveNavPoints(List<NavPoint> navPoints) async {
    _navPoints = navPoints;
    final prefs = await SharedPreferences.getInstance();
    final navPointsJson = navPoints.map((point) => point.toJson()).toList();
    await prefs.setString(_navPointsKey, jsonEncode(navPointsJson));
    notifyListeners(); // 通知监听器数据已变化
  }
  
  // 从本地加载导航点
  Future<List<NavPoint>> loadNavPoints() async {
    final prefs = await SharedPreferences.getInstance();
    final navPointsStr = prefs.getString(_navPointsKey);
    
    if (navPointsStr != null) {
      try {
        final navPointsJson = jsonDecode(navPointsStr) as List;
        _navPoints = navPointsJson
            .map((json) => NavPoint.fromJson(json as Map<String, dynamic>))
            .toList();
      } catch (e) {
        print('加载导航点失败: $e');
        _navPoints = [];
      }
    }
    return _navPoints;
  }
  
  // 清空所有导航点
  Future<void> clearAllNavPoints() async {
    _navPoints.clear();
    await saveNavPoints(_navPoints);
    notifyListeners(); // 通知监听器数据已变化
  }
  
  // 导出导航点为JSON字符串
  String exportToJson() {
    final navPointsJson = _navPoints.map((point) => point.toJson()).toList();
    return jsonEncode(navPointsJson);
  }
  
  // 从JSON字符串导入导航点
  Future<void> importFromJson(String jsonString) async {
    try {
      final navPointsJson = jsonDecode(jsonString) as List;
      _navPoints = navPointsJson
          .map((json) => NavPoint.fromJson(json as Map<String, dynamic>))
          .toList();
      await saveNavPoints(_navPoints);
      notifyListeners(); // 通知监听器数据已变化
    } catch (e) {
      print('导入导航点失败: $e');
      throw Exception('导入失败：无效的JSON格式');
    }
  }
}
