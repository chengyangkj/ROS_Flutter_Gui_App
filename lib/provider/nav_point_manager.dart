import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';

class NavPointManager {
  static const String _navPointsKey = 'nav_points';
  static const String _navPointsCounterKey = 'nav_points_counter';
  
  // 单例模式
  static final NavPointManager _instance = NavPointManager._internal();
  factory NavPointManager() => _instance;
  NavPointManager._internal();

  // 导航点列表
  List<NavPoint> _navPoints = [];
  
  // 获取导航点列表
  List<NavPoint> get navPoints => List.unmodifiable(_navPoints);
  
  // 添加导航点
  Future<void> addNavPoint(double x, double y, double theta, String name) async {
    final counter = await _getNextId();
    final navPoint = NavPoint(
      id: 'nav_point_$counter',
      x: x,
      y: y,
      theta: theta,
      name: name,
      createdAt: DateTime.now(),
    );
    
    _navPoints.add(navPoint);
    await _saveNavPoints();
  }
  
  // 删除导航点
  Future<void> removeNavPoint(String id) async {
    _navPoints.removeWhere((point) => point.id == id);
    await _saveNavPoints();
  }
  
  // 更新导航点
  Future<void> updateNavPoint(String id, {double? x, double? y, double? theta, String? name}) async {
    final index = _navPoints.indexWhere((point) => point.id == id);
    if (index != -1) {
      final oldPoint = _navPoints[index];
      _navPoints[index] = oldPoint.copyWith(
        x: x ?? oldPoint.x,
        y: y ?? oldPoint.y,
        theta: theta ?? oldPoint.theta,
        name: name ?? oldPoint.name,
      );
      await _saveNavPoints();
    }
  }
  
  // 获取下一个ID
  Future<int> _getNextId() async {
    final prefs = await SharedPreferences.getInstance();
    final counter = prefs.getInt(_navPointsCounterKey) ?? 0;
    await prefs.setInt(_navPointsCounterKey, counter + 1);
    return counter;
  }
  
  // 保存导航点到本地
  Future<void> _saveNavPoints() async {
    final prefs = await SharedPreferences.getInstance();
    final navPointsJson = _navPoints.map((point) => point.toJson()).toList();
    await prefs.setString(_navPointsKey, jsonEncode(navPointsJson));
  }
  
  // 从本地加载导航点
  Future<void> loadNavPoints() async {
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
  }
  
  // 清空所有导航点
  Future<void> clearAllNavPoints() async {
    _navPoints.clear();
    await _saveNavPoints();
    
    // 重置计数器
    final prefs = await SharedPreferences.getInstance();
    await prefs.setInt(_navPointsCounterKey, 0);
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
      await _saveNavPoints();
    } catch (e) {
      print('导入导航点失败: $e');
      throw Exception('导入失败：无效的JSON格式');
    }
  }
}
