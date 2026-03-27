import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:flutter/foundation.dart';

class MapManager extends ChangeNotifier {
  static const String _occupancyMapKey = 'occupancy_map';

  static final MapManager _instance = MapManager._internal();
  factory MapManager() => _instance;
  MapManager._internal();

  ValueNotifier<OccupancyMap> occupancyMap = ValueNotifier<OccupancyMap>(OccupancyMap());
  ValueNotifier<TopologyMap> topologyMap = ValueNotifier<TopologyMap>(TopologyMap(points: []));

  Future<void> init() async {
    await loadLocalOccupancyMap();
  }

  bool get hasPoint => topologyMap.value.points.isNotEmpty;
  List<NavPoint> get navPoints => topologyMap.value.points;
  List<TopologyRoute> get routes => topologyMap.value.routes;

  Future<int> getNextPointId() async {
    int nextId = 0;
    final existingIds = navPoints.map((point) {
      final match = RegExp(r'_(\d+)$').firstMatch(point.name);
      return match != null ? int.tryParse(match.group(1)!) ?? -1 : -1;
    }).toSet();
    
    while (existingIds.contains(nextId)) {
      nextId++;
    }
    return nextId;
  }

  void addNavPoint(NavPoint point) {
    final index = topologyMap.value.points.indexWhere((p) => p.name == point.name);
    if (index != -1) {
      topologyMap.value.points[index] = point;
    } else {
      topologyMap.value.points.add(point);
    }
    topologyMap.notifyListeners();
    notifyListeners();
  }

  void removeNavPoint(String name) {
    topologyMap.value.points.removeWhere((p) => p.name == name);
    topologyMap.value.routes.removeWhere((r) => r.fromPoint == name || r.toPoint == name);
    topologyMap.notifyListeners();
    notifyListeners();
  }

  void updateNavPoint(String name, NavPoint newPoint) {
    final index = topologyMap.value.points.indexWhere((p) => p.name == name);
    if (index != -1) {
      if (name != newPoint.name) {
        for (var route in topologyMap.value.routes) {
          if (route.fromPoint == name) route.fromPoint = newPoint.name;
          if (route.toPoint == name) route.toPoint = newPoint.name;
        }
      }
      topologyMap.value.points[index] = newPoint;
      topologyMap.notifyListeners();
      notifyListeners();
    }
  }

  NavPoint? getNavPoint(String name) {
    final index = topologyMap.value.points.indexWhere((p) => p.name == name);
    return index == -1 ? null : topologyMap.value.points[index];
  }

  void addRoute(TopologyRoute route) {
    final index = topologyMap.value.routes.indexWhere(
      (r) => r.fromPoint == route.fromPoint && r.toPoint == route.toPoint
    );
    if (index != -1) {
      topologyMap.value.routes[index] = route;
    } else {
      topologyMap.value.routes.add(route);
    }
    topologyMap.notifyListeners();
    notifyListeners();
  }

  void removeRoute(String fromPoint, String toPoint) {
    topologyMap.value.routes.removeWhere(
      (r) => r.fromPoint == fromPoint && r.toPoint == toPoint
    );
    topologyMap.notifyListeners();
    notifyListeners();
  }

  void updateRoute(String fromPoint, String toPoint, TopologyRoute newRoute) {
    final index = topologyMap.value.routes.indexWhere(
      (r) => r.fromPoint == fromPoint && r.toPoint == toPoint
    );
    if (index != -1) {
      topologyMap.value.routes[index] = newRoute;
      topologyMap.notifyListeners();
      notifyListeners();
    }
  }

  TopologyRoute? getRoute(String fromPoint, String toPoint) {
    final index = topologyMap.value.routes.indexWhere(
      (r) => r.fromPoint == fromPoint && r.toPoint == toPoint
    );
    return index == -1 ? null : topologyMap.value.routes[index];
  }

  void updateTopologyMapFromRos(TopologyMap rosTopologyMap) {
    topologyMap.value = rosTopologyMap;
    topologyMap.notifyListeners();
    notifyListeners();
    print('MapManager: 收到ROS拓扑地图，${rosTopologyMap.points.length}个点，${rosTopologyMap.routes.length}条路径');
  }

  void updateOccupancyMapFromRos(OccupancyMap rosOccupancyMap) {
    occupancyMap.value = rosOccupancyMap;
    occupancyMap.notifyListeners();
    notifyListeners();
  }

  void updateOccupancyMap(OccupancyMap map) {
    occupancyMap.value = map;
    occupancyMap.notifyListeners();
    notifyListeners();
  }

  void updateTopologyMap(TopologyMap map) {
    topologyMap.value = map;
    topologyMap.notifyListeners();
    notifyListeners();
  }

  Future<void> saveLocalOccupancyMap() async {
    final prefs = await SharedPreferences.getInstance();
    final map = occupancyMap.value;
    
    if (map.data.isEmpty) return;
    
    final json = {
      'width': map.mapConfig.width,
      'height': map.mapConfig.height,
      'resolution': map.mapConfig.resolution,
      'originX': map.mapConfig.originX,
      'originY': map.mapConfig.originY,
      'data': map.data.map((row) => row.toList()).toList(),
    };
    
    await prefs.setString(_occupancyMapKey, jsonEncode(json));
    print('MapManager: 栅格地图已保存到本地');
  }

  Future<void> loadLocalOccupancyMap() async {
    final prefs = await SharedPreferences.getInstance();
    final jsonStr = prefs.getString(_occupancyMapKey);
    
    if (jsonStr != null) {
      try {
        final json = jsonDecode(jsonStr) as Map<String, dynamic>;
        final map = OccupancyMap();
        map.mapConfig.width = json['width'] as int;
        map.mapConfig.height = json['height'] as int;
        map.mapConfig.resolution = (json['resolution'] as num).toDouble();
        map.mapConfig.originX = (json['originX'] as num).toDouble();
        map.mapConfig.originY = (json['originY'] as num).toDouble();
        
        final dataList = json['data'] as List;
        map.data = dataList.map((row) => (row as List).map((v) => v as int).toList()).toList();
        
        occupancyMap.value = map;
        occupancyMap.notifyListeners();
        print('MapManager: 从本地加载栅格地图');
      } catch (e) {
        print('MapManager: 加载本地栅格地图失败: $e');
      }
    }
  }

  Future<void> saveAll() async {
    await saveLocalOccupancyMap();
  }

  void clearAll() {
    topologyMap.value = TopologyMap(points: []);
    occupancyMap.value = OccupancyMap();
    topologyMap.notifyListeners();
    occupancyMap.notifyListeners();
    notifyListeners();
  }

  void setNavPoints(List<NavPoint> points) {
    topologyMap.value = TopologyMap(
      mapName: topologyMap.value.mapName,
      mapProperty: topologyMap.value.mapProperty,
      points: points,
      routes: topologyMap.value.routes,
    );
    topologyMap.notifyListeners();
    notifyListeners();
  }

  String exportTopologyToJson() {
    return jsonEncode(topologyMap.value.toJson());
  }

  Future<void> importTopologyFromJson(String jsonString) async {
    try {
      final json = jsonDecode(jsonString) as Map<String, dynamic>;
      topologyMap.value = TopologyMap.fromJson(json);
      topologyMap.notifyListeners();
      notifyListeners();
    } catch (e) {
      print('MapManager: 导入拓扑地图失败: $e');
      throw Exception('invalid_json');
    }
  }
}

