import 'dart:convert';

import 'package:http/http.dart' as http;

import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';

class HttpChannel {
  Uri _buildUri(String path, {Map<String, String>? queryParameters}) {
    final base = globalSetting.tileServerUrl;
    return Uri.parse('$base$path').replace(queryParameters: queryParameters);
  }

  Future<List<String>> getAllMapList() async {
    final uri = _buildUri('/getAllMapList');
    final res = await http.get(uri);
    if (res.statusCode != 200) {
      throw Exception('getAllMapList failed: ${res.statusCode} ${res.body}');
    }
    final list = jsonDecode(res.body) as List<dynamic>;
    return list.map((e) => e.toString()).toList();
  }

  Future<String> getCurrentMap() async {
    final uri = _buildUri('/currentMap');
    final res = await http.get(uri);
    if (res.statusCode != 200) {
      throw Exception('getCurrentMap failed: ${res.statusCode} ${res.body}');
    }
    final j = jsonDecode(res.body) as Map<String, dynamic>;
    return j['current_map'] as String? ?? '';
  }

  Future<void> setCurrentMap(String name) async {
    final uri = _buildUri('/setCurrentMap', queryParameters: {'name': name});
    final res = await http.get(uri);
    if (res.statusCode != 200) {
      throw Exception('setCurrentMap failed: ${res.statusCode} ${res.body}');
    }
  }

  Future<TopologyMap> getTopologyMap({String? mapName}) async {
    final uri = mapName != null && mapName.isNotEmpty
        ? _buildUri('/getTopologyMap', queryParameters: {'map_name': mapName})
        : _buildUri('/getTopologyMap');
    final res = await http.get(uri);
    if (res.statusCode == 404) {
      return TopologyMap(points: []);
    }
    if (res.statusCode != 200) {
      throw Exception('getTopologyMap failed: ${res.statusCode} ${res.body}');
    }
    final j = jsonDecode(res.body) as Map<String, dynamic>;
    return TopologyMap.fromJson(j);
  }

  Future<void> updateMapEdit({
    required String editSessionId,
    required TopologyMap topologyMap,
    required Map<int, int> obstacleEdits,
    String? mapName,
  }) async {
    final name = mapName ?? await getCurrentMap();
    if (name.isEmpty) {
      throw Exception('当前无地图可用，请先选择或创建地图');
    }
    final obstacleEditsJson = obstacleEdits
        .map((cellIndex, value) => MapEntry(cellIndex.toString(), value));
    final topologyJson = topologyMap.toJson();
    topologyJson['map_name'] = name;
    final uri = _buildUri(
      '/updateMapEdit',
      queryParameters: <String, String>{
        'session_id': editSessionId,
        'map_name': name,
        'topology_json': jsonEncode(topologyJson),
        'obstacle_edits_json': jsonEncode(obstacleEditsJson),
      },
    );
    final res = await http.get(uri);
    if (res.statusCode != 200) {
      throw Exception('updateMapEdit failed: ${res.statusCode} ${res.body}');
    }
  }
}
