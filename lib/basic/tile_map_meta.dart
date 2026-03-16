import 'dart:convert';
import 'dart:math' show pow;
import 'dart:ui';

import 'package:flutter_map/flutter_map.dart';
import 'package:http/http.dart' as http;
import 'package:latlong2/latlong.dart';
import 'package:proj4dart/proj4dart.dart' as proj4;
import 'package:vector_math/vector_math_64.dart' as vm;

class MapMeta {
  MapMeta({
    required this.resolution,
    required this.originX,
    required this.originY,
    required this.width,
    required this.height,
    required this.maxZoom,
    required this.extraZoomLevels,
  });

  final double resolution;
  final double originX;
  final double originY;
  final int width;
  final int height;
  final int maxZoom;
  final int extraZoomLevels;

  vm.Vector2 idx2xy(vm.Vector2 idx) {
    final x = idx.x * resolution + originX;
    final y = originY + (height - idx.y) * resolution;
    return vm.Vector2(x, y);
  }

  static Future<MapMeta> fetch(String baseUrl) async {
    final uri = Uri.parse('$baseUrl/tiles/meta');
    final res = await http.get(uri);
    if (res.statusCode != 200) {
      throw Exception('Failed to load map meta: ${res.statusCode}');
    }
    final j = jsonDecode(res.body) as Map<String, dynamic>;
    return MapMeta(
      resolution: (j['resolution'] as num).toDouble(),
      originX: (j['origin_x'] as num).toDouble(),
      originY: (j['origin_y'] as num).toDouble(),
      width: j['width'] as int,
      height: j['height'] as int,
      maxZoom: j['max_zoom'] as int,
      extraZoomLevels: j['extra_zoom_levels'] as int? ?? 1,
    );
  }
}

Future<bool> setExtraZoomLevels(String baseUrl, int value) async {
  final uri = Uri.parse('$baseUrl/tiles/config');
  final res = await http.post(
    uri,
    headers: {'Content-Type': 'application/json'},
    body: '{"extra_zoom_levels": $value}',
  );
  return res.statusCode == 200;
}

double getMapSize(MapMeta meta) => 256.0 * pow(2, meta.maxZoom);

LatLng worldToLatLng(MapMeta meta, double worldX, double worldY) {
  final mapSize = getMapSize(meta);
  final scale = pow(2, meta.extraZoomLevels).toDouble();

  double px = (worldX - meta.originX) / meta.resolution;
  double py = meta.height - (worldY - meta.originY) / meta.resolution;
  px *= scale;
  py *= scale;
  final lat = (py * 180 / mapSize - 90).clamp(-90.0, 90.0);
  final lng = (px * 360 / mapSize - 180).clamp(-180.0, 180.0);
  return LatLng(lat, lng);
}

vm.Vector2 latLngToWorld(MapMeta meta, LatLng latLng) {
  final mapSize = getMapSize(meta);
  final scale = pow(2, meta.extraZoomLevels).toDouble();

  double py = (latLng.latitude + 90) * mapSize / 180;
  double px = (latLng.longitude + 180) * mapSize / 360;
  py /= scale;
  px /= scale;
  final worldX = px * meta.resolution + meta.originX;
  final worldY = meta.originY + (meta.height - py) * meta.resolution;
  return vm.Vector2(worldX, worldY);
}

LatLngBounds getTileBounds() => LatLngBounds(
  const LatLng(-90, -180),
  const LatLng(90, 180),
);

Crs createLocalMapCrs(MapMeta meta) {
  final mapSize = 256.0 * pow(2, meta.maxZoom);
  final bounds = Rect.fromLTRB(-180, -90, 180, 90);
  const maxScaleZoom = 24;
  final scales = List.generate(
    maxScaleZoom + 1,
    (z) => 256.0 * pow(2, z) / mapSize,
  );
  return Proj4Crs.fromFactory(
    code: 'LOCAL_MAP',
    proj4Projection: proj4.Projection.add(
      'LOCAL_MAP',
      '+proj=longlat +datum=WGS84 +no_defs',
    ),
    bounds: bounds,
    scales: scales,
  );
}
