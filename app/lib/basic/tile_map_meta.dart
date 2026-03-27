import 'dart:convert';
import 'dart:math' as math show pow;
import 'dart:ui';

import 'package:flutter_map/flutter_map.dart';
import 'package:http/http.dart' as http;
import 'package:latlong2/latlong.dart';
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

double getMapSize(MapMeta meta) => 256.0 * math.pow(2, meta.maxZoom);

LatLng worldToLatLng(MapMeta meta, double worldX, double worldY) {
  final mapSize = getMapSize(meta);
  final scale = math.pow(2, meta.extraZoomLevels).toDouble();

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
  final scale = math.pow(2, meta.extraZoomLevels).toDouble();

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

class _LocalMapProjection extends Projection {
  const _LocalMapProjection() : super(null);

  @override
  (double, double) projectXY(LatLng latlng) =>
      (latlng.longitude, latlng.latitude);

  @override
  LatLng unprojectXY(double x, double y) => LatLng(
        y.clamp(-90.0, 90.0),
        x.clamp(-180.0, 180.0),
      );
}

class LocalMapCrs extends Crs {
  // ignore: prefer_const_constructors_in_immutables
  LocalMapCrs._({
    required this.mapSize,
    required Projection projection,
    required List<double> scales,
  })  : _projection = projection,
        _scales = scales,
        super(code: 'LOCAL_MAP', infinite: true);

  final double mapSize;
  final Projection _projection;
  final List<double> _scales;

  (double, double) _transform(double x, double y, double s) {
    final a = mapSize / 360;
    final b = mapSize / 2;
    final c = mapSize / 180;
    final d = mapSize / 2;
    return (s * (a * x + b), s * (c * y + d));
  }

  (double, double) _untransform(double px, double py, double s) {
    final a = mapSize / 360;
    final b = mapSize / 2;
    final c = mapSize / 180;
    final d = mapSize / 2;
    return ((px / s - b) / a, (py / s - d) / c);
  }

  @override
  Projection get projection => _projection;

  @override
  (double, double) transform(double x, double y, double scale) =>
      _transform(x, y, scale);

  @override
  (double, double) untransform(double x, double y, double scale) =>
      _untransform(x, y, scale);

  @override
  (double, double) latLngToXY(LatLng latlng, double scale) {
    final (x, y) = _projection.projectXY(latlng);
    return _transform(x, y, scale);
  }

  @override
  LatLng offsetToLatLng(Offset point, double zoom) {
    final (x, y) = _untransform(point.dx, point.dy, scale(zoom));
    return _projection.unprojectXY(x, y);
  }

  @override
  Rect? getProjectedBounds(double zoom) => null;

  @override
  double scale(double zoom) {
    final iZoom = zoom.floor();
    if (zoom == iZoom) {
      return _scales[iZoom.clamp(0, _scales.length - 1)];
    }
    final baseScale = _scales[iZoom.clamp(0, _scales.length - 1)];
    final nextIdx = (iZoom + 1).clamp(0, _scales.length - 1);
    final nextScale = _scales[nextIdx];
    final scaleDiff = nextScale - baseScale;
    final zDiff = zoom - iZoom;
    return baseScale + scaleDiff * zDiff;
  }

  @override
  double zoom(double scale) {
    double? downScale;
    int downZoom = 0;
    for (var i = _scales.length - 1; i >= 0; i--) {
      final curr = _scales[i];
      if (curr <= scale && (downScale == null || downScale < curr)) {
        downScale = curr;
        downZoom = i;
      }
    }
    if (downScale == null) return double.negativeInfinity;
    if (scale == downScale) return downZoom.toDouble();
    final nextZoom = (downZoom + 1).clamp(0, _scales.length - 1);
    final nextScale = _scales[nextZoom];
    final scaleDiff = nextScale - downScale;
    return (scale - downScale) / scaleDiff + downZoom;
  }
}

Crs createLocalMapCrs(MapMeta meta) {
  final mapSize = 256.0 * math.pow(2, meta.maxZoom);
  const maxScaleZoom = 24;
  final scales = List.generate(
    maxScaleZoom + 1,
    (z) => 256.0 * math.pow(2, z) / mapSize,
  );
  return LocalMapCrs._(
    mapSize: mapSize,
    projection: const _LocalMapProjection(),
    scales: scales,
  );
}
