import 'dart:convert';
import 'dart:math' show Point, pow;

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:http/http.dart' as http;
import 'package:latlong2/latlong.dart';
import 'package:proj4dart/proj4dart.dart' as proj4;
import 'package:url_launcher/url_launcher.dart';

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

double GetMapSize(MapMeta meta) => 256.0 * pow(2, meta.maxZoom);


//机器人的世界坐标转换为经纬度
LatLng WorldToLatLng(MapMeta meta, double worldX, double worldY) {
  final mapSize = GetMapSize(meta);
  final scale = pow(2, meta.extraZoomLevels).toDouble();

  //计算图元坐标
  double px = (worldX - meta.originX) / meta.resolution;
  double py = meta.height - (worldY - meta.originY) / meta.resolution;
  px*=scale;
  py*=scale;
  print('worldX: $worldX, worldY: $worldY,meta.originX: ${meta.originX}, meta.originY: ${meta.originY}, resolution: ${meta.resolution}, px: $px, py: $py mapSize: $mapSize');
  return LatLng(
    py * 180 / mapSize - 90,
    px * 360 / mapSize - 180,
  );
}

Crs createLocalMapCrs(MapMeta meta) {
  final mapSize = 256.0 * pow(2, meta.maxZoom);
  final bounds = Bounds<double>(
    const Point(-180.0, -90.0),
    const Point(180.0, 90.0),
  );
  final scales = List.generate(
    meta.maxZoom + 1,
    (z) => 256.0 * pow(2, z) / mapSize,
  );
  final transformation = Transformation(
    mapSize / 360,
    mapSize / 2,
    mapSize / 180,
    mapSize / 2,
  );
  return Proj4Crs.fromFactory(
    code: 'LOCAL_MAP',
    proj4Projection: proj4.Projection.add(
      'LOCAL_MAP',
      '+proj=longlat +datum=WGS84 +no_defs',
    ),
    transformation: transformation,
    bounds: bounds,
    scales: scales,
  );
}

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Tiles Map Demo',
      theme: ThemeData(colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple)),
      home: const MyHomePage(title: 'Tiles Map'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});
  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  static const String kTileServerUrl = 'http://192.168.35.105:7684';

  MapMeta? _meta;
  String? _error;

  @override
  void initState() {
    super.initState();
    _loadMeta();
  }

  Future<void> _loadMeta() async {
    try {
      final meta = await MapMeta.fetch(kTileServerUrl);
      setState(() {
        _meta = meta;
        _error = null;
      });
    } catch (e) {
      setState(() {
        _meta = null;
        _error = e.toString();
      });
    }
  }

  Future<void> _showZoomConfig(BuildContext context) async {
    if (_meta == null) return;
    int value = _meta!.extraZoomLevels;
    final result = await showDialog<int>(
      context: context,
      builder: (ctx) => StatefulBuilder(
        builder: (ctx, setDialogState) => AlertDialog(
          title: const Text('放大级别'),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text('extra_zoom_levels: $value (max_zoom 约: ${_meta!.maxZoom - _meta!.extraZoomLevels + value})'),
              Slider(
                value: value.toDouble(),
                min: 0,
                max: 8,
                divisions: 8,
                label: '$value',
                onChanged: (v) => setDialogState(() => value = v.round()),
              ),
            ],
          ),
          actions: [
            TextButton(onPressed: () => Navigator.pop(ctx), child: const Text('取消')),
            FilledButton(
              onPressed: () => Navigator.pop(ctx, value),
              child: const Text('应用'),
            ),
          ],
        ),
      ),
    );
    if (result != null && result != _meta!.extraZoomLevels) {
      final ok = await setExtraZoomLevels(kTileServerUrl, result);
      if (ok && context.mounted) {
        await _loadMeta();
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    if (_error != null) {
      return Scaffold(
        appBar: AppBar(title: Text(widget.title)),
        body: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Text(_error!, textAlign: TextAlign.center),
              const SizedBox(height: 16),
              ElevatedButton(onPressed: _loadMeta, child: const Text('Retry')),
            ],
          ),
        ),
      );
    }
    if (_meta == null) {
      return Scaffold(
        appBar: AppBar(title: Text(widget.title)),
        body: const Center(child: CircularProgressIndicator()),
      );
    }

    final meta = _meta!;
    final crs = createLocalMapCrs(meta);

    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
        actions: [
          IconButton(
            icon: const Icon(Icons.settings),
            onPressed: () => _showZoomConfig(context),
          ),
        ],
      ),
      body: SizedBox.expand(
        child: FlutterMap(
          options: MapOptions(
            crs: crs,
            initialCenter: const LatLng(0, 0),
            initialZoom: 2.0.clamp(0.0, meta.maxZoom.toDouble()),
            minZoom: 0,
            maxZoom: meta.maxZoom.toDouble(),
            cameraConstraint: CameraConstraint.unconstrained(),
          ),
        children: [
          TileLayer(
            urlTemplate: '$kTileServerUrl/tiles/{z}/{x}/{y}.png',
            userAgentPackageName: 'flutter_map_test',
            tileBounds: LatLngBounds(
              const LatLng(-90, -180),
              const LatLng(90, 180),
            ),
            tileBuilder: (context, child, tileImage) {
              if (tileImage.imageInfo?.image != null && !tileImage.loadError) {
                return RawImage(
                    image: tileImage.imageInfo!.image,
                    fit: BoxFit.fill,
                    filterQuality: FilterQuality.none,
                    opacity: tileImage.opacity == 1
                        ? null
                        : AlwaysStoppedAnimation(tileImage.opacity),
                  
                );
              }
              return child;
            },
          ),
          MarkerLayer(
            markers: [
              Marker(
                point: WorldToLatLng(meta, 0,0),
                width: 32,
                height: 32,
                alignment: Alignment.center,
                child: const Icon(Icons.person_pin_circle, color: Colors.blue, size: 32),
              ),
            ],
          ),
          RichAttributionWidget(
            attributions: [
              TextSourceAttribution(
                'Local map tiles',
                onTap: () => launchUrl(Uri.parse('https://openstreetmap.org/copyright')),
              ),
            ],
          ),
        ],
        ),
      ),
    );
  }
}
