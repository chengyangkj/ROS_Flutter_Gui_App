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
  });

  final double resolution;
  final double originX;
  final double originY;
  final int width;
  final int height;
  final int maxZoom;

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
    );
  }
}

Crs createLocalMapCrs(MapMeta meta) {
  final mapSize = 256.0 * pow(2, meta.maxZoom);
  final bounds = Bounds<double>(
    const Point(0.0, 0.0),
    Point(mapSize, mapSize),
  );
  final scales = List.generate(
    meta.maxZoom + 1,
    (z) => 256.0 * pow(2, z) / mapSize,
  );
  final transformation = const Transformation(1, 0, 1, 0);
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
  static const String kTileServerUrl = 'http://192.168.35.105:8080';

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
    final mapSize = 256.0 * pow(2, meta.maxZoom);
    final center = mapSize / 2;

    return Scaffold(
      appBar: AppBar(title: Text(widget.title)),
      body: FlutterMap(
        options: MapOptions(
          crs: crs,
          initialCenter: LatLng(center, center),
          initialZoom: 2.0.clamp(0.0, meta.maxZoom.toDouble()),
          minZoom: 0,
          maxZoom: meta.maxZoom.toDouble(),
        ),
        children: [
          TileLayer(
            urlTemplate: '$kTileServerUrl/tiles/{z}/{x}/{y}.png',
            userAgentPackageName: 'flutter_map_test',
            tileBounds: LatLngBounds(
              LatLng(0, 0),
              LatLng(mapSize, mapSize),
            ),
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
    );
  }
}
