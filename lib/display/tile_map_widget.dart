import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/tile_map_meta.dart';
import 'package:ros_flutter_gui_app/display/costmap.dart';
import 'package:ros_flutter_gui_app/display/grid.dart' show WorldToLatLngFn, buildGridLayer;
import 'package:ros_flutter_gui_app/display/laser.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/path.dart';
import 'package:ros_flutter_gui_app/display/pointcloud.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/polygon.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/robot_marker.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/topology_line.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/provider/them_provider.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

class TileMapWidget extends StatefulWidget {
  final Function(NavPoint?)? onNavPointTap;
  final VoidCallback? onTap;
  final bool followRobot;

  const TileMapWidget({
    super.key,
    this.onNavPointTap,
    this.onTap,
    this.followRobot = false,
  });

  @override
  State<TileMapWidget> createState() => TileMapWidgetState();
}

class TileMapWidgetState extends State<TileMapWidget> {
  MapMeta? _meta;
  String? _error;
  final MapController _mapController = MapController();
  double _currentZoom = 2.0;
  bool _isDarkMode = true;

  @override
  void initState() {
    super.initState();
    loadMeta();
  }

  Future<void> loadMeta() async {
    try {
      final meta = await MapMeta.fetch(globalSetting.tileServerUrl);
      if (mounted) {
        setState(() {
          _meta = meta;
          _error = null;
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _meta = null;
          _error = e.toString();
        });
      }
    }
  }

  List<LatLng> _pathToLatLngs(List<vm.Vector2> path, MapMeta meta) {
    final occMap = context.read<RosChannel>().map_.value;
    final List<LatLng> result = [];
    for (final p in path) {
      vm.Vector2 world;
      if (occMap.mapConfig.resolution > 0 && occMap.width() > 0) {
        world = occMap.idx2xy(vm.Vector2(p.x, p.y));
      } else {
        world = meta.idx2xy(vm.Vector2(p.x, p.y));
      }
      result.add(worldToLatLng(meta, world.x, world.y));
    }
    return result;
  }

  @override
  Widget build(BuildContext context) {
    final themeProvider = Provider.of<ThemeProvider>(context, listen: true);
    _isDarkMode = themeProvider.themeMode == ThemeMode.dark;

    if (_error != null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(_error!, textAlign: TextAlign.center),
            const SizedBox(height: 16),
            ElevatedButton(onPressed: loadMeta, child: const Text('重试')),
          ],
        ),
      );
    }
    if (_meta == null) {
      return const Center(child: CircularProgressIndicator());
    }

    final meta = _meta!;
    final crs = createLocalMapCrs(meta);

    return FlutterMap(
      mapController: _mapController,
      options: MapOptions(
        crs: crs,
        initialCenter: const LatLng(0, 0),
        initialZoom: _currentZoom.clamp(0.0, meta.maxZoom.toDouble()),
        minZoom: 0,
        maxZoom: meta.maxZoom.toDouble(),
        cameraConstraint: CameraConstraint.unconstrained(),
        onMapEvent: (event) {
          if (event is MapEventWithMove) {
            _currentZoom = event.camera.zoom;
          }
        },
        onTap: (tapPosition, latLng) {
          widget.onTap?.call();
          _handleTap(latLng);
        },
        interactionOptions: InteractionOptions(
          flags: InteractiveFlag.all,
        ),
      ),
      children: [
        TileLayer(
          urlTemplate: '${globalSetting.tileServerUrl}/tiles/{z}/{x}/{y}.png',
          userAgentPackageName: 'ros_flutter_gui_app',
          tileBounds: getTileBounds(),
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
        _buildOverlayLayers(meta),
      ],
    );
  }

  void _handleTap(LatLng latLng) {
    if (_meta == null) return;
    final world = latLngToWorld(_meta!, latLng);
    final worldX = world.x;
    final worldY = world.y;

    final rosChannel = context.read<RosChannel>();
    final navPoints = rosChannel.topologyMap_.value.points.isNotEmpty
        ? rosChannel.topologyMap_.value.points
        : rosChannel.mapManager.navPoints;
    const hitRadius = 0.5;
    for (final p in navPoints) {
      if ((p.x - worldX).abs() < hitRadius && (p.y - worldY).abs() < hitRadius) {
        widget.onNavPointTap?.call(p);
        return;
      }
    }
    widget.onNavPointTap?.call(null);
  }

  WorldToLatLngFn _worldToLatLng(MapMeta meta) => (x, y) => worldToLatLng(meta, x, y);

  Widget _buildOverlayLayers(MapMeta meta) {
    final toLatLng = _worldToLatLng(meta);
    return Consumer2<RosChannel, GlobalState>(
      builder: (context, rosChannel, globalState, _) {
        final layers = <Widget>[];

        if (globalState.isLayerVisible('showGrid')) {
          layers.add(buildGridLayer(
            resolution: meta.resolution,
            originX: meta.originX,
            originY: meta.originY,
            width: meta.width,
            height: meta.height,
            worldToLatLng: toLatLng,
            isDark: _isDarkMode,
          ));
        }
        if (globalState.isLayerVisible('showGlobalPath')) {
          layers.add(buildPathLayer(_pathToLatLngs(rosChannel.globalPath.value, meta), Colors.blue));
        }
        if (globalState.isLayerVisible('showLocalPath')) {
          layers.add(buildPathLayer(_pathToLatLngs(rosChannel.localPath.value, meta), Colors.green));
        }
        if (globalState.isLayerVisible('showTracePath')) {
          layers.add(buildPathLayer(_pathToLatLngs(rosChannel.tracePath.value, meta), Colors.yellow));
        }
        if (globalState.isLayerVisible('showLaser')) {
          layers.add(buildLaserLayer(rosChannel, toLatLng));
        }
        if (globalState.isLayerVisible('showPointCloud')) {
          layers.add(buildPointCloudLayer(rosChannel, toLatLng));
        }
        if (globalState.isLayerVisible('showRobotFootprint')) {
          layers.add(buildRobotFootprintLayer(rosChannel, toLatLng));
        }
        if (globalState.isLayerVisible('showTopology')) {
          final topologyMap = rosChannel.topologyMap_.value;
          final navPoints = topologyMap.points.isNotEmpty
              ? topologyMap.points
              : rosChannel.mapManager.navPoints;
          layers.add(buildTopologyLineLayer(
            navPoints,
            topologyMap.routes,
            toLatLng,
            onNavPointTap: widget.onNavPointTap,
          ));
        }
        if (globalState.isLayerVisible('showLocalCostmap')) {
          layers.add(buildLocalCostMapOverlayLayer(
            rosChannel.localCostmap.value,
            0.5,
          ));
        }

        layers.add(buildRobotMarkerLayer(rosChannel, toLatLng));

        return Stack(children: layers);
      },
    );
  }

  void moveToRobot() {
    final rosChannel = context.read<RosChannel>();
    final pose = rosChannel.robotPoseMap.value;
    if (_meta != null) {
      _mapController.move(
        worldToLatLng(_meta!, pose.x, pose.y),
        6.0,
      );
    }
  }

  void zoomIn() {
    _mapController.move(_mapController.camera.center, _currentZoom + 0.5);
  }

  void zoomOut() {
    _mapController.move(_mapController.camera.center, _currentZoom - 0.5);
  }

  void setRelocMode(bool enabled) {}

  RobotPose getRelocRobotPose() {
    return context.read<RosChannel>().robotPoseMap.value;
  }
}

