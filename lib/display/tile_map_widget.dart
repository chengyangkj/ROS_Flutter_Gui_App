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
    final sw = worldToLatLng(
      meta,
      meta.originX,
      meta.originY + meta.height * meta.resolution,
    );
    final ne = worldToLatLng(
      meta,
      meta.originX + meta.width * meta.resolution,
      meta.originY,
    );
    final mapBounds = LatLngBounds(sw, ne);

    return FlutterMap(
      mapController: _mapController,
      options: MapOptions(
        crs: crs,
        initialCameraFit: CameraFit.bounds(
          bounds: mapBounds,
          padding: const EdgeInsets.all(24),
          maxZoom: meta.maxZoom.toDouble(),
        ),
        minZoom: 0,
        maxZoom: meta.maxZoom.toDouble(),
        cameraConstraint: const CameraConstraint.unconstrained(),
        onMapEvent: (event) {
          if (event is MapEventWithMove) {
            _currentZoom = event.camera.zoom;
          }
        },
        onTap: (tapPosition, latLng) {
          widget.onTap?.call();
          _handleTap(latLng);
        },
        interactionOptions: const InteractionOptions(flags: InteractiveFlag.all),
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
          layers.add(buildPathLayer(
            rosChannel.globalPath.value.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
            Colors.blue,
          ));
        }
        if (globalState.isLayerVisible('showLocalPath')) {
          layers.add(buildPathLayer(
            rosChannel.localPath.value.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
            Colors.green,
          ));
        }
        if (globalState.isLayerVisible('showTracePath')) {
          layers.add(buildPathLayer(
            rosChannel.tracePath.value.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
            Colors.yellow,
          ));
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

