import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/display/costmap.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'package:ros_flutter_gui_app/display/laser.dart';
import 'package:ros_flutter_gui_app/display/path.dart';
import 'package:ros_flutter_gui_app/display/pointcloud.dart';
import 'package:ros_flutter_gui_app/display/polygon.dart';
import 'package:ros_flutter_gui_app/display/robot_marker.dart';
import 'package:ros_flutter_gui_app/display/topology_line.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/map_tile_provider.dart';
import 'package:ros_flutter_gui_app/provider/map_manager.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:vector_math/vector_math_64.dart' as vm;

class MainMapWidget extends StatefulWidget {
  const MainMapWidget({
    super.key,
    required this.rosChannel,
    required this.globalState,
    required this.mapManager,
    required this.tileServerPort,
    required this.onNavPointTap,
  });

  final RosChannel rosChannel;
  final GlobalState globalState;
  final MapManager mapManager;
  final int tileServerPort;
  final void Function(NavPoint?)? onNavPointTap;

  @override
  State<MainMapWidget> createState() => MainMapWidgetState();
}

class MainMapWidgetState extends State<MainMapWidget> {
  final MapController _mapController = MapController();

  MapController get mapController => _mapController;

  @override
  void initState() {
    super.initState();
    _loadMeta();
    widget.rosChannel.topologyMap_.addListener(_onTopologyOrMapChanged);
    widget.rosChannel.map_.addListener(_onTopologyOrMapChanged);
  }

  void _onTopologyOrMapChanged() {
    if (mounted) setState(() {});
  }

  Future<void> _loadMeta() async {
    final baseUrl = 'http://${globalSetting.robotIp}:${widget.tileServerPort}';
    await Provider.of<MapTileProvider>(context, listen: false).loadMeta(baseUrl);
  }

  String get _tileBaseUrl {
    return 'http://${globalSetting.robotIp}:${widget.tileServerPort}';
  }

  void zoomIn() {
    final meta = Provider.of<MapTileProvider>(context, listen: false).meta;
    if (meta != null && _mapController.camera.zoom < meta.maxZoom) {
      _mapController.move(
        _mapController.camera.center,
        (_mapController.camera.zoom + 1).clamp(0.0, meta.maxZoom.toDouble()),
      );
    }
  }

  void zoomOut() {
    final meta = Provider.of<MapTileProvider>(context, listen: false).meta;
    if (meta != null && _mapController.camera.zoom > 0) {
      _mapController.move(
        _mapController.camera.center,
        (_mapController.camera.zoom - 1).clamp(0.0, meta.maxZoom.toDouble()),
      );
    }
  }

  void centerOnRobot(bool follow) {
    final mapTile = Provider.of<MapTileProvider>(context, listen: false);
    if (!mapTile.hasMeta) return;
    final pose = widget.rosChannel.robotPoseMap.value;
    final center = mapTile.worldToLatLng(pose.x, pose.y);
    _mapController.move(center, 6.0);
  }

  void hideInfoPanel() {}

  bool get showInfoPanel => false;

  Future<void> reloadNavPointsAndMap() async {
    await _loadMeta();
  }

  void showRelocDialog() {
    final pose = widget.rosChannel.robotPoseMap.value;
    final xCtrl = TextEditingController(text: pose.x.toStringAsFixed(2));
    final yCtrl = TextEditingController(text: pose.y.toStringAsFixed(2));
    final thetaCtrl = TextEditingController(text: pose.theta.toStringAsFixed(4));
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('重定位'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              decoration: const InputDecoration(labelText: 'X (m)'),
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              controller: xCtrl,
            ),
            TextField(
              decoration: const InputDecoration(labelText: 'Y (m)'),
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              controller: yCtrl,
            ),
            TextField(
              decoration: const InputDecoration(labelText: '方向 θ (弧度)'),
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              controller: thetaCtrl,
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('取消'),
          ),
          FilledButton(
            onPressed: () {
              final x = double.tryParse(xCtrl.text) ?? pose.x;
              final y = double.tryParse(yCtrl.text) ?? pose.y;
              final theta = double.tryParse(thetaCtrl.text) ?? pose.theta;
              widget.rosChannel.sendRelocPose(RobotPose(x, y, theta));
              Navigator.pop(ctx);
            },
            child: const Text('确认'),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    widget.rosChannel.topologyMap_.removeListener(_onTopologyOrMapChanged);
    widget.rosChannel.map_.removeListener(_onTopologyOrMapChanged);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<MapTileProvider>(
      builder: (context, mapTile, _) {
        if (mapTile.error != null) {
          return Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(mapTile.error ?? '', textAlign: TextAlign.center),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: _loadMeta,
                  child: const Text('重试'),
                ),
              ],
            ),
          );
        }
        if (mapTile.loading || !mapTile.hasMeta) {
          return const Center(child: CircularProgressIndicator());
        }
        return _buildMap(mapTile);
      },
    );
  }

  Widget _buildMap(MapTileProvider mapTile) {
    final meta = mapTile.meta!;
    final crs = mapTile.createLocalMapCrs();

    return SizedBox.expand(
      child: FlutterMap(
      mapController: _mapController,
      options: MapOptions(
        crs: crs,
        initialCenter: const LatLng(0, 0),
        initialZoom: 1.0.clamp(0.0, meta.maxZoom.toDouble()),
        minZoom: 0,
        maxZoom: meta.maxZoom.toDouble(),
        cameraConstraint: CameraConstraint.unconstrained()
      ),
      children: [
        TileLayer(
          urlTemplate: '$_tileBaseUrl/tiles/{z}/{x}/{y}.png',
          userAgentPackageName: 'ros_flutter_gui_app',
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
        if (widget.globalState.isLayerVisible('showGrid'))
          buildGridLayer(
            resolution: meta.resolution,
            originX: meta.originX,
            originY: meta.originY,
            width: meta.width,
            height: meta.height,
            worldToLatLng: mapTile.worldToLatLng,
            isDark: Theme.of(context).brightness == Brightness.dark,
          ),
        if (widget.globalState.isLayerVisible('showGlobalPath'))
          ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: widget.rosChannel.globalPath,
            builder: (context, points, _) => _pathLayer(mapTile, points, Colors.blue),
          ),
        if (widget.globalState.isLayerVisible('showLocalPath'))
          ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: widget.rosChannel.localPath,
            builder: (context, points, _) => _pathLayer(mapTile, points, Colors.green),
          ),
        if (widget.globalState.isLayerVisible('showTracePath'))
          ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: widget.rosChannel.tracePath,
            builder: (context, points, _) => _pathLayer(mapTile, points, Colors.yellow),
          ),
        if (widget.globalState.isLayerVisible('showLaser'))
          buildLaserLayer(widget.rosChannel, mapTile.worldToLatLng),
        if (widget.globalState.isLayerVisible('showPointCloud'))
          buildPointCloudLayer(widget.rosChannel, mapTile.worldToLatLng),
        if (widget.globalState.isLayerVisible('showRobotFootprint'))
          buildRobotFootprintLayer(widget.rosChannel, mapTile.worldToLatLng),
        if (widget.globalState.isLayerVisible('showTopology'))
          ValueListenableBuilder(
            valueListenable: widget.mapManager.topologyMap,
            builder: (context, topologyMap, _) => buildTopologyLineLayer(
              topologyMap.points,
              topologyMap.routes,
              mapTile.worldToLatLng,
              onNavPointTap: widget.onNavPointTap,
            ),
          ),
        if (widget.globalState.isLayerVisible('showLocalCostmap'))
          ValueListenableBuilder<OccupancyMap>(
            valueListenable: widget.rosChannel.localCostmap,
            builder: (context, cm, _) => buildLocalCostMapOverlayLayer(cm, 0.5),
          ),
        buildRobotMarkerLayer(widget.rosChannel, mapTile.worldToLatLng),
      ],
      ),
    );
  }

  Widget _pathLayer(MapTileProvider mapTile, List<vm.Vector2> points, Color color) {
    if (points.isEmpty) return const SizedBox.shrink();
    final latLngs = points.map((p) => mapTile.worldToLatLng(p.x, p.y)).toList();
    return buildPathLayer(latLngs, color, strokeWidth: 3);
  }
}
