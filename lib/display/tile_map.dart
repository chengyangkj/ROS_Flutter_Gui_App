import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:vector_math/vector_math_64.dart' as vm;
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/tile_map_meta.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/costmap.dart';
import 'package:ros_flutter_gui_app/display/grid.dart' show WorldToLatLngFn, buildGridLayer;
import 'package:ros_flutter_gui_app/display/laser.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/path.dart';
import 'package:ros_flutter_gui_app/display/pointcloud.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/polygon.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/robot.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/display/topology_line.dart' hide WorldToLatLngFn;
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/provider/them_provider.dart';

enum ObstacleEditTool {
  None,
  Brush,
  Eraser,
}

class TileMap extends StatefulWidget {
  final Function(NavPoint?)? onNavPointTap;
  final ValueChanged<TopologyRoute>? onRouteTap;
  final TopologyRoute? selectedRoute;
  final VoidCallback? onTap;
  final void Function(double worldX, double worldY)? onTapWorld;
  final bool enableMapInteraction;
  final bool enableTopologyEdit;
  final ObstacleEditTool obstacleEditTool;
  final double obstacleBrushSizeMeters;
  final String? selectedNavPointName;
  final bool followRobot;

  const TileMap({
    super.key,
    this.onNavPointTap,
    this.onRouteTap,
    this.selectedRoute,
    this.onTap,
    this.onTapWorld,
    this.enableMapInteraction = true,
    this.enableTopologyEdit = false,
    this.obstacleEditTool = ObstacleEditTool.None,
    this.obstacleBrushSizeMeters = 0.25,
    this.selectedNavPointName,
    this.followRobot = false,
  });

  @override
  State<TileMap> createState() => TileMapState();
}

class TileMapState extends State<TileMap> {
  MapMeta? _meta;
  String? _error;
  final MapController _mapController = MapController();
  double _currentZoom = 2.0;
  bool _isDarkMode = true;
  RobotPose? _relocPose;
  final Map<String, NavPoint> _draggingNavPoints = {};
  final Map<int, int> _obstacleEdits = {};

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

    return ValueListenableBuilder<Mode>(
      valueListenable: context.read<GlobalState>().mode,
      builder: (context, mode, _) {
        final enableObstacleEdit = widget.obstacleEditTool != ObstacleEditTool.None;
        return Listener(
          behavior: HitTestBehavior.translucent,
          onPointerDown: (e) {
            if (!enableObstacleEdit) return;
            _paintObstacleAtLocal(e.localPosition);
          },
          onPointerMove: (e) {
            if (!enableObstacleEdit) return;
            _paintObstacleAtLocal(e.localPosition);
          },
          child: FlutterMap(
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
              interactionOptions: InteractionOptions(
                flags: !widget.enableMapInteraction
                    ? InteractiveFlag.none
                    : InteractiveFlag.all,
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
          ),
        );
      },
    );
  }

  void _handleTap(LatLng latLng) {
    if (_meta == null) return;
    final world = latLngToWorld(_meta!, latLng);
    final worldX = world.x;
    final worldY = world.y;
    widget.onTapWorld?.call(worldX, worldY);

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

  Map<int, int> getObstacleEdits() => Map.unmodifiable(_obstacleEdits);

  void clearObstacleEdits() {
    if (_obstacleEdits.isEmpty) return;
    setState(() {
      _obstacleEdits.clear();
    });
  }

  void _paintObstacleAtLocal(Offset localPosition) {
    final meta = _meta;
    if (meta == null) return;
    final tool = widget.obstacleEditTool;
    if (tool == ObstacleEditTool.None) return;
    final camera = _mapController.camera;

    final latLng = camera.unprojectAtZoom(localPosition + camera.pixelOrigin);
    final world = latLngToWorld(meta, latLng);
    _applyObstacleBrush(world.x, world.y);
  }

  void _applyObstacleBrush(double worldX, double worldY) {
    final meta = _meta;
    if (meta == null) return;

    final tool = widget.obstacleEditTool;
    if (tool == ObstacleEditTool.None) return;
    final value = tool == ObstacleEditTool.Brush ? 1 : -1;

    final res = meta.resolution;
    final radius = widget.obstacleBrushSizeMeters;
    final rCells = (radius / res).ceil();
    final centerCol = ((worldX - meta.originX) / res).floor();
    final centerRow = ((worldY - meta.originY) / res).floor();

    bool changed = false;
    for (var dy = -rCells; dy <= rCells; dy++) {
      for (var dx = -rCells; dx <= rCells; dx++) {
        final col = centerCol + dx;
        final row = centerRow + dy;
        if (col < 0 || col >= meta.width || row < 0 || row >= meta.height) continue;
        final cx = meta.originX + (col + 0.5) * res;
        final cy = meta.originY + (row + 0.5) * res;
        final ddx = cx - worldX;
        final ddy = cy - worldY;
        if (ddx * ddx + ddy * ddy > radius * radius) continue;

        final key = row * meta.width + col;
        final prev = _obstacleEdits[key] ?? 0;
        if (prev == value) continue;
        _obstacleEdits[key] = value;
        changed = true;
      }
    }
    if (changed && mounted) setState(() {});
  }

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
          layers.add(ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: rosChannel.globalPath,
            builder: (_, path, __) => buildPathLayer(
              path.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
              Colors.blue,
            ),
          ));
        }
        if (globalState.isLayerVisible('showLocalPath')) {
          layers.add(ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: rosChannel.localPath,
            builder: (_, path, __) => buildPathLayer(
              path.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
              Colors.green,
            ),
          ));
        }
        if (globalState.isLayerVisible('showTracePath')) {
          layers.add(ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: rosChannel.tracePath,
            builder: (_, path, __) => buildPathLayer(
              path.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
              Colors.yellow,
            ),
          ));
        }
        if (globalState.isLayerVisible('showLaser')) {
          layers.add(ValueListenableBuilder<Mode>(
            valueListenable: globalState.mode,
            builder: (_, mode, __) => ValueListenableBuilder(
              valueListenable: rosChannel.laserPointData,
              builder: (_, ___, ____) => buildLaserLayer(
                rosChannel,
                toLatLng,
                robotPoseOverride: mode == Mode.reloc ? _relocPose : null,
              ),
            ),
          ));
        }
        if (globalState.isLayerVisible('showPointCloud')) {
          layers.add(ValueListenableBuilder(
            valueListenable: rosChannel.pointCloud2Data,
            builder: (_, __, ___) => buildPointCloudLayer(rosChannel, toLatLng),
          ));
        }
        if (globalState.isLayerVisible('showRobotFootprint')) {
          layers.add(ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: rosChannel.robotFootprint,
            builder: (_, __, ___) => buildRobotFootprintLayer(rosChannel, toLatLng),
          ));
        }
        if (globalState.isLayerVisible('showTopology')) {
          layers.add(ValueListenableBuilder(
            valueListenable: rosChannel.topologyMap_,
            builder: (_, topologyMap, ___) {
              final rawNavPoints = topologyMap.points.isNotEmpty
                  ? topologyMap.points
                  : rosChannel.mapManager.navPoints;
              final navPoints = rawNavPoints
                  .map((p) => _draggingNavPoints[p.name] ?? p)
                  .toList();
              return ValueListenableBuilder<Mode>(
                valueListenable: globalState.mode,
                builder: (_, mode, __) {
                  final isEdit = mode == Mode.mapEdit && widget.enableTopologyEdit;
                  return buildTopologyLineLayer(
                    navPoints,
                    topologyMap.routes,
                    toLatLng,
                    onNavPointTap: widget.onNavPointTap,
                    onRouteTap: widget.onRouteTap,
                    isEditMode: isEdit,
                    selectedNavPointName: widget.selectedNavPointName,
                    selectedRoute: widget.selectedRoute,
                    onNavPointChanged: isEdit
                        ? (updated) {
                            rosChannel.mapManager.updateNavPoint(updated.name, updated);
                            _draggingNavPoints.remove(updated.name);
                            setState(() {});
                          }
                        : null,
                    onNavPointMoveDelta: isEdit
                        ? (point, delta) {
                            final meta = _meta;
                            if (meta == null) return;
                            final camera = _mapController.camera;
                            final base = _draggingNavPoints[point.name] ?? point;
                            final currentLatLng = toLatLng(base.x, base.y);
                            final currentOffset =
                                camera.getOffsetFromOrigin(currentLatLng);
                            final newLatLng = camera.unprojectAtZoom(
                              currentOffset + delta + camera.pixelOrigin,
                            );
                            final world = latLngToWorld(meta, newLatLng);
                            final newX = world.x;
                            final newY = world.y;
                            final dragging = NavPoint(
                              name: point.name,
                              x: newX,
                              y: newY,
                              theta: base.theta,
                              type: base.type,
                            );
                            _draggingNavPoints[point.name] = dragging;
                            widget.onNavPointTap?.call(dragging);
                            setState(() {});
                          }
                        : null,
                  );
                },
              );
            },
          ));
        }

        if (_obstacleEdits.isNotEmpty) {
          layers.add(_buildObstacleEditLayer(meta, toLatLng));
        }
        if (globalState.isLayerVisible('showLocalCostmap')) {
          layers.add(ValueListenableBuilder(
            valueListenable: rosChannel.localCostmap,
            builder: (_, cm, ___) =>
                buildLocalCostMapOverlayLayer(cm, 0.5),
          ));
        }

        layers.add(ValueListenableBuilder(
          valueListenable: rosChannel.robotPoseMap,
          builder: (_, __, ___) => ValueListenableBuilder(
            valueListenable: globalState.mode,
            builder: (_, mode, ___) {
              final isReloc = mode == Mode.reloc;
              if (isReloc) {
                _relocPose ??= rosChannel.robotPoseMap.value;
              } else {
                _relocPose = null;
              }
              return buildRobotMarkerLayer(
                rosChannel,
                toLatLng,
                poseOverride: isReloc ? _relocPose : null,
                isEditMode: isReloc,
                onThetaChanged: isReloc
                    ? (theta) {
                        final current = _relocPose ?? rosChannel.robotPoseMap.value;
                        _relocPose = RobotPose(current.x, current.y, theta);
                        print('TileMapState _relocPose=$_relocPose');
                        setState(() {});
                      }
                    : null,
                onMoveDelta: isReloc
                    ? (delta) {
                        final meta = _meta;
                        if (meta == null) return;
                        final current = _relocPose ?? rosChannel.robotPoseMap.value;
                        final camera = _mapController.camera;
                        final currentLatLng = toLatLng(current.x, current.y);
                        final currentPx = camera.projectAtZoom(currentLatLng);
                        final newLatLng = camera.unprojectAtZoom(currentPx + delta);
                        final world = latLngToWorld(meta, newLatLng);
                        _relocPose = RobotPose(world.x, world.y, current.theta);
                        setState(() {});
                      }
                    : null,
              );
            },
          ),
        ));

        return Stack(children: layers);
      },
    );
  }

  Widget _buildObstacleEditLayer(MapMeta meta, WorldToLatLngFn toLatLng) {
    final camera = _mapController.camera;
    final res = meta.resolution;
    final px0 = camera.getOffsetFromOrigin(toLatLng(meta.originX, meta.originY));
    final px1 = camera.getOffsetFromOrigin(toLatLng(meta.originX + res, meta.originY));
    final cellSizePx = (px1 - px0).distance.abs().clamp(2.0, 80.0);

    final markers = <Marker>[];
    _obstacleEdits.forEach((key, value) {
      if (value == 0) return;
      final col = key % meta.width;
      final row = key ~/ meta.width;
      final cx = meta.originX + col * res;
      final cy = meta.originY + row * res;
      final color = value > 0
          ? const Color(0xFF000000)
          : const Color(0xFFFFFFFF);
      markers.add(Marker(
        point: toLatLng(cx, cy),
        width: cellSizePx,
        height: cellSizePx,
        alignment: Alignment.center,
        child: IgnorePointer(
          child: Container(
            decoration: BoxDecoration(
              color: color,
            ),
          ),
        ),
      ));
    });

    if (markers.isEmpty) return const SizedBox.shrink();
    return MarkerLayer(markers: markers);
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


  RobotPose getRelocRobotPose() {
    return _relocPose ?? context.read<RosChannel>().robotPoseMap.value;
  }

  void flushDraggingNavPoints() {
    if (_draggingNavPoints.isEmpty) return;
    final rosChannel = context.read<RosChannel>();
    final mapManager = rosChannel.mapManager;
    _draggingNavPoints.forEach((name, navPoint) {
      mapManager.updateNavPoint(name, navPoint);
    });
    _draggingNavPoints.clear();
  }
}

