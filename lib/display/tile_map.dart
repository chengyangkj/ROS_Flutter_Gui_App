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
import 'package:ros_flutter_gui_app/provider/http_channel.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';

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
  final bool editMode;
  final ObstacleEditTool obstacleEditTool;
  final double obstacleBrushSizeMeters;
  final void Function(NavPoint oldPoint, NavPoint newPoint)? onNavPointEditEnd;
  final void Function(Map<int, int> oldEdits, Map<int, int> newEdits)? onObstacleEditEnd;
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
    this.editMode = false,
    this.obstacleEditTool = ObstacleEditTool.None,
    this.obstacleBrushSizeMeters = 0.25,
    this.onNavPointEditEnd,
    this.onObstacleEditEnd,
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
  String _currentMapName = '';
  double _currentZoom = 2.0;
  RobotPose? _relocPose;
  final Map<String, NavPoint> _draggingNavPoints = {};
  final Map<String, NavPoint> _draggingNavPointsStart = {};
  final Map<int, int> _obstacleEdits = {};
  final Map<int, int> _obstacleStrokeOldEdits = {};
  final Map<int, int> _obstacleStrokeNewEdits = {};
  Offset? _lastObstaclePaintLocal;

  @override
  void initState() {
    super.initState();
    loadMeta();
  }

  Future<void> loadMeta() async {
    try {
      final httpChannel = context.read<HttpChannel>();
      final mapName = await httpChannel.getCurrentMap();
      final meta = await MapMeta.fetch(globalSetting.tileServerUrl);
      if (mounted) {
        setState(() {
          _meta = meta;
          _error = null;
          _currentMapName = mapName;
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
    if (_error != null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(_error!, textAlign: TextAlign.center),
            const SizedBox(height: 16),
            ElevatedButton(onPressed: loadMeta, child: Text(AppLocalizations.of(context)!.retry)),
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
            _obstacleStrokeOldEdits.clear();
            _obstacleStrokeNewEdits.clear();
            _lastObstaclePaintLocal = e.localPosition;
            _paintObstacleAtLocal(e.localPosition);
          },
          onPointerMove: (e) {
            if (!enableObstacleEdit) return;
            final prev = _lastObstaclePaintLocal;
            _lastObstaclePaintLocal = e.localPosition;
            if (prev != null) {
              const stepPx = 6.0;
              final d = (e.localPosition - prev).distance;
              final steps = (d / stepPx).ceil().clamp(1, 64);
              for (var i = 1; i <= steps; i++) {
                final t = i / steps;
                _paintObstacleAtLocal(Offset(
                  prev.dx + (e.localPosition.dx - prev.dx) * t,
                  prev.dy + (e.localPosition.dy - prev.dy) * t,
                ));
              }
            } else {
              _paintObstacleAtLocal(e.localPosition);
            }
          },
          onPointerUp: (_) {
            if (!enableObstacleEdit) return;
            _lastObstaclePaintLocal = null;
            if (widget.onObstacleEditEnd != null &&
                _obstacleStrokeNewEdits.isNotEmpty) {
              widget.onObstacleEditEnd!(
                Map<int, int>.from(_obstacleStrokeOldEdits),
                Map<int, int>.from(_obstacleStrokeNewEdits),
              );
            }
            _obstacleStrokeOldEdits.clear();
            _obstacleStrokeNewEdits.clear();
          },
          onPointerCancel: (_) {
            _lastObstaclePaintLocal = null;
            _obstacleStrokeOldEdits.clear();
            _obstacleStrokeNewEdits.clear();
          },
          child: FlutterMap(
            mapController: _mapController,
            options: MapOptions(
              backgroundColor: const Color.fromRGBO(205, 205, 205, 1),
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
                urlTemplate: _currentMapName.isNotEmpty
                    ? '${globalSetting.tileServerUrl}/tiles/{map_name}/{z}/{x}/{y}.png'
                    : '${globalSetting.tileServerUrl}/tiles/{z}/{x}/{y}.png',
                additionalOptions: {'map_name': _currentMapName},
                userAgentPackageName: 'ros_flutter_gui_app',
                tileBounds: getTileBounds(),
                tileProvider: NetworkTileProvider(
                  cachingProvider: const DisabledMapCachingProvider(),
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

  void applyObstacleEdits(Map<int, int> edits) {
    if (edits.isEmpty) return;
    setState(() {
      for (final entry in edits.entries) {
        final key = entry.key;
        final value = entry.value;
        // value 0 表示清除；100 表示障碍物。
        // 为了让“原图障碍”也能被覆盖，这里要显式存储 0。
        if (value == 0 || value > 0) {
          _obstacleEdits[key] = value;
        }
      }
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
    // 障碍物编辑值语义：
    // 100: 有障碍物（黑色单元）
    // 0: 无障碍物（白色单元，表示清除）
    // 100: 有障碍（黑色）；0: 无障碍（白色，覆盖原障碍）
    final value = tool == ObstacleEditTool.Brush ? 100 : 0;

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
        final hasPrev = _obstacleEdits.containsKey(key);
        final prev = hasPrev ? (_obstacleEdits[key] as int) : 0;
        if (hasPrev && prev == value) continue;

        _obstacleStrokeOldEdits[key] ??= prev;
        _obstacleEdits[key] = value;
        _obstacleStrokeNewEdits[key] = value;
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

       
        if (!widget.editMode && globalState.isLayerVisible('showGlobalPath')) {
          layers.add(ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: rosChannel.globalPath,
            builder: (_, path, __) => buildPathLayer(
              path.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
              Colors.blue,
            ),
          ));
        }

        if (!widget.editMode && globalState.isLayerVisible('showLocalPath')) {
          layers.add(ValueListenableBuilder<List<vm.Vector2>>(
            valueListenable: rosChannel.localPath,
            builder: (_, path, __) => buildPathLayer(
              path.map((p) => worldToLatLng(meta, p.x, p.y)).toList(),
              Colors.green,
            ),
          ));
        }

        if (!widget.editMode &&  globalState.isLayerVisible('showTracePath')) {
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
        if (!widget.editMode &&
            globalState.isLayerVisible('showRobotFootprint')) {
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
          
                  return buildTopologyLineLayer(
                    navPoints,
                    topologyMap.routes,
                    toLatLng,
                    onNavPointTap: widget.onNavPointTap,
                    onRouteTap: widget.onRouteTap,
                    isEditMode: widget.editMode ,
                    selectedNavPointName: widget.selectedNavPointName,
                    selectedRoute: widget.selectedRoute,
                    onNavPointChanged: widget.editMode 
                        ? (updated) {
                            final base = _draggingNavPoints[updated.name] ?? updated;
                            final dragging = NavPoint(
                              name: updated.name,
                              x: base.x,
                              y: base.y,
                              theta: updated.theta,
                              type: base.type,
                            );
                            _draggingNavPoints[updated.name] = dragging;
                            _draggingNavPointsStart[updated.name] ??=
                                rosChannel.mapManager.getNavPoint(updated.name) ?? updated;
                            setState(() {});
                          }
                        : null,
                    onNavPointMoveDelta: widget.editMode 
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
                            _draggingNavPointsStart[point.name] ??= point;
                            widget.onNavPointTap?.call(dragging);
                            setState(() {});
                          }
                        : null,
                    onNavPointThetaEnd: widget.editMode 
                        ? (p) {
                            final start = _draggingNavPointsStart[p.name];
                            final current = _draggingNavPoints[p.name] ?? p;
                            if (start != null) {
                              widget.onNavPointEditEnd?.call(start, current);
                            }
                            _draggingNavPoints.remove(p.name);
                            _draggingNavPointsStart.remove(p.name);
                            setState(() {});
                          }
                        : null,
                    onNavPointMoveEnd: widget.editMode 
                        ? (p) {
                            final start = _draggingNavPointsStart[p.name];
                            final current = _draggingNavPoints[p.name] ?? p;
                            if (start != null) {
                              widget.onNavPointEditEnd?.call(start, current);
                            }
                            _draggingNavPoints.remove(p.name);
                            _draggingNavPointsStart.remove(p.name);
                            setState(() {});
                          }
                        : null,
                  );
                },
              );
            },
          ));
        }

        // 障碍物编辑层：放到最底层，避免遮挡其他图层
        if (_obstacleEdits.isNotEmpty) {
          layers.insert(0, _buildObstacleEditLayer(meta, toLatLng));
        }
        if (globalState.isLayerVisible('showLocalCostmap')) {
          layers.add(ValueListenableBuilder(
            valueListenable: rosChannel.localCostmap,
            builder: (_, cm, ___) =>
                buildLocalCostMapOverlayLayer(cm, 0.5),
          ));
        }

        if(!widget.editMode) {

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
        }

        return Stack(children: layers);
      },
    );
  }

  Widget _buildObstacleEditLayer(MapMeta meta, WorldToLatLngFn toLatLng) {
    final res = meta.resolution;
    final polygons = <Polygon>[];
    _obstacleEdits.forEach((key, value) {
      final col = key % meta.width;
      final row = key ~/ meta.width;
      final cx = meta.originX + col * res;
      final cy = meta.originY + row * res;
      final color = value > 0
          ? const Color(0xFF000000)
          : const Color(0xFFFFFFFF);
      polygons.add(Polygon(
        points: [
          toLatLng(cx, cy),
          toLatLng(cx + res, cy),
          toLatLng(cx + res, cy + res),
          toLatLng(cx, cy + res),
        ],
        color: color,
        borderColor: color,
        borderStrokeWidth: 0,
      ));
    });

    if (polygons.isEmpty) return const SizedBox.shrink();
    return PolygonLayer(polygons: polygons);
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

