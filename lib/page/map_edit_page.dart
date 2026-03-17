import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/tile_map.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:toastification/toastification.dart';

enum EditToolType {
  Move,
  AddNavPoint,
  AddRoute,
  BrushObstacle,
  EraseObstacle,
}

class MapEditPage extends StatefulWidget {
  final VoidCallback? onExit;

  const MapEditPage({super.key, this.onExit});

  @override
  State<MapEditPage> createState() => _MapEditPageState();
}

class _MapEditPageState extends State<MapEditPage> {
  EditToolType? selectedTool;
  NavPoint? selectedNavPoint;
  final GlobalKey<TileMapState> _tileMapKey = GlobalKey<TileMapState>();
  String? _routeStartPointName;
  TopologyRoute? _selectedRoute;
  RouteInfo? _editingRouteInfo;
  double _obstacleBrushSizeMeters = 0.05;
  late final GlobalState _globalState;

  @override
  void initState() {
    super.initState();
    selectedTool = EditToolType.Move;
    _globalState = context.read<GlobalState>();
    _globalState.mode.value = Mode.mapEdit;
  }
  
  @override
  void dispose() {
    super.dispose();
  }



  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final rosChannel = context.read<RosChannel>();
    final mapManager = rosChannel.mapManager;

    return WillPopScope(
      onWillPop: () async {
        _globalState.mode.value = Mode.normal;
        return true;
      },
      child: Scaffold(
        body: Stack(
          children: [
            TileMap(
              key: _tileMapKey,
              enableMapInteraction: selectedTool == EditToolType.Move,
              enableTopologyEdit: true,
              obstacleEditTool: selectedTool == EditToolType.BrushObstacle
                  ? ObstacleEditTool.Brush
                  : selectedTool == EditToolType.EraseObstacle
                      ? ObstacleEditTool.Eraser
                      : ObstacleEditTool.None,
              obstacleBrushSizeMeters: _obstacleBrushSizeMeters,
              selectedNavPointName: selectedNavPoint?.name,
              selectedRoute: _selectedRoute,
              onRouteTap: (route) {
                setState(() {
                  _selectedRoute = route;
                  _editingRouteInfo = RouteInfo(controller: route.routeInfo.controller);
                });
              },
              onNavPointTap: (p) {
                setState(() {
                  selectedNavPoint = p;
                });
                if (selectedTool == EditToolType.AddRoute) {
                  _handleAddRouteTap(rosChannel, p);
                } else {
                  setState(() {
                    _routeStartPointName = null;
                  });
                }
              },
              onTapWorld: (worldX, worldY) async {
                if (selectedTool != EditToolType.AddNavPoint) return;

                final name = await _showAddNavPointDialog(
                  context,
                  mapManager,
                  worldX,
                  worldY,
                );
                if (!mounted) return;
                if (name == null || name.trim().isEmpty) return;

                final navPoint = NavPoint(
                  name: name.trim(),
                  x: worldX,
                  y: worldY,
                  theta: 0.0,
                  type: NavPointType.navGoal,
                );
                mapManager.addNavPoint(navPoint);
                setState(() {
                  selectedNavPoint = navPoint;
                });
              },
              followRobot: false,
            ),
            Positioned(
              top: 0,
              left: 0,
              right: 0,
              child: _buildTopToolbar(context, theme),
            ),
            Positioned(
              left: 10,
              top: 80,
              child: _buildEditToolbar(theme),
            ),
            Positioned(
              left: 10,
              bottom: 16,
              child: _buildObstacleBrushSize(theme),
            ),
            Positioned(
              top: 80,
              right: 10,
              child: _buildRoutePanel(theme, rosChannel),
            ),
            Positioned(
              top: 80,
              right: 10,
              child: Padding(
                padding: const EdgeInsets.only(top: 220),
                child: _buildNavPointPanel(theme),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildTopToolbar(BuildContext context, ThemeData theme) {
    final rosChannel = context.read<RosChannel>();
    final mapManager = rosChannel.mapManager;

    return Container(
      height: 56,
      decoration: BoxDecoration(
        color: Colors.orange,
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.2),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Row(
        children: [
          const SizedBox(width: 8),
          IconButton(
            icon: const Icon(Icons.save, color: Colors.white, size: 26),
            tooltip: '保存并发布到ROS',
            onPressed: () async {
              try {
                _tileMapKey.currentState?.flushDraggingNavPoints();
                final topologyMap = mapManager.topologyMap.value;
                await rosChannel.updateTopologyMap(topologyMap);
                await mapManager.saveLocalTopologyMap();
                await rosChannel.publishOccupancyGrid();

                if (!mounted) return;
                toastification.show(
                  context: context,
                  type: ToastificationType.success,
                  style: ToastificationStyle.flatColored,
                  title: const Text('保存成功'),
                  description: const Text('已发布拓扑地图与栅格地图'),
                  autoCloseDuration: const Duration(seconds: 3),
                );
              } catch (e) {
                if (!mounted) return;
                toastification.show(
                  context: context,
                  type: ToastificationType.error,
                  style: ToastificationStyle.flatColored,
                  title: const Text('保存失败'),
                  description: Text('$e'),
                  autoCloseDuration: const Duration(seconds: 3),
                );
              }
            },
          ),
          const SizedBox(width: 8),
          const Expanded(
            child: Center(
              child: Text(
                '地图编辑',
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ),
          IconButton(
            icon: const Icon(Icons.close, color: Colors.white, size: 26),
            tooltip: '退出',
            onPressed: () async {
              _tileMapKey.currentState?.flushDraggingNavPoints();
              await mapManager.saveLocalTopologyMap();
              widget.onExit?.call();
              _globalState.mode.value = Mode.normal;
              if (mounted) Navigator.pop(context);
            },
          ),
          const SizedBox(width: 8),
        ],
      ),
    );
  }

  Widget _buildEditToolbar(ThemeData theme) {
    return Card(
      elevation: 8,
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            _buildToolButton(
              icon: Icons.open_with,
              label: '移动/选择',
              tool: EditToolType.Move,
              activeColor: Colors.grey,
            ),
            const SizedBox(height: 8),
            _buildToolButton(
              icon: Icons.add_location,
              label: '添加点位',
              tool: EditToolType.AddNavPoint,
              activeColor: Colors.blue,
            ),
            const SizedBox(height: 8),
            _buildToolButton(
              icon: Icons.link,
              label: '拓扑连线',
              tool: EditToolType.AddRoute,
              activeColor: Colors.deepPurple,
            ),
            const SizedBox(height: 8),
            _buildToolButton(
              icon: Icons.brush,
              label: '画笔',
              tool: EditToolType.BrushObstacle,
              activeColor: Colors.green,
            ),
            const SizedBox(height: 8),
            _buildToolButton(
              icon: Icons.auto_fix_high,
              label: '橡皮擦',
              tool: EditToolType.EraseObstacle,
              activeColor: Colors.red,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildObstacleBrushSize(ThemeData theme) {
    final show = selectedTool == EditToolType.BrushObstacle ||
        selectedTool == EditToolType.EraseObstacle;
    if (!show) return const SizedBox.shrink();
    return Card(
      elevation: 10,
      child: SizedBox(
        width: 260,
        child: Padding(
          padding: const EdgeInsets.all(10),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                '画笔大小: ${_obstacleBrushSizeMeters.toStringAsFixed(2)} m',
                style: theme.textTheme.labelLarge?.copyWith(fontWeight: FontWeight.w700),
              ),
              Slider(
                value: _obstacleBrushSizeMeters,
                min: 0.05,
                max: 1.0,
                divisions: 19,
                onChanged: (v) => setState(() => _obstacleBrushSizeMeters = v),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildToolButton({
    required IconData icon,
    required String label,
    required EditToolType tool,
    required Color activeColor,
  }) {
    final isActive = selectedTool == tool;
    return InkWell(
      onTap: () {
        setState(() {
          selectedTool = isActive ? null : tool;
          if (selectedTool != EditToolType.AddRoute) {
            _routeStartPointName = null;
          }
          if (selectedTool != EditToolType.AddRoute) {
            _selectedRoute = null;
            _editingRouteInfo = null;
          }
        });
      },
      child: Container(
        width: 120,
        padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 6),
        decoration: BoxDecoration(
          color: isActive ? activeColor.withOpacity(0.15) : Colors.transparent,
          borderRadius: BorderRadius.circular(8),
          border: isActive ? Border.all(color: activeColor, width: 2) : null,
        ),
        child: Column(
          children: [
            Icon(icon, size: 22, color: isActive ? activeColor : Colors.grey),
            const SizedBox(height: 4),
            Text(
              label,
              style: TextStyle(
                fontSize: 12,
                fontWeight: isActive ? FontWeight.bold : FontWeight.normal,
                color: isActive ? activeColor : Colors.grey,
              ),
            ),
          ],
        ),
      ),
    );
  }

  void _handleAddRouteTap(RosChannel rosChannel, NavPoint? p) {
    if (p == null) {
      setState(() {
        _routeStartPointName = null;
      });
      return;
    }
    final mapManager = rosChannel.mapManager;
    if (_routeStartPointName == null) {
      setState(() {
        _routeStartPointName = p.name;
      });
      toastification.show(
        context: context,
        title: Text('已选择起点: ${p.name}'),
        autoCloseDuration: const Duration(seconds: 2),
      );
      return;
    }

    if (_routeStartPointName == p.name) {
      setState(() {
        _routeStartPointName = null;
      });
      return;
    }

    final route = TopologyRoute(
      fromPoint: _routeStartPointName!,
      toPoint: p.name,
      routeInfo: RouteInfo(controller: 'FollowPath'),
    );
    mapManager.addRoute(route);
    setState(() {
      _routeStartPointName = null;
    });

    toastification.show(
      context: context,
      title: Text('已创建连线: ${route.fromPoint} -> ${route.toPoint}'),
      autoCloseDuration: const Duration(seconds: 2),
    );
  }

  Widget _buildNavPointPanel(ThemeData theme) {
    final p = selectedNavPoint;
    if (p == null) return const SizedBox.shrink();
    return Card(
      elevation: 10,
      child: SizedBox(
        width: 320,
        child: Padding(
          padding: const EdgeInsets.all(12),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                children: [
                  Expanded(
                    child: Text(
                      '点位属性',
                      style: theme.textTheme.titleMedium?.copyWith(
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ),
                  IconButton(
                    tooltip: '关闭',
                    onPressed: () {
                      setState(() {
                        selectedNavPoint = null;
                      });
                    },
                    icon: const Icon(Icons.close),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              _buildKv('名称', p.name),
              _buildKv('X', p.x.toStringAsFixed(3)),
              _buildKv('Y', p.y.toStringAsFixed(3)),
              _buildKv('Theta', p.theta.toStringAsFixed(3)),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildKv(String k, String v) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        children: [
          SizedBox(
            width: 64,
            child: Text(
              k,
              style: const TextStyle(fontWeight: FontWeight.w700),
            ),
          ),
          Expanded(child: Text(v)),
        ],
      ),
    );
  }

  Widget _buildRoutePanel(ThemeData theme, RosChannel rosChannel) {
    final route = _selectedRoute;
    if (route == null) return const SizedBox.shrink();

    final mapManager = rosChannel.mapManager;
    final topologyMap = mapManager.topologyMap.value;
    final supportControllers = topologyMap.mapProperty.supportControllers;
    final controllerOptions = supportControllers.isNotEmpty
        ? supportControllers
        : <String>{route.routeInfo.controller, 'FollowPath'}.toList();

    final info = _editingRouteInfo ?? route.routeInfo;
    return Card(
      elevation: 10,
      child: SizedBox(
        width: 320,
        child: Padding(
          padding: const EdgeInsets.all(12),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                children: [
                  Expanded(
                    child: Text(
                      '拓扑线属性',
                      style: theme.textTheme.titleMedium?.copyWith(
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ),
                  IconButton(
                    tooltip: '关闭',
                    onPressed: () {
                      setState(() {
                        _selectedRoute = null;
                        _editingRouteInfo = null;
                      });
                    },
                    icon: const Icon(Icons.close),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Text('方向: ${route.fromPoint} -> ${route.toPoint}'),
              const SizedBox(height: 12),
              DropdownButtonFormField<String>(
                value: info.controller,
                decoration: const InputDecoration(
                  labelText: '控制器',
                  border: OutlineInputBorder(),
                  isDense: true,
                ),
                items: controllerOptions
                    .map((c) => DropdownMenuItem(value: c, child: Text(c)))
                    .toList(),
                onChanged: (value) {
                  if (value == null) return;
                  setState(() {
                    _editingRouteInfo = RouteInfo(controller: value);
                  });
                  mapManager.updateRoute(
                    route.fromPoint,
                    route.toPoint,
                    TopologyRoute(
                      fromPoint: route.fromPoint,
                      toPoint: route.toPoint,
                      routeInfo: RouteInfo(controller: value),
                    ),
                  );
                },
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: () {
                        mapManager.removeRoute(route.fromPoint, route.toPoint);
                        setState(() {
                          _selectedRoute = null;
                          _editingRouteInfo = null;
                        });
                      },
                      icon: const Icon(Icons.delete, size: 18),
                      label: const Text('删除该方向'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                        foregroundColor: Colors.white,
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }


  Future<String?> _showAddNavPointDialog(
    BuildContext context,
    dynamic mapManager,
    double x,
    double y,
  ) async {
    final controller = TextEditingController();
    final id = await mapManager.getNextPointId();
    controller.text = 'NAV_POINT_$id';

    return showDialog<String>(
      context: context,
      builder: (context) {
        return AlertDialog(
          title: const Text('添加导航点'),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text('位置: (${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)})'),
              const SizedBox(height: 12),
              TextField(
                controller: controller,
                decoration: const InputDecoration(
                  labelText: '名称',
                  border: OutlineInputBorder(),
                ),
                autofocus: true,
              ),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('取消'),
            ),
            ElevatedButton(
              onPressed: () => Navigator.of(context).pop(controller.text),
              child: const Text('确定'),
            ),
          ],
        );
      },
    );
  }
}
