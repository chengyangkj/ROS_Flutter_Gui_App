import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/tile_map.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/page/map_edit_command.dart';
import 'package:toastification/toastification.dart';

enum EditToolType {
  Move,
  AddNavPoint,
  AddRoute,
  BrushObstacle,
  EraseObstacle,
}

class _UndoIntent extends Intent {
  const _UndoIntent();
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
  final CommandManager _commandManager = CommandManager();

  @override
  void initState() {
    super.initState();
    selectedTool = EditToolType.Move;
    _globalState = context.read<GlobalState>();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      _globalState.mode.value = Mode.mapEdit;
    });
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
      child: Shortcuts(
        shortcuts: <LogicalKeySet, Intent>{
          LogicalKeySet(LogicalKeyboardKey.control, LogicalKeyboardKey.keyZ):
              const _UndoIntent(),
        },
        child: Actions(
          actions: <Type, Action<Intent>>{
            _UndoIntent: CallbackAction<Intent>(
              onInvoke: (intent) {
                if (_commandManager.canUndo()) {
                  setState(() {
                    _commandManager.undo();
                  });
                }
                return null;
              },
            ),
          },
          child: Scaffold(
            body: Stack(
              children: [
                TileMap(
                  key: _tileMapKey,
                  enableMapInteraction: selectedTool == EditToolType.Move,
                  editMode: true,
                  obstacleEditTool: selectedTool == EditToolType.BrushObstacle
                      ? ObstacleEditTool.Brush
                      : selectedTool == EditToolType.EraseObstacle
                          ? ObstacleEditTool.Eraser
                          : ObstacleEditTool.None,
                  obstacleBrushSizeMeters: _obstacleBrushSizeMeters,
                onObstacleEditEnd: (oldEdits, newEdits) {
                  if (oldEdits.isEmpty && newEdits.isEmpty) return;
                  _commandManager.executeCommand(
                    ObstacleEditCommand(
                      _tileMapKey,
                      oldEdits,
                      newEdits,
                    ),
                  );
                },
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
                  onNavPointEditEnd: (oldPoint, newPoint) {
                    _commandManager.executeCommand(
                      ModifyPointCommand(
                        mapManager.topologyMap.value,
                        oldPoint,
                        newPoint,
                        () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                      ),
                    );
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
                _commandManager.executeCommand(
                  AddPointCommand(
                    mapManager.topologyMap.value,
                    navPoint,
                    () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                  ),
                );
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
              left: 10,
              bottom: 64,
              child: _buildAddRobotPositionButton(theme),
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
            icon: const Icon(Icons.undo, color: Colors.white, size: 26),
            tooltip: '撤销',
            onPressed: _commandManager.canUndo()
                ? () {
                    setState(() {
                      _commandManager.undo();
                    });
                  }
                : null,
          ),
          const SizedBox(width: 8),
          IconButton(
            icon: const Icon(Icons.save, color: Colors.white, size: 26),
            tooltip: '保存并发布到ROS',
            onPressed: () async {
              try {
                _tileMapKey.currentState?.flushDraggingNavPoints();
                final topologyMap = mapManager.topologyMap.value;
                final obstacleEdits =
                    _tileMapKey.currentState?.getObstacleEdits() ?? {};
                final editSessionId = DateTime.now().millisecondsSinceEpoch.toString();

                await mapManager.saveLocalTopologyMap();
                await rosChannel.updateMapEditHttp(
                  editSessionId: editSessionId,
                  topologyMap: topologyMap,
                  obstacleEdits: obstacleEdits,
                );

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

  Widget _buildAddRobotPositionButton(ThemeData theme) {
    final show = selectedTool == EditToolType.AddNavPoint;
    return AnimatedOpacity(
      opacity: show ? 1.0 : 0.0,
      duration: const Duration(milliseconds: 200),
      child: SizedBox(
        width: 220,
        child: ElevatedButton.icon(
          onPressed: show ? _onAddRobotPositionPressed : null,
          icon: const Icon(Icons.my_location, size: 18),
          label: const Text('添加机器人当前位置'),
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.blue,
            foregroundColor: Colors.white,
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(10),
            ),
            elevation: 4,
          ),
        ),
      ),
    );
  }

  void _onAddRobotPositionPressed() {
    final rosChannel = context.read<RosChannel>();
    final mapManager = rosChannel.mapManager;
    final pose = rosChannel.robotPoseMap.value;

    final nextIdFuture = mapManager.getNextPointId();
    nextIdFuture.then((id) {
      final navPoint = NavPoint(
        name: 'NAV_POINT_$id',
        x: pose.x,
        y: pose.y,
        theta: pose.theta,
        type: NavPointType.navGoal,
      );

      _commandManager.executeCommand(
        AddPointCommand(
          mapManager.topologyMap.value,
          navPoint,
          () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
        ),
      );

      setState(() {
        selectedNavPoint = navPoint;
        _selectedRoute = null;
        _editingRouteInfo = null;
      });
    });
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
    _commandManager.executeCommand(
      AddRouteCommand(
        mapManager.topologyMap.value,
        route,
        () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
      ),
    );
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
    final mapManager = context.read<RosChannel>().mapManager;
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
              _buildEditableKvText(
                label: '名称',
                value: p.name,
                onCommitted: (newName) {
                  final oldPoint = p;
                  if (newName.trim().isEmpty) return;
                  final trimmed = newName.trim();
                  if (trimmed == oldPoint.name) return;

                  final newPoint = NavPoint(
                    name: trimmed,
                    x: oldPoint.x,
                    y: oldPoint.y,
                    theta: oldPoint.theta,
                    type: oldPoint.type,
                  );
                  _commandManager.executeCommand(
                    RenamePointCommand(
                      mapManager.topologyMap.value,
                      oldPoint,
                      newPoint,
                      () => mapManager.updateTopologyMap(
                        mapManager.topologyMap.value,
                      ),
                    ),
                  );
                  setState(() {
                    selectedNavPoint = newPoint;
                    _selectedRoute = null;
                    _editingRouteInfo = null;
                    _routeStartPointName = null;
                  });
                },
              ),
              _buildEditableKvNumber(
                label: 'X',
                value: p.x,
                onCommitted: (newX) {
                  final oldPoint = p;
                  final newPoint = NavPoint(
                    name: oldPoint.name,
                    x: newX,
                    y: oldPoint.y,
                    theta: oldPoint.theta,
                    type: oldPoint.type,
                  );
                  _commandManager.executeCommand(
                    ModifyPointCommand(
                      mapManager.topologyMap.value,
                      oldPoint,
                      newPoint,
                      () => mapManager.updateTopologyMap(
                        mapManager.topologyMap.value,
                      ),
                    ),
                  );
                  setState(() {
                    selectedNavPoint = newPoint;
                  });
                },
              ),
              _buildEditableKvNumber(
                label: 'Y',
                value: p.y,
                onCommitted: (newY) {
                  final oldPoint = p;
                  final newPoint = NavPoint(
                    name: oldPoint.name,
                    x: oldPoint.x,
                    y: newY,
                    theta: oldPoint.theta,
                    type: oldPoint.type,
                  );
                  _commandManager.executeCommand(
                    ModifyPointCommand(
                      mapManager.topologyMap.value,
                      oldPoint,
                      newPoint,
                      () => mapManager.updateTopologyMap(
                        mapManager.topologyMap.value,
                      ),
                    ),
                  );
                  setState(() {
                    selectedNavPoint = newPoint;
                  });
                },
              ),
              _buildEditableKvNumber(
                label: 'Theta',
                value: p.theta,
                onCommitted: (newTheta) {
                  final oldPoint = p;
                  final newPoint = NavPoint(
                    name: oldPoint.name,
                    x: oldPoint.x,
                    y: oldPoint.y,
                    theta: newTheta,
                    type: oldPoint.type,
                  );
                  _commandManager.executeCommand(
                    ModifyPointCommand(
                      mapManager.topologyMap.value,
                      oldPoint,
                      newPoint,
                      () => mapManager.updateTopologyMap(
                        mapManager.topologyMap.value,
                      ),
                    ),
                  );
                  setState(() {
                    selectedNavPoint = newPoint;
                  });
                },
              ),
              const SizedBox(height: 12),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: () {
                    final oldPoint = selectedNavPoint;
                    if (oldPoint == null) return;
                    _commandManager.executeCommand(
                      DeletePointCommand(
                        mapManager.topologyMap.value,
                        oldPoint,
                        () => mapManager.updateTopologyMap(
                          mapManager.topologyMap.value,
                        ),
                      ),
                    );
                    setState(() {
                      selectedNavPoint = null;
                      _selectedRoute = null;
                      _editingRouteInfo = null;
                      _routeStartPointName = null;
                    });
                  },
                  icon: const Icon(Icons.delete, size: 18),
                  label: const Text('删除该点位'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red,
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(vertical: 12),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildEditableKvText({
    required String label,
    required String value,
    required ValueChanged<String> onCommitted,
  }) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          SizedBox(
            width: 64,
            child: Text(
              label,
              style: const TextStyle(fontWeight: FontWeight.w700),
            ),
          ),
          Expanded(
            child: TextField(
              controller: TextEditingController(text: value),
              decoration: const InputDecoration(
                isDense: true,
                border: OutlineInputBorder(),
              ),
              onSubmitted: (s) => onCommitted(s),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEditableKvNumber({
    required String label,
    required double value,
    required ValueChanged<double> onCommitted,
  }) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          SizedBox(
            width: 64,
            child: Text(
              label,
              style: const TextStyle(fontWeight: FontWeight.w700),
            ),
          ),
          Expanded(
            child: TextField(
              controller: TextEditingController(text: value.toStringAsFixed(3)),
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              decoration: const InputDecoration(
                isDense: true,
                border: OutlineInputBorder(),
              ),
              onSubmitted: (s) {
                final parsed = double.tryParse(s);
                if (parsed == null) return;
                onCommitted(parsed);
              },
            ),
          ),
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
                  final oldRoute = mapManager.getRoute(route.fromPoint, route.toPoint);
                  if (oldRoute == null) return;
                  final newRoute = TopologyRoute(
                    fromPoint: route.fromPoint,
                    toPoint: route.toPoint,
                    routeInfo: RouteInfo(controller: value),
                  );
                  _commandManager.executeCommand(
                    ModifyRouteCommand(
                      mapManager.topologyMap.value,
                      oldRoute,
                      newRoute,
                      () =>
                          mapManager.updateTopologyMap(mapManager.topologyMap.value),
                    ),
                  );
                  setState(() {
                    _editingRouteInfo = RouteInfo(controller: value);
                  });
                },
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: () {
                        final oldRoute = mapManager.getRoute(route.fromPoint, route.toPoint);
                        if (oldRoute != null) {
                          _commandManager.executeCommand(
                            DeleteRouteCommand(
                              mapManager.topologyMap.value,
                              oldRoute,
                              () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                            ),
                          );
                        }
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
