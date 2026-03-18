import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/tile_map.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/http_channel.dart';
import 'package:ros_flutter_gui_app/provider/map_manager.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/page/map_edit_command.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
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
  String _currentMapName = '';

  @override
  void initState() {
    super.initState();
    selectedTool = EditToolType.Move;
    _globalState = context.read<GlobalState>();
    WidgetsBinding.instance.addPostFrameCallback((_) async {
      if (!mounted) return;
      _globalState.mode.value = Mode.mapEdit;
      try {
        final httpChannel = context.read<HttpChannel>();
        final mapManager = context.read<RosChannel>().mapManager;
        _currentMapName = await httpChannel.getCurrentMap();
        final topo = await httpChannel.getTopologyMap();
        mapManager.updateTopologyMap(topo);
        if (mounted) setState(() {});
      } catch (_) {}
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
                    _showRouteDialog(context, theme, rosChannel);
                  },
                  onNavPointTap: (p) {
                    setState(() {
                      selectedNavPoint = p;
                    });
                    if (selectedTool == EditToolType.AddRoute) {
                      _handleAddRouteTap(rosChannel, p);
                    } else if (p != null) {
                      setState(() => _routeStartPointName = null);
                      _showNavPointDialog(context, theme);
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
                    if (mounted) setState(() => selectedNavPoint = navPoint);
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
                  right: 0,
                  top: 48,
                  bottom: 0,
                  width: 64,
                  child: Container(
                    color: theme.colorScheme.surfaceContainerHighest,
                    child: Column(
                      children: [
                        Expanded(
                          child: SingleChildScrollView(
                            padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 6),
                            child: _buildEditToolbar(context, theme),
                          ),
                        ),
                        _buildObstacleBrushSize(theme),
                      ],
                    ),
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
    final httpChannel = context.read<HttpChannel>();
    final mapManager = rosChannel.mapManager;

    return Container(
      height: 48,
      decoration: BoxDecoration(
        color: Colors.orange.shade700,
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.08),
            blurRadius: 2,
            offset: const Offset(0, 1),
          ),
        ],
      ),
      child: Row(
        children: [
          const SizedBox(width: 6),
          _buildUndoButton(),
          const SizedBox(width: 2),
          _buildSaveButton(httpChannel, mapManager),
          const SizedBox(width: 2),
          _buildSaveAsButton(httpChannel, mapManager),
          const SizedBox(width: 2),
          _buildMapManageButton(context, theme, httpChannel, mapManager),
          if (selectedTool == EditToolType.AddNavPoint) ...[
            const SizedBox(width: 4),
            TextButton.icon(
              onPressed: _onAddRobotPositionPressed,
              icon: const Icon(Icons.my_location, color: Colors.white, size: 16),
              label: Text(AppLocalizations.of(context)!.add_current_position, style: const TextStyle(color: Colors.white, fontSize: 12)),
              style: TextButton.styleFrom(
                backgroundColor: Colors.blue.shade700,
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                minimumSize: Size.zero,
                tapTargetSize: MaterialTapTargetSize.shrinkWrap,
              ),
            ),
          ],
          Expanded(
            child: Center(
              child: Text(
                AppLocalizations.of(context)!.map_edit,
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 15,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ),
          ),
          IconButton(
            icon: const Icon(Icons.close, color: Colors.white, size: 22),
            iconSize: 22,
            padding: const EdgeInsets.all(8),
            constraints: const BoxConstraints(),
            tooltip: AppLocalizations.of(context)!.exit,
            onPressed: () async {
              _tileMapKey.currentState?.flushDraggingNavPoints();
              widget.onExit?.call();
              _globalState.mode.value = Mode.normal;
              if (mounted) Navigator.pop(context);
            },
          ),
          const SizedBox(width: 6),
        ],
      ),
    );
  }

  Widget _buildUndoButton() {
    final canUndo = _commandManager.canUndo();
    return TextButton(
      onPressed: canUndo
          ? () {
              setState(() {
                _commandManager.undo();
              });
            }
          : null,
      style: TextButton.styleFrom(
        foregroundColor: Colors.white,
        disabledForegroundColor: Colors.white54,
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        minimumSize: Size.zero,
        tapTargetSize: MaterialTapTargetSize.shrinkWrap,
      ),
      child: Text(AppLocalizations.of(context)!.undo, style: const TextStyle(fontSize: 13)),
    );
  }

  Widget _buildMapManageButton(
    BuildContext context,
    ThemeData theme,
    HttpChannel httpChannel,
    MapManager mapManager,
  ) {
    return TextButton(
      style: TextButton.styleFrom(
        foregroundColor: Colors.white,
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        minimumSize: Size.zero,
        tapTargetSize: MaterialTapTargetSize.shrinkWrap,
      ),
      onPressed: () => _showMapManagementDialog(context, theme, httpChannel, mapManager),
      child: Text(AppLocalizations.of(context)!.map_management, style: const TextStyle(fontSize: 13)),
    );
  }

  Widget _buildSaveButton(HttpChannel httpChannel, MapManager mapManager) {
    return TextButton(
      style: TextButton.styleFrom(
        foregroundColor: Colors.white,
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        minimumSize: Size.zero,
        tapTargetSize: MaterialTapTargetSize.shrinkWrap,
      ),
      onPressed: () async {
        try {
          _tileMapKey.currentState?.flushDraggingNavPoints();
          final topologyMap = mapManager.topologyMap.value;
          final obstacleEdits =
              _tileMapKey.currentState?.getObstacleEdits() ?? {};
          final editSessionId = DateTime.now().millisecondsSinceEpoch.toString();
          await httpChannel.updateMapEdit(
            editSessionId: editSessionId,
            topologyMap: topologyMap,
            obstacleEdits: obstacleEdits,
          );
          if (!mounted) return;
          _tileMapKey.currentState?.loadMeta();
          toastification.show(
            context: context,
            type: ToastificationType.success,
            style: ToastificationStyle.flatColored,
            title: Text(AppLocalizations.of(context)!.save_success),
            description: Text(AppLocalizations.of(context)!.save_success_desc),
            autoCloseDuration: const Duration(seconds: 3),
          );
        } catch (e) {
          if (!mounted) return;
          final l10n = AppLocalizations.of(context)!;
          final desc = e.toString().contains('no_map_available') ? l10n.no_map_available : e.toString();
          toastification.show(
            context: context,
            type: ToastificationType.error,
            style: ToastificationStyle.flatColored,
            title: Text(l10n.save_failed),
            description: Text(desc),
            autoCloseDuration: const Duration(seconds: 3),
          );
        }
      },
      child: Text(AppLocalizations.of(context)!.save, style: const TextStyle(fontSize: 13)),
    );
  }

  Widget _buildSaveAsButton(HttpChannel httpChannel, MapManager mapManager) {
    return TextButton(
      style: TextButton.styleFrom(
        foregroundColor: Colors.white,
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        minimumSize: Size.zero,
        tapTargetSize: MaterialTapTargetSize.shrinkWrap,
      ),
      onPressed: () => _showSaveAsDialog(context, httpChannel, mapManager),
      child: Text(AppLocalizations.of(context)!.save_as, style: const TextStyle(fontSize: 13)),
    );
  }

  Future<void> _showSaveAsDialog(
    BuildContext context,
    HttpChannel httpChannel,
    MapManager mapManager,
  ) async {
    final controller = TextEditingController(text: _currentMapName);
    final name = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text(AppLocalizations.of(context)!.save_as),
        content: TextField(
          controller: controller,
          decoration: InputDecoration(
            labelText: AppLocalizations.of(context)!.map_name,
            border: OutlineInputBorder(),
          ),
          autofocus: true,
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(),
            child: Text(AppLocalizations.of(context)!.cancel),
          ),
          ElevatedButton(
            onPressed: () => Navigator.of(ctx).pop(controller.text.trim()),
            child: Text(AppLocalizations.of(context)!.ok),
          ),
        ],
      ),
    );
    if (!mounted || name == null || name.isEmpty) return;
    try {
      _tileMapKey.currentState?.flushDraggingNavPoints();
      final topologyMap = mapManager.topologyMap.value;
      final obstacleEdits =
          _tileMapKey.currentState?.getObstacleEdits() ?? {};
      final editSessionId = DateTime.now().millisecondsSinceEpoch.toString();
      await httpChannel.updateMapEdit(
        editSessionId: editSessionId,
        topologyMap: topologyMap,
        obstacleEdits: obstacleEdits,
        mapName: name,
      );
      await httpChannel.setCurrentMap(name);
      final topo = await httpChannel.getTopologyMap(mapName: name);
      mapManager.updateTopologyMap(topo);
      _tileMapKey.currentState?.loadMeta();
      setState(() => _currentMapName = name);
      if (!mounted) return;
      toastification.show(
        context: context,
        type: ToastificationType.success,
        style: ToastificationStyle.flatColored,
        title: Text(AppLocalizations.of(context)!.save_as_success),
        description: Text(AppLocalizations.of(context)!.save_as_desc(name)),
        autoCloseDuration: const Duration(seconds: 3),
      );
    } catch (e) {
      if (!mounted) return;
      final l10n = AppLocalizations.of(context)!;
      final desc = e.toString().contains('no_map_available') ? l10n.no_map_available : e.toString();
      toastification.show(
        context: context,
        type: ToastificationType.error,
        style: ToastificationStyle.flatColored,
        title: Text(l10n.save_as_failed),
        description: Text(desc),
        autoCloseDuration: const Duration(seconds: 3),
      );
    }
  }

  void _showMapManagementDialog(
    BuildContext context,
    ThemeData theme,
    HttpChannel httpChannel,
    MapManager mapManager,
  ) {
    showDialog(
      context: context,
      builder: (ctx) => _MapManagementDialog(
        httpChannel: httpChannel,
        tileServerUrl: globalSetting.tileServerUrl,
        mapManager: mapManager,
        onSwitchMap: (name) async {
          await httpChannel.setCurrentMap(name);
          final topo = await httpChannel.getTopologyMap(mapName: name);
          mapManager.updateTopologyMap(topo);
          if (!ctx.mounted) return;
          Navigator.of(ctx).pop();
          if (!mounted) return;
          _tileMapKey.currentState?.loadMeta();
          setState(() => _currentMapName = name);
        },
      ),
    );
  }

  Widget _buildEditToolbar(BuildContext context, ThemeData theme) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        _buildToolButton(
          context,
          icon: Icons.open_with,
          label: AppLocalizations.of(context)!.tool_move,
          tool: EditToolType.Move,
          activeColor: Colors.grey,
        ),
        const SizedBox(height: 4),
        _buildToolButton(
          context,
          icon: Icons.add_location,
          label: AppLocalizations.of(context)!.tool_point,
          tool: EditToolType.AddNavPoint,
          activeColor: Colors.blue,
        ),
        const SizedBox(height: 4),
        _buildToolButton(
          context,
          icon: Icons.link,
          label: AppLocalizations.of(context)!.tool_route,
          tool: EditToolType.AddRoute,
          activeColor: Colors.deepPurple,
        ),
        const SizedBox(height: 4),
        _buildToolButton(
          context,
          icon: Icons.brush,
          label: AppLocalizations.of(context)!.tool_brush,
          tool: EditToolType.BrushObstacle,
          activeColor: Colors.green,
        ),
        const SizedBox(height: 4),
        _buildToolButton(
          context,
          icon: Icons.auto_fix_high,
          label: AppLocalizations.of(context)!.tool_eraser,
          tool: EditToolType.EraseObstacle,
          activeColor: Colors.red,
        ),
      ],
    );
  }

  Widget _buildObstacleBrushSize(ThemeData theme) {
    final show = selectedTool == EditToolType.BrushObstacle ||
        selectedTool == EditToolType.EraseObstacle;
    if (!show) return const SizedBox.shrink();
    return Container(
      padding: const EdgeInsets.fromLTRB(6, 8, 6, 12),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            '${_obstacleBrushSizeMeters.toStringAsFixed(2)}m',
            style: theme.textTheme.labelSmall?.copyWith(fontWeight: FontWeight.w600),
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
    );
  }

  void _onAddRobotPositionPressed() async {
    final rosChannel = context.read<RosChannel>();
    final mapManager = rosChannel.mapManager;
    final pose = rosChannel.robotPoseMap.value;

    final id = await mapManager.getNextPointId();
    if (!mounted) return;

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

    if (!mounted) return;
    setState(() {
      selectedNavPoint = navPoint;
      _selectedRoute = null;
      _editingRouteInfo = null;
    });
  }

  Widget _buildToolButton(
    BuildContext context, {
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
        width: 52,
        padding: const EdgeInsets.symmetric(vertical: 6, horizontal: 4),
        decoration: BoxDecoration(
          color: isActive ? activeColor.withOpacity(0.2) : Colors.transparent,
          borderRadius: BorderRadius.circular(6),
          border: isActive ? Border.all(color: activeColor, width: 2) : null,
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              icon,
              size: 20,
              color: isActive ? activeColor : Colors.grey,
            ),
            const SizedBox(height: 2),
            Text(
              label,
              style: TextStyle(
                fontSize: 10,
                fontWeight: isActive ? FontWeight.w600 : FontWeight.normal,
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
        title: Text(AppLocalizations.of(context)!.route_start_selected(p.name)),
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
      title: Text(AppLocalizations.of(context)!.route_created(route.fromPoint, route.toPoint)),
      autoCloseDuration: const Duration(seconds: 2),
    );
  }

  void _showRouteDialog(BuildContext context, ThemeData theme, RosChannel rosChannel) {
    final route = _selectedRoute;
    if (route == null) return;
    final mapManager = rosChannel.mapManager;
    final topologyMap = mapManager.topologyMap.value;
    final supportControllers = topologyMap.mapProperty.supportControllers;
    final controllerOptions = supportControllers.isNotEmpty
        ? supportControllers
        : <String>{route.routeInfo.controller, 'FollowPath'}.toList();
    String controllerValue = (_editingRouteInfo ?? route.routeInfo).controller;

    showDialog(
      context: context,
      barrierDismissible: true,
      builder: (ctx) => StatefulBuilder(
        builder: (context, setDialogState) {
          return AlertDialog(
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
            title: Text(AppLocalizations.of(context)!.route_properties),
            content: SingleChildScrollView(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(AppLocalizations.of(context)!.direction(route.fromPoint, route.toPoint)),
                  const SizedBox(height: 12),
                  DropdownButtonFormField<String>(
                    value: controllerValue,
                    decoration: InputDecoration(
                      labelText: AppLocalizations.of(context)!.controller,
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
                          () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                        ),
                      );
                      setDialogState(() => controllerValue = value);
                      setState(() => _editingRouteInfo = RouteInfo(controller: value));
                    },
                  ),
                ],
              ),
            ),
            actions: [
              TextButton(
                onPressed: () {
                  Navigator.of(ctx).pop();
                  setState(() {
                    _selectedRoute = null;
                    _editingRouteInfo = null;
                  });
                },
                child: Text(AppLocalizations.of(context)!.close),
              ),
              FilledButton.icon(
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
                  Navigator.of(ctx).pop();
                  setState(() {
                    _selectedRoute = null;
                    _editingRouteInfo = null;
                  });
                },
                icon: const Icon(Icons.delete, size: 18),
                label: Text(AppLocalizations.of(context)!.delete_route),
                style: FilledButton.styleFrom(
                  backgroundColor: Colors.red,
                  foregroundColor: Colors.white,
                ),
              ),
            ],
          );
        },
      ),
    );
  }

  void _showNavPointDialog(BuildContext context, ThemeData theme) {
    final p = selectedNavPoint;
    if (p == null) return;
    final mapManager = context.read<RosChannel>().mapManager;
    NavPoint localPoint = p;

    showDialog(
      context: context,
      barrierDismissible: true,
      builder: (ctx) => StatefulBuilder(
        builder: (context, setDialogState) {
          return AlertDialog(
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
            title: Text(AppLocalizations.of(context)!.point_properties),
            content: SingleChildScrollView(
              child: SizedBox(
                width: 320,
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    _buildEditableKvText(
                      label: AppLocalizations.of(context)!.name,
                      value: localPoint.name,
                      onCommitted: (newName) {
                        final oldPoint = localPoint;
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
                            () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                          ),
                        );
                        setDialogState(() => localPoint = newPoint);
                        setState(() {
                          selectedNavPoint = newPoint;
                          _selectedRoute = null;
                          _editingRouteInfo = null;
                          _routeStartPointName = null;
                        });
                      },
                    ),
                    _buildEditableKvNumber(
                      label: AppLocalizations.of(context)!.coord_x,
                      value: localPoint.x,
                      onCommitted: (newX) {
                        final oldPoint = localPoint;
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
                            () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                          ),
                        );
                        setDialogState(() => localPoint = newPoint);
                        setState(() => selectedNavPoint = newPoint);
                      },
                    ),
                    _buildEditableKvNumber(
                      label: AppLocalizations.of(context)!.coord_y,
                      value: localPoint.y,
                      onCommitted: (newY) {
                        final oldPoint = localPoint;
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
                            () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                          ),
                        );
                        setDialogState(() => localPoint = newPoint);
                        setState(() => selectedNavPoint = newPoint);
                      },
                    ),
                    _buildEditableKvNumber(
                      label: AppLocalizations.of(context)!.heading,
                      value: localPoint.theta,
                      onCommitted: (newTheta) {
                        final oldPoint = localPoint;
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
                            () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                          ),
                        );
                        setDialogState(() => localPoint = newPoint);
                        setState(() => selectedNavPoint = newPoint);
                      },
                    ),
                  ],
                ),
              ),
            ),
            actions: [
              TextButton(
                onPressed: () {
                  Navigator.of(ctx).pop();
                  setState(() => selectedNavPoint = null);
                },
                child: Text(AppLocalizations.of(context)!.close),
              ),
              FilledButton.icon(
                onPressed: () {
                  final oldPoint = localPoint;
                  _commandManager.executeCommand(
                    DeletePointCommand(
                      mapManager.topologyMap.value,
                      oldPoint,
                      () => mapManager.updateTopologyMap(mapManager.topologyMap.value),
                    ),
                  );
                  Navigator.of(ctx).pop();
                  setState(() {
                    selectedNavPoint = null;
                    _selectedRoute = null;
                    _editingRouteInfo = null;
                    _routeStartPointName = null;
                  });
                },
                icon: const Icon(Icons.delete, size: 18),
                label: Text(AppLocalizations.of(context)!.delete_point),
                style: FilledButton.styleFrom(
                  backgroundColor: Colors.red,
                  foregroundColor: Colors.white,
                ),
              ),
            ],
          );
        },
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
          title: Text(AppLocalizations.of(context)!.add_nav_point),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(AppLocalizations.of(context)!.position_format(x.toStringAsFixed(2), y.toStringAsFixed(2))),
              const SizedBox(height: 12),
              TextField(
                controller: controller,
                decoration: InputDecoration(
                  labelText: AppLocalizations.of(context)!.name,
                  border: OutlineInputBorder(),
                ),
                autofocus: true,
              ),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: Text(AppLocalizations.of(context)!.cancel),
            ),
            ElevatedButton(
              onPressed: () => Navigator.of(context).pop(controller.text),
              child: Text(AppLocalizations.of(context)!.ok),
            ),
          ],
        );
      },
    );
  }
}

class _MapManagementDialog extends StatefulWidget {
  final HttpChannel httpChannel;
  final String tileServerUrl;
  final MapManager mapManager;
  final Future<void> Function(String name) onSwitchMap;

  const _MapManagementDialog({
    required this.httpChannel,
    required this.tileServerUrl,
    required this.mapManager,
    required this.onSwitchMap,
  });

  @override
  State<_MapManagementDialog> createState() => _MapManagementDialogState();
}

class _MapManagementDialogState extends State<_MapManagementDialog> {
  List<String> _mapNames = [];
  String _currentMap = '';
  bool _loading = true;
  String? _error;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    setState(() {
      _loading = true;
      _error = null;
    });
    try {
      final names = await widget.httpChannel.getAllMapList();
      final current = await widget.httpChannel.getCurrentMap();
      if (mounted) {
        setState(() {
          _mapNames = names;
          _currentMap = current;
          _loading = false;
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _error = e.toString();
          _loading = false;
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      title: Text(AppLocalizations.of(context)!.map_management),
      content: SizedBox(
        width: 400,
        child: _loading
            ? const Center(child: CircularProgressIndicator())
            : _error != null
                ? Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Text(_error!, style: const TextStyle(color: Colors.red)),
                      const SizedBox(height: 12),
                      TextButton(
                        onPressed: _load,
                        child: Text(AppLocalizations.of(context)!.retry),
                      ),
                    ],
                  )
                : _mapNames.isEmpty
                    ? Text(AppLocalizations.of(context)!.no_map)
                    : ListView.builder(
                        shrinkWrap: true,
                        itemCount: _mapNames.length,
                        itemBuilder: (context, index) {
                          final name = _mapNames[index];
                          final isCurrent = name == _currentMap;
                          final thumbUrl =
                              '${widget.tileServerUrl}/tiles/$name/0/0/0.png?id=$name';
                          return Padding(
                            key: ValueKey(name),
                            padding: const EdgeInsets.symmetric(vertical: 8),
                            child: Row(
                              children: [
                                ClipRRect(
                                  borderRadius: BorderRadius.circular(8),
                                  child: Image.network(
                                    thumbUrl,
                                    key: ValueKey(thumbUrl),
                                    gaplessPlayback: false,
                                    width: 64,
                                    height: 64,
                                    fit: BoxFit.cover,
                                    errorBuilder: (_, __, ___) => Container(
                                      width: 64,
                                      height: 64,
                                      color: Colors.grey.shade300,
                                      child: const Icon(Icons.map),
                                    ),
                                  ),
                                ),
                                const SizedBox(width: 12),
                                Expanded(
                                  child: Text(
                                    name,
                                    style: const TextStyle(
                                      fontWeight: FontWeight.w600,
                                      fontSize: 14,
                                    ),
                                  ),
                                ),
                                TextButton(
                                  onPressed: isCurrent
                                      ? null
                                      : () async {
                                          await widget.onSwitchMap(name);
                                        },
                                  child: Text(AppLocalizations.of(context)!.edit),
                                ),
                                
                                const SizedBox(width: 4),
                                if (isCurrent)
                                  Container(
                                    padding: const EdgeInsets.symmetric(
                                      horizontal: 8,
                                      vertical: 4,
                                    ),
                                    decoration: BoxDecoration(
                                      color: Colors.green.shade100,
                                      borderRadius: BorderRadius.circular(4),
                                    ),
                                    child: Text(
                                      AppLocalizations.of(context)!.current_in_use,
                                      style: TextStyle(
                                        fontSize: 12,
                                        color: Colors.green,
                                      ),
                                    ),
                                  )
                                else
                                  TextButton(
                                    onPressed: () async {
                                      await widget.onSwitchMap(name);
                                    },
                                    child: Text(AppLocalizations.of(context)!.switch_map),
                                  ),
                                  const SizedBox(width: 4),
                                IconButton(
                                  icon: Icon(
                                    Icons.delete,
                                    size: 20,
                                    color: isCurrent ? Colors.grey : Colors.red,
                                  ),
                                  tooltip: isCurrent ? AppLocalizations.of(context)!.delete_map_tooltip_current : AppLocalizations.of(context)!.delete,
                                  onPressed: isCurrent
                                      ? null
                                      : () async {
                                    final confirm = await showDialog<bool>(
                                      context: context,
                                      builder: (ctx) => AlertDialog(
                                        title: Text(AppLocalizations.of(context)!.confirm_delete),
                                        content: Text(AppLocalizations.of(context)!.confirm_delete_map(name)),
                                        actions: [
                                          TextButton(
                                            onPressed: () => Navigator.of(ctx).pop(false),
                                            child: Text(AppLocalizations.of(context)!.cancel),
                                          ),
                                          FilledButton(
                                            onPressed: () => Navigator.of(ctx).pop(true),
                                            style: FilledButton.styleFrom(backgroundColor: Colors.red),
                                            child: Text(AppLocalizations.of(context)!.delete),
                                          ),
                                        ],
                                      ),
                                    );
                                    if (confirm != true || !mounted) return;
                                    try {
                                      await widget.httpChannel.deleteMap(name);
                                      if (!mounted) return;
                                      await _load();
                                      if (!mounted) return;
                                      toastification.show(
                                        context: context,
                                        type: ToastificationType.success,
                                        title: Text(AppLocalizations.of(context)!.map_deleted(name)),
                                        autoCloseDuration: const Duration(seconds: 2),
                                      );
                                    } catch (e) {
                                      if (!mounted) return;
                                      toastification.show(
                                        context: context,
                                        type: ToastificationType.error,
                                        title: Text(AppLocalizations.of(context)!.delete_failed(e.toString())),
                                        autoCloseDuration: const Duration(seconds: 3),
                                      );
                                    }
                                  },
                                ),
                              ],
                            ),
                          );
                        },
                      ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.of(context).pop(),
          child: Text(AppLocalizations.of(context)!.close),
        ),
      ],
    );
  }
}
