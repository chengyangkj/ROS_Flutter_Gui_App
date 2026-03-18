import 'package:flutter/material.dart';
import 'package:flutter/gestures.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/display/tile_map.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/http_channel.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/page/map_edit_page.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:toastification/toastification.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/page/gamepad_widget.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_status.dart';
import 'package:ros_flutter_gui_app/page/diagnostic_page.dart';
import 'package:ros_flutter_gui_app/provider/diagnostic_manager.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';



class MainFlamePage extends StatefulWidget {
  @override
  _MainFlamePageState createState() => _MainFlamePageState();
}

class _MainFlamePageState extends State<MainFlamePage> {
  final GlobalKey<TileMapState> _tileMapKey = GlobalKey<TileMapState>();
  bool showLayerControl = false;
  bool showCamera = false;
  NavPoint? selectedNavPoint;
  TopologyRoute? _selectedRoute;
  RouteInfo? _editingRouteInfo;
  
  // 相机相关变量
  Offset camPosition = Offset(30, 10); // 初始位置
  bool isCamFullscreen = false; // 是否全屏
  Offset camPreviousPosition = Offset(30, 10); // 保存进入全屏前的位置
  late double camWidgetWidth;
  late double camWidgetHeight;

  @override
  void initState() {
    super.initState();
    Provider.of<GlobalState>(context, listen: false).loadLayerSettings();
    WidgetsBinding.instance.addPostFrameCallback((_) async {
      if (!mounted) return;
      try {
        final httpChannel = Provider.of<HttpChannel>(context, listen: false);
        final mapManager = Provider.of<RosChannel>(context, listen: false).mapManager;
        final topo = await httpChannel.getTopologyMap();
        mapManager.updateTopologyMap(topo);
      } catch (_) {}
    });

    // 初始化相机尺寸
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (mounted) {
        final screenSize = MediaQuery.of(context).size;
        camWidgetWidth = screenSize.width / 3.5;
        camWidgetHeight = camWidgetWidth / (globalSetting.imageWidth / globalSetting.imageHeight);
      }
    });
    
    // 监听诊断数据
    _setupDiagnosticListener();
  }
  
  Future<void> _reloadData() async {
    _tileMapKey.currentState?.loadMeta();
  }

  // 设置诊断数据监听器
  void _setupDiagnosticListener() {
    final rosChannel = Provider.of<RosChannel>(context, listen: false);
    
    // 设置新错误/警告回调
    rosChannel.diagnosticManager.setOnNewErrorsWarnings(_onNewErrorsWarnings);
  }


  // 新错误/警告/失活回调
  void _onNewErrorsWarnings(List<Map<String, dynamic>> newErrorsWarnings) {
    for (var errorWarning in newErrorsWarnings) {
      final hardwareId = errorWarning['hardwareId'] as String;
      final componentName = errorWarning['componentName'] as String;
      final state = errorWarning['state'] as DiagnosticState;
      
      // 只对错误、警告和失活状态显示toast
      if (state.level == DiagnosticStatus.ERROR || 
          state.level == DiagnosticStatus.WARN || 
          state.level == DiagnosticStatus.STALE) {
        _showDiagnosticToast(hardwareId, componentName, state);
      }
    }
  }

  // 显示诊断toast通知
  void _showDiagnosticToast(String hardwareId, String componentName, DiagnosticState state) {
    if (!mounted) return;
    
    String levelText;
    Color levelColor;
    ToastificationType toastType;
    IconData iconData;
    
    switch (state.level) {
      case DiagnosticStatus.WARN:
        levelText = AppLocalizations.of(context)!.diagnostic_warning;
        levelColor = Colors.orange;
        toastType = ToastificationType.warning;
        iconData = Icons.warning;
        break;
      case DiagnosticStatus.ERROR:
        levelText = AppLocalizations.of(context)!.diagnostic_error;
        levelColor = Colors.red;
        toastType = ToastificationType.error;
        iconData = Icons.error;
        break;
      case DiagnosticStatus.STALE:
        levelText = AppLocalizations.of(context)!.diagnostic_stale;
        levelColor = Colors.grey;
        toastType = ToastificationType.info;
        iconData = Icons.schedule;
        break;
      default:
        return; // 其他状态不显示toast
    }
    
    final l10n = AppLocalizations.of(context)!;
    final hwId = hardwareId == 'unknown_hardware' ? l10n.unknown_hardware : hardwareId;
    final msg = state.message == 'data_stale' ? l10n.data_stale : state.message;
    toastification.show(
      context: context,
      type: toastType,
      title: Text(l10n.diagnostic_health(levelText, componentName)),
      description: Text(l10n.diagnostic_hardware(hwId, msg)),
      autoCloseDuration: const Duration(seconds: 5),
      icon: Icon(
        iconData,
        color: levelColor,
      ),
    );
  }

    @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    
    return Scaffold(
          body: Stack(
            children: [
              TileMap(
                key: _tileMapKey,
                onTap: () {
                  setState(() {
                    selectedNavPoint = null;
                    _selectedRoute = null;
                    _editingRouteInfo = null;
                  });
                },
                onNavPointTap: (NavPoint? point) {
                  setState(() {
                    selectedNavPoint = point;
                    if (point != null) {
                      _selectedRoute = null;
                      _editingRouteInfo = null;
                      _showNavPointDialog(context, point);
                    }
                  });
                },
                selectedRoute: _selectedRoute,
                onRouteTap: (route) {
                  setState(() {
                    _selectedRoute = route;
                    _editingRouteInfo = RouteInfo(controller: route.routeInfo.controller);
                    selectedNavPoint = null;
                  });
                },
                selectedNavPointName: selectedNavPoint?.name,
                enableMapInteraction: Provider.of<GlobalState>(context, listen: true).mode.value != Mode.reloc,
                followRobot: Provider.of<GlobalState>(context, listen: true).mode.value == Mode.robotFixedCenter,
              ),
              _buildTopMenuBar(context, theme),
              _buildLeftToolbar(context, theme),
              _buildRightToolbar(context, theme),
              Positioned(
                top: 60,
                right: 5,
                child: _buildSelectionPanel(theme),
              ),
              _buildBottomControls(context, theme),
              _buildCameraWidget(context, theme),
              _buildGamepadWidget(context, theme),
              _buildMapLegend(context, theme),
            ],
          ),
        );
  }

  Widget _buildSelectionPanel(ThemeData theme) {
    final route = _selectedRoute;
    if (route != null) {
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
                        AppLocalizations.of(context)!.route_properties,
                        style: theme.textTheme.titleMedium?.copyWith(
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ),
                    IconButton(
                      tooltip: AppLocalizations.of(context)!.close,
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
                Text(AppLocalizations.of(context)!.direction(route.fromPoint, route.toPoint)),
                const SizedBox(height: 12),
                TextFormField(
                  initialValue: info.controller,
                  readOnly: true,
                  decoration: InputDecoration(
                    labelText: AppLocalizations.of(context)!.controller_readonly,
                    border: OutlineInputBorder(),
                    isDense: true,
                  ),
                ),
              ],
            ),
          ),
        ),
      );
    }

    return const SizedBox.shrink();
  }

  Widget _buildTopMenuBar(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 5,
      top: 1,
      child: Container(
        height: 50,
        padding: const EdgeInsets.symmetric(vertical: 8),
        child: SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          child: Row(
            children: [
              // 线速度显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: Icon(
                    const IconData(0xe606, fontFamily: "Speed"),
                    color: Colors.green[400],
                  ),
                  label: ValueListenableBuilder<RobotSpeed>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: true)
                            .robotSpeed_,
                    builder: (context, speed, child) {
                      return Text('${(speed.vx).toStringAsFixed(2)} m/s');
                    },
                  ),
                ),
              ),
              // 角速度显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: const Icon(IconData(0xe680, fontFamily: "Speed")),
                  label: ValueListenableBuilder<RobotSpeed>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: true)
                            .robotSpeed_,
                    builder: (context, speed, child) {
                      return Text(
                          '${rad2deg(speed.vw).toStringAsFixed(2)} deg/s');
                    },
                  ),
                ),
              ),
              // 电池电量显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: Icon(
                    const IconData(0xe995, fontFamily: "Battery"),
                    color: Colors.amber[300],
                  ),
                  label: ValueListenableBuilder<double>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: false)
                            .battery_,
                    builder: (context, battery, child) {
                      return Text('${battery.toStringAsFixed(0)} %');
                    },
                  ),
                ),
              ),
              // 导航状态显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: const Icon(
                    Icons.navigation,
                    color: Colors.green,
                    size: 16,
                  ),
                  label: ValueListenableBuilder<ActionStatus>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: true)
                            .navStatus_,
                    builder: (context, navStatus, child) {
                      return Text('${navStatus.toString()}');
                    },
                  ),
                ),
              ),
              // 诊断状态显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: Consumer<RosChannel>(
                  builder: (context, rosChannel, child) {
                    final diagnosticManager = rosChannel.diagnosticManager;
                    final statusCounts = diagnosticManager.getStatusCounts();
                    
                    int errorCount = statusCounts[DiagnosticStatus.ERROR] ?? 0;
                    int warnCount = statusCounts[DiagnosticStatus.WARN] ?? 0;
                    
                    Color chipColor = Colors.green;
                    IconData chipIcon = Icons.check_circle;
                    String chipText = AppLocalizations.of(context)!.diagnostic_normal;
                    
                    if (errorCount > 0) {
                      chipColor = Colors.red;
                      chipIcon = Icons.error;
                      chipText = AppLocalizations.of(context)!.error_count(errorCount.toString());
                    } else if (warnCount > 0) {
                      chipColor = Colors.orange;
                      chipIcon = Icons.warning;
                      chipText = AppLocalizations.of(context)!.warn_count(warnCount.toString());
                    }
                    
                    return  RawChip(
                          avatar: Icon(
                            chipIcon,
                            color: chipColor,
                            size: 16,
                          ),
                          label: Text(chipText),
                          backgroundColor: chipColor.withOpacity(0.1),
                          elevation: 0,
                          onPressed: () {
                             Navigator.push(
                            context,
                            MaterialPageRoute(
                              builder: (context) => const DiagnosticPage(),
                            ),
                          );
                        },

                    );
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildLeftToolbar(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 5,
      top: 60,
      child: ConstrainedBox(
        constraints: BoxConstraints(maxWidth: 280),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: [
          // 图层开关控制
          Card(
            elevation: 10,
            child: SingleChildScrollView(
              scrollDirection: Axis.horizontal,
              child: Row(
                children: [
                  IconButton(
                    icon: Icon(Icons.layers),
                    color:
                        showLayerControl ? Colors.green : theme.iconTheme.color,
                    onPressed: () {
                      setState(() {
                        showLayerControl = !showLayerControl;
                      });
                    },
                  ),
                  if (showLayerControl) ...[
                    // 使用循环生成图层控制按钮
                    ...Provider.of<GlobalState>(context, listen: true).layerNames.map((layerName) {
                      // 定义每个图层的图标和颜色配置
                      final l10n = AppLocalizations.of(context)!;
                      final layerConfig = <String, Map<String, dynamic>>{
                        'showGrid': {
                          'icon': Icons.grid_on,
                          'iconOff': Icons.grid_off,
                          'color': Colors.green,
                          'tooltip': l10n.layer_grid,
                        },
                        'showGlobalCostmap': {
                          'icon': Icons.map,
                          'iconOff': Icons.map_outlined,
                          'color': Colors.green,
                          'tooltip': l10n.layer_global_costmap,
                        },
                        'showLocalCostmap': {
                          'icon': Icons.map_outlined,
                          'iconOff': Icons.map_outlined,
                          'color': Colors.green,
                          'tooltip': l10n.layer_local_costmap,
                        },
                        'showLaser': {
                          'icon': Icons.radar,
                          'iconOff': Icons.radar_outlined,
                          'color': Colors.green,
                          'tooltip': l10n.layer_laser,
                        },
                        'showPointCloud': {
                          'icon': Icons.cloud,
                          'iconOff': Icons.cloud_outlined,
                          'color': Colors.green,
                          'tooltip': l10n.layer_pointcloud,
                        },
                        'showGlobalPath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.blue,
                          'tooltip': l10n.layer_global_path,
                        },
                        'showLocalPath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.green,
                          'tooltip': l10n.layer_local_path,
                        },
                        'showTracePath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.yellow,
                          'tooltip': l10n.layer_trace,
                        },
                        'showTopology': {
                          'icon': Icons.account_tree,
                          'iconOff': Icons.account_tree_outlined,
                          'color': Colors.orange,
                          'tooltip': l10n.layer_topology,
                        },
                        'showRobotFootprint': {
                          'icon': Icons.smart_toy,
                          'iconOff': Icons.smart_toy_outlined,
                          'color': Colors.blue,
                          'tooltip': l10n.layer_robot_footprint,
                        },
                      };
                      
                      final config = layerConfig[layerName];
                      if (config == null) return const SizedBox.shrink();
                      
                      return Tooltip(
                        message: config['tooltip'] as String,
                        child: ValueListenableBuilder<bool>(
                          valueListenable: Provider.of<GlobalState>(context, listen: true).getLayerState(layerName),
                          builder: (context, isVisible, child) {
                            return IconButton(
                              icon: Icon(
                                isVisible ? config['icon'] as IconData : config['iconOff'] as IconData,
                                size: 20,
                              ),
                              color: isVisible ? config['color'] as Color : Colors.grey,
                              onPressed: () {
                                Provider.of<GlobalState>(context, listen: false).toggleLayer(layerName);
                              },
                            );
                          },
                        ),
                      );
                    }).toList(),
                  ],
                ],
              ),
            ),
          ),

          ValueListenableBuilder(
            valueListenable: Provider.of<GlobalState>(context, listen: false).mode,
            builder: (context, mode, _) {
              return Card(
                elevation: 10,
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    IconButton(
                      onPressed: () {
                        var globalState =
                            Provider.of<GlobalState>(context, listen: false);
                        globalState.mode.value =
                            mode == Mode.reloc ? Mode.normal : Mode.reloc;
                      },
                      icon: Icon(
                        const IconData(0xe60f, fontFamily: "Reloc"),
                        color: mode == Mode.reloc
                            ? Colors.green
                            : theme.iconTheme.color,
                      ),
                    ),
                    if (mode == Mode.reloc) ...[
                      IconButton(
                        onPressed: () {
                          Provider.of<GlobalState>(context, listen: false)
                              .mode
                              .value = Mode.normal;
                          Provider.of<RosChannel>(context, listen: false)
                              .sendRelocPose(
                            _tileMapKey.currentState?.getRelocRobotPose() ??
                                RobotPose.zero(),
                          );
                        },
                        icon: Icon(Icons.check, color: Colors.green),
                      ),
                      IconButton(
                        onPressed: () {
                          Provider.of<GlobalState>(context, listen: false)
                              .mode
                              .value = Mode.normal;
                        },
                        icon: Icon(Icons.close, color: Colors.red),
                      ),
                    ],
                  ],
                ),
              );
            },
          ),
          
          const SizedBox(height: 8),
          
          // 显示相机图像
          Card(
            elevation: 10,
            child: IconButton(
              icon: Icon(Icons.camera_alt),
              color: showCamera ? Colors.green : theme.iconTheme.color,
              onPressed: () {
                setState(() {
                  showCamera = !showCamera;
                });
              },
              tooltip: AppLocalizations.of(context)!.camera_image,
            ),
          ),
          
          const SizedBox(height: 8),
          
          // 手动控制
          Card(
            elevation: 10,
            child: IconButton(
              icon: Icon(
                const IconData(0xea45, fontFamily: "GamePad"),
                color: Provider.of<GlobalState>(context, listen: false)
                        .isManualCtrl
                        .value
                    ? Colors.green
                    : theme.iconTheme.color,
              ),
              onPressed: () {
                if (Provider.of<GlobalState>(context, listen: false)
                    .isManualCtrl
                    .value) {
                  Provider.of<GlobalState>(context, listen: false)
                      .isManualCtrl
                      .value = false;
                  Provider.of<RosChannel>(context, listen: false)
                      .stopMunalCtrl();
                  setState(() {});
                } else {
                  Provider.of<GlobalState>(context, listen: false)
                      .isManualCtrl
                      .value = true;
                  Provider.of<RosChannel>(context, listen: false)
                      .startMunalCtrl();
                  setState(() {});
                }
              },
            ),
          ),
        ],
      ),
    ),
    );
  }

  void _showNavPointDialog(BuildContext context, NavPoint point) {
    final theme = Theme.of(context);
    showDialog(
      context: context,
      barrierDismissible: true,
      builder: (ctx) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Row(
          children: [
            Icon(Icons.location_on, color: Colors.blue[700], size: 24),
            const SizedBox(width: 10),
            Text(AppLocalizations.of(context)!.nav_point_info),
          ],
        ),
        content: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.blue[50],
                  borderRadius: BorderRadius.circular(10),
                  border: Border.all(color: Colors.blue[200]!),
                ),
                child: Row(
                  children: [
                    Icon(Icons.label, color: Colors.blue[700], size: 18),
                    const SizedBox(width: 10),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            point.name,
                            style: theme.textTheme.bodyLarge?.copyWith(
                              fontWeight: FontWeight.bold,
                              color: Colors.blue[800],
                            ),
                          ),
                          const SizedBox(height: 6),
                          Container(
                            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                            decoration: BoxDecoration(
                              color: _getTypeColor(point.type),
                              borderRadius: BorderRadius.circular(12),
                            ),
                            child: Text(
                              _getTypeText(point.type),
                              style: theme.textTheme.bodySmall?.copyWith(
                                color: Colors.white,
                                fontWeight: FontWeight.w600,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 16),
              _buildInfoSection(ctx, theme, AppLocalizations.of(context)!.position_coords, Icons.gps_fixed, [
                _buildInfoRow(AppLocalizations.of(context)!.coord_x, '${point.x.toStringAsFixed(2)} m'),
                _buildInfoRow(AppLocalizations.of(context)!.coord_y, '${point.y.toStringAsFixed(2)} m'),
                _buildInfoRow(AppLocalizations.of(context)!.heading, '${(point.theta * 180 / 3.14159).toStringAsFixed(1)}°'),
              ]),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(),
            child: Text(AppLocalizations.of(context)!.cancel),
          ),
          FilledButton.icon(
            onPressed: () {
              if (Provider.of<GlobalState>(context, listen: false).isManualCtrl.value) {
                toastification.show(
                  context: context,
                  title: Text(AppLocalizations.of(context)!.stop_manual_first),
                  autoCloseDuration: const Duration(seconds: 3),
                );
                return;
              }
              Provider.of<RosChannel>(context, listen: false).sendNavigationGoal(
                RobotPose(point.x, point.y, point.theta),
              );
              toastification.show(
                context: context,
                title: Text(AppLocalizations.of(context)!.nav_goal_sent(point.name)),
                autoCloseDuration: const Duration(seconds: 3),
              );
              Navigator.of(ctx).pop();
              setState(() => selectedNavPoint = null);
            },
            icon: const Icon(Icons.navigation, size: 20),
            label: Text(AppLocalizations.of(context)!.send_nav_goal),
            style: FilledButton.styleFrom(
              backgroundColor: Colors.blue[600],
              foregroundColor: Colors.white,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRightToolbar(BuildContext context, ThemeData theme) {
    return Positioned(
      right: 5,
      top: 30,
      child: Column(
            children: [
              // 地图编辑按钮
              Card(
                elevation: 10,
                child: IconButton(
                  icon: Icon(
                    Icons.edit_document,
                    color: (Provider.of<GlobalState>(context, listen: false)
                                .mode
                                .value ==
                            Mode.mapEdit)
                        ? Colors.orange
                        : theme.iconTheme.color,
                  ),
                  onPressed: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute(
                        builder: (context) => MapEditPage(
                          onExit: () {
                            _reloadData();
                          },
                        ),
                      ),
                    );
                  },
                  tooltip: AppLocalizations.of(context)!.map_edit,
                ),
              ),
              
              const SizedBox(height: 8),
              
              // 放大按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    _tileMapKey.currentState?.zoomIn();
                  },
                  icon: Icon(
                    Icons.zoom_in,
                    color: theme.iconTheme.color,
                  ),
                  tooltip: AppLocalizations.of(context)!.zoom_in,
                ),
              ),
              // 缩小按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    _tileMapKey.currentState?.zoomOut();
                  },
                  icon: Icon(
                    Icons.zoom_out,
                    color: theme.iconTheme.color,
                  ),
                  tooltip: AppLocalizations.of(context)!.zoom_out,
                ),
              ),
              // 定位到机器人按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    var globalState =
                        Provider.of<GlobalState>(context, listen: false);
                    if (globalState.mode.value == Mode.robotFixedCenter) {
                      globalState.mode.value = Mode.normal;
                    } else {
                      globalState.mode.value = Mode.robotFixedCenter;
                      _tileMapKey.currentState?.moveToRobot();
                    }
                    setState(() {});
                  },
                  icon: Icon(
                    Icons.location_searching,
                    color:
                        Provider.of<GlobalState>(context, listen: false).mode.value ==
                                Mode.robotFixedCenter
                            ? Colors.green
                            : theme.iconTheme.color,
                  ),
                ),
              ),
              // // 退出按钮
              // Card(
              //   elevation: 10,
              //   child: IconButton(
              //     onPressed: () {
              //       Navigator.push(context, MaterialPageRoute(builder: (context) => ConnectPage()));
              //     },
              //     icon: Icon(
              //       Icons.exit_to_app,
              //       color: theme.iconTheme.color,
              //     ),
              //     tooltip: '退出',
              //   ),
              // ),
            ],
          ),
    );
  }

  Widget _buildBottomControls(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 5,
      bottom: 10,
      child: Consumer<GlobalState>(
        builder: (context, globalState, child) {
          return Visibility(
            visible: !globalState.isManualCtrl.value,
            child: Row(
              children: [
                // 急停按钮
                Card(
                  color: Colors.red,
                  child: Container(
                    width: 100,
                    height: 50,
                    child: TextButton(
                      child: const Text(
                        "STOP",
                        style: TextStyle(color: Colors.white),
                      ),
                      onPressed: () {
                        Provider.of<RosChannel>(context, listen: false)
                            .sendEmergencyStop();
                        toastification.show(
                          context: context,
                          title: Text(AppLocalizations.of(context)!.emergency_stopped),
                          autoCloseDuration: const Duration(seconds: 3),
                        );
                      },
                    ),
                  ),
                ),
                // 停止导航按钮
                Consumer<RosChannel>(
                  builder: (context, rosChannel, child) {
                    return ValueListenableBuilder<ActionStatus>(
                      valueListenable: rosChannel.navStatus_,
                      builder: (context, navStatus, child) {
                        return Visibility(
                          visible: navStatus == ActionStatus.executing ||
                              navStatus == ActionStatus.accepted,
                          child: Card(
                            color: Colors.blue,
                            child: Container(
                              width: 50,
                              height: 50,
                              child: IconButton(
                                icon: const Icon(
                                  Icons.stop_circle,
                                  size: 30,
                                  color: Colors.white,
                                ),
                                onPressed: () {
                                  Provider.of<RosChannel>(context,
                                          listen: false)
                                      .sendCancelNav();
                                  toastification.show(
                                    context: context,
                                    title: Text(AppLocalizations.of(context)!.nav_stopped),
                                    autoCloseDuration: const Duration(seconds: 3),
                                  );
                                },
                              ),
                            ),
                          ),
                        );
                      },
                    );
                  },
                ),
              ],
            ),
          );
        },
      ),
    );
  }

  Widget _buildInfoSection(BuildContext context, ThemeData theme, String title, IconData icon, List<Widget> children) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Icon(icon, color: Colors.grey[600], size: 18),
            const SizedBox(width: 8),
            Text(
              title,
              style: theme.textTheme.titleSmall?.copyWith(
                fontWeight: FontWeight.w600,
                color: Colors.grey[700],
              ),
            ),
          ],
        ),
        const SizedBox(height: 12),
        Container(
          padding: const EdgeInsets.all(12),
          decoration: BoxDecoration(
            color: Colors.grey[50],
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: Colors.grey[200]!),
          ),
          child: Column(
            children: children,
          ),
        ),
      ],
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            label, 
            style: Theme.of(context).textTheme.bodyMedium?.copyWith(
              color: Colors.grey[600],
            ),
          ),
          Text(
            value, 
            style: Theme.of(context).textTheme.bodyMedium?.copyWith(
              fontWeight: FontWeight.w600,
              color: Colors.grey[800],
            ),
          ),
        ],
      ),
    );
  }


  Color _getTypeColor(NavPointType type) {
    switch (type) {
      case NavPointType.navGoal:
        return Colors.blue[600]!;
      case NavPointType.chargeStation:
        return Colors.green[600]!;
    }
  }

  String _getTypeText(NavPointType type) {
    switch (type) {
      case NavPointType.navGoal:
        return AppLocalizations.of(context)!.nav_goal;
      case NavPointType.chargeStation:
        return AppLocalizations.of(context)!.charge_station;
    }
  }
  
  // 构建相机显示组件
  Widget _buildCameraWidget(BuildContext context, ThemeData theme) {
    if (!showCamera) return const SizedBox.shrink();
    
    final screenSize = MediaQuery.of(context).size;
    
    return Positioned(
      left: camPosition.dx,
      top: camPosition.dy,
      child: GestureDetector(
        onPanUpdate: (details) {
          if (!isCamFullscreen) {
            setState(() {
              double newX = camPosition.dx + details.delta.dx;
              double newY = camPosition.dy + details.delta.dy;
              // 限制位置在屏幕范围内
              newX = newX.clamp(0.0, screenSize.width - camWidgetWidth);
              newY = newY.clamp(0.0, screenSize.height - camWidgetHeight);
              camPosition = Offset(newX, newY);
            });
          }
        },
        child: Container(
          child: Stack(
            children: [
              LayoutBuilder(
                builder: (context, constraints) {
                  // 在非全屏状态下，获取屏幕宽高
                  double containerWidth = isCamFullscreen
                      ? screenSize.width
                      : camWidgetWidth;
                  double containerHeight = isCamFullscreen
                      ? screenSize.height
                      : camWidgetHeight;

                  return Image.network(
                    'http://${globalSetting.robotIp}:${globalSetting.imagePort}/stream?topic=${globalSetting.imageTopic}',
                    width: containerWidth,
                    height: containerHeight,
                    fit: BoxFit.fill,
                    errorBuilder: (_, __, ___) => Container(
                      width: containerWidth,
                      height: containerHeight,
                      color: Colors.grey[300],
                      child: Icon(Icons.videocam_off, size: 48, color: Colors.grey[600]),
                    ),
                  );
                },
              ),
              Positioned(
                right: 0,
                top: 0,
                child: IconButton(
                  icon: Icon(
                    isCamFullscreen
                        ? Icons.fullscreen_exit
                        : Icons.fullscreen,
                    color: Colors.black,
                  ),
                  constraints: BoxConstraints(), // 移除按钮的默认大小约束，变得更加紧凑
                  onPressed: () {
                    setState(() {
                      isCamFullscreen = !isCamFullscreen;
                      if (isCamFullscreen) {
                        // 进入全屏时，保存当前位置，并将位置设为 (0, 0)
                        camPreviousPosition = camPosition;
                        camPosition = Offset(0, 0);
                      } else {
                        // 退出全屏时，恢复之前的位置
                        camPosition = camPreviousPosition;
                      }
                    });
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
  
  Widget _buildGamepadWidget(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 0,
      right: 0,
      bottom: 0,
      child: Center(
        child: SizedBox(
          width: 360,
          height: 150,
          child: GamepadWidget(),
        ),
      ),
    );
  }

  // 构建地图图例组件
  Widget _buildMapLegend(BuildContext context, ThemeData theme) {
    return Positioned(
      right: 10,
      top: 2,
      child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              // 图例项目 - 横着排列
              _buildCompactLegendItem(AppLocalizations.of(context)!.legend_free, _getFreeAreaColor()),
              const SizedBox(width: 12),
              _buildCompactLegendItem(AppLocalizations.of(context)!.legend_occupied, _getOccupiedAreaColor()),
              const SizedBox(width: 12),
              _buildCompactLegendItem(AppLocalizations.of(context)!.legend_unknown, _getUnknownAreaColor()),
            ],
          ),
        ),
      
    );
  }

  // 构建精简图例项目
  Widget _buildCompactLegendItem(String label, Color color) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        // 颜色指示器
        Container(
          width: 10,
          height: 10,
          decoration: BoxDecoration(
            color: color,
            borderRadius: BorderRadius.circular(2),
            border: Border.all(
              color: Colors.grey[400]!,
              width: 0.5,
            ),
          ),
        ),
        const SizedBox(width: 4),
        // 标签
        Text(
          label,
          style: TextStyle(
            fontSize: 7,
            fontWeight: FontWeight.w500,
            color: Colors.grey[700],
          ),
        ),
      ],
    );
  }

  Color _getFreeAreaColor() => const Color(0xFFFFFFFF);

  Color _getOccupiedAreaColor() => const Color(0xFF3C3C3C);

  Color _getUnknownAreaColor() => const Color(0xFFC8C8C8);

  @override
  void dispose() {
    super.dispose();
  }
}




