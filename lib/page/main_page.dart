import 'package:flutter/material.dart';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flutter/gestures.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/page/main_flame.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/page/map_edit_page.dart';
import 'package:ros_flutter_gui_app/provider/nav_point_manager.dart';
import 'package:ros_flutter_gui_app/provider/them_provider.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:toastification/toastification.dart';
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/page/gamepad_widget.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_status.dart';
import 'package:ros_flutter_gui_app/page/diagnostic_page.dart';
import 'package:ros_flutter_gui_app/provider/diagnostic_manager.dart';



class MainFlamePage extends StatefulWidget {
  @override
  _MainFlamePageState createState() => _MainFlamePageState();
}

class _MainFlamePageState extends State<MainFlamePage> {
  late MainFlame game;
  bool showLayerControl = false;
  bool showCamera = false;
  NavPoint? selectedNavPoint;
  
  // 相机相关变量
  Offset camPosition = Offset(30, 10); // 初始位置
  bool isCamFullscreen = false; // 是否全屏
  Offset camPreviousPosition = Offset(30, 10); // 保存进入全屏前的位置
  late double camWidgetWidth;
  late double camWidgetHeight;

  @override
  void initState() {
    super.initState();
    final rosChannel = Provider.of<RosChannel>(context, listen: false);
    final globalState = Provider.of<GlobalState>(context, listen: false);
    final navPointManager = Provider.of<NavPointManager>(context, listen: false);
    final themeProvider = Provider.of<ThemeProvider>(context, listen: false);
    game = MainFlame(
      rosChannel: rosChannel, 
      themeProvider: themeProvider,
      globalState: globalState,
      navPointManager: navPointManager,
    );
    game.onNavPointTap = (NavPoint? point) {
      setState(() {
        selectedNavPoint = point;
      });
    };
    
    // 加载图层设置
    Provider.of<GlobalState>(context, listen: false).loadLayerSettings();
    
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
  
  // 重新加载导航点和地图数据
  Future<void> _reloadData() async {
    await game.reloadNavPointsAndMap();
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
        levelText = '警告';
        levelColor = Colors.orange;
        toastType = ToastificationType.warning;
        iconData = Icons.warning;
        break;
      case DiagnosticStatus.ERROR:
        levelText = '错误';
        levelColor = Colors.red;
        toastType = ToastificationType.error;
        iconData = Icons.error;
        break;
      case DiagnosticStatus.STALE:
        levelText = '失活';
        levelColor = Colors.grey;
        toastType = ToastificationType.info;
        iconData = Icons.schedule;
        break;
      default:
        return; // 其他状态不显示toast
    }
    
    toastification.show(
      context: context,
      type: toastType,
      title: Text('健康诊断:[$levelText] $componentName'),
      description: Text('硬件ID: $hardwareId\n消息: ${state.message}'),
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
              // 游戏画布
              Listener(
                onPointerSignal: (pointerSignal) {
                  if (pointerSignal is PointerScrollEvent) {
                    final position = Vector2(pointerSignal.position.dx, pointerSignal.position.dy);
                    game.onScroll(pointerSignal.scrollDelta.dy, position);
                  }
                },
                child: GestureDetector(
                  onScaleStart: (details) {
                    final position = Vector2(details.localFocalPoint.dx, details.localFocalPoint.dy);
                    game.onScaleStart(position);
                  },
                  onScaleUpdate: (details) {
                    final position = Vector2(details.localFocalPoint.dx, details.localFocalPoint.dy);
                    game.onScaleUpdate(details.scale, position);
                  },
                  onScaleEnd: (details) {
                    game.onScaleEnd();
                  },
                  onTapDown: (details) {
                    // 处理点击事件，检测waypoint
                    game.onTap(details.localPosition);
                  },
                  child: GameWidget(game: game),
                ),
              ),
              _buildTopMenuBar(context, theme),
              _buildLeftToolbar(context, theme),
              _buildRightToolbar(context, theme),
              _buildBottomControls(context, theme),
              _buildCameraWidget(context, theme),
              _buildGamepadWidget(context, theme),
              _buildMapLegend(context, theme),
            ],
          ),
        );
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
                    String chipText = '正常';
                    
                    if (errorCount > 0) {
                      chipColor = Colors.red;
                      chipIcon = Icons.error;
                      chipText = '错误: $errorCount';
                    } else if (warnCount > 0) {
                      chipColor = Colors.orange;
                      chipIcon = Icons.warning;
                      chipText = '警告: $warnCount';
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
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // 图层开关控制
          Card(
            elevation: 10,
            child: Container(
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
                      final layerConfig = <String, Map<String, dynamic>>{
                        'showGrid': {
                          'icon': Icons.grid_on,
                          'iconOff': Icons.grid_off,
                          'color': Colors.green,
                          'tooltip': '网格图层',
                        },
                        'showGlobalCostmap': {
                          'icon': Icons.map,
                          'iconOff': Icons.map_outlined,
                          'color': Colors.green,
                          'tooltip': '全局代价地图',
                        },
                        'showLocalCostmap': {
                          'icon': Icons.map_outlined,
                          'iconOff': Icons.map_outlined,
                          'color': Colors.green,
                          'tooltip': '局部代价地图',
                        },
                        'showLaser': {
                          'icon': Icons.radar,
                          'iconOff': Icons.radar_outlined,
                          'color': Colors.green,
                          'tooltip': '激光雷达数据',
                        },
                        'showPointCloud': {
                          'icon': Icons.cloud,
                          'iconOff': Icons.cloud_outlined,
                          'color': Colors.green,
                          'tooltip': '点云数据',
                        },
                        'showGlobalPath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.blue,
                          'tooltip': '全局路径',
                        },
                        'showLocalPath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.green,
                          'tooltip': '局部路径',
                        },
                        'showTracePath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.yellow,
                          'tooltip': '轨迹路径',
                        },
                        'showTopology': {
                          'icon': Icons.account_tree,
                          'iconOff': Icons.account_tree_outlined,
                          'color': Colors.orange,
                          'tooltip': '拓扑地图',
                        },
                        'showRobotFootprint': {
                          'icon': Icons.smart_toy,
                          'iconOff': Icons.smart_toy_outlined,
                          'color': Colors.blue,
                          'tooltip': '机器人轮廓',
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

          // 重定位工具
          Card(
            elevation: 10,
            child: Container(
              child: Row(
                children: [
                  IconButton(
                    onPressed: () {
                      var globalState =
                          Provider.of<GlobalState>(context, listen: false);
                      if (globalState.mode.value == Mode.reloc) {
                        globalState.mode.value = Mode.normal;
                      } else {
                        globalState.mode.value = Mode.reloc;
                      }
                      game.setRelocMode(globalState.mode.value == Mode.reloc);
                      setState(() {});
                    },
                    icon: Icon(
                      const IconData(0xe60f, fontFamily: "Reloc"),
                      color: Provider.of<GlobalState>(context, listen: false)
                                  .mode
                                  .value ==
                              Mode.reloc
                          ? Colors.green
                          : theme.iconTheme.color,
                    ),
                  ),
                  if (Provider.of<GlobalState>(context, listen: false)
                          .mode
                          .value ==
                      Mode.reloc) ...[
                    IconButton(
                      onPressed: () {
                        // 确认重定位逻辑
                        Provider.of<GlobalState>(context, listen: false)
                            .mode
                            .value = Mode.normal;
                        game.setRelocMode(false);
                        Provider.of<RosChannel>(context, listen: false).sendRelocPose(game.getRelocRobotPose());
                        setState(() {});
                      },
                      icon: Icon(Icons.check, color: Colors.green),
                    ),
                    IconButton(
                      onPressed: () {
                        // 取消重定位逻辑
                        Provider.of<GlobalState>(context, listen: false)
                            .mode
                            .value = Mode.normal;
                        game.setRelocMode(false);
                        setState(() {});
                      },
                      icon: Icon(Icons.close, color: Colors.red),
                    ),
                  ],
                ],
              ),
            ),
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
              tooltip: '相机图像',
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
    );
  }

  Widget _buildRightToolbar(BuildContext context, ThemeData theme) {
    return Positioned(
      right: 5,
      top: 30,
      child: Row(
        children: [
          // 右侧信息面板
          if (game.showInfoPanel && selectedNavPoint != null)
            Container(
              width: 300, // 固定宽度，不占满右侧
              child: Card(
                elevation: 16,
                shadowColor: Colors.black.withOpacity(0.3),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(16),
                ),
                child: Container(
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(16),
                    gradient: LinearGradient(
                      begin: Alignment.topLeft,
                      end: Alignment.bottomRight,
                      colors: [
                        Colors.white,
                        Colors.blue[50]!,
                      ],
                    ),
                  ),
                  child: Padding(
                    padding: const EdgeInsets.all(20.0), // 减少内边距
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      mainAxisSize: MainAxisSize.min, // 根据内容自适应高度
                      children: [
                        // 标题栏
                        Row(
                          mainAxisAlignment: MainAxisAlignment.spaceBetween,
                          children: [
                            Row(
                              children: [
                                Container(
                                  padding: const EdgeInsets.all(6), // 减少图标容器大小
                                  decoration: BoxDecoration(
                                    color: Colors.blue[100],
                                    borderRadius: BorderRadius.circular(10),
                                  ),
                                  child: Icon(
                                    Icons.location_on,
                                    color: Colors.blue[700],
                                    size: 20, // 减少图标大小
                                  ),
                                ),
                                const SizedBox(width: 10),
                                Text(
                                  '导航点信息',
                                  style: theme.textTheme.titleMedium?.copyWith( // 使用较小的标题样式
                                    fontWeight: FontWeight.bold,
                                    color: Colors.grey[800],
                                  ),
                                ),
                              ],
                            ),
                            // 添加关闭按钮
                            Container(
                              decoration: BoxDecoration(
                                color: Colors.grey[100],
                                borderRadius: BorderRadius.circular(16),
                              ),
                              child: IconButton(
                                icon: const Icon(Icons.close, size: 18),
                                onPressed: () {
                                  game.hideInfoPanel();
                                  setState(() {
                                    selectedNavPoint = null;
                                  });
                                },
                                tooltip: '关闭',
                                style: IconButton.styleFrom(
                                  padding: const EdgeInsets.all(6),
                                ),
                              ),
                            ),
                          ],
                        ),
                        Divider(height: 20, thickness: 1, color: Colors.grey[300]), // 减少分隔线高度
                        
                        // 导航点名称和类型
                        Container(
                          padding: const EdgeInsets.all(12), // 减少内边距
                          decoration: BoxDecoration(
                            color: Colors.blue[50],
                            borderRadius: BorderRadius.circular(10),
                            border: Border.all(color: Colors.blue[200]!, width: 1.5),
                            boxShadow: [
                              BoxShadow(
                                color: Colors.blue[100]!.withOpacity(0.3),
                                blurRadius: 6,
                                offset: Offset(0, 2),
                              ),
                            ],
                          ),
                          child: Row(
                            children: [
                              Container(
                                padding: const EdgeInsets.all(6),
                                decoration: BoxDecoration(
                                  color: Colors.blue[100],
                                  borderRadius: BorderRadius.circular(6),
                                ),
                                child: Icon(
                                  Icons.label,
                                  color: Colors.blue[700],
                                  size: 16, // 减少图标大小
                                ),
                              ),
                              const SizedBox(width: 10),
                              Expanded(
                                child: Column(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Text(
                                      selectedNavPoint!.name,
                                      style: theme.textTheme.bodyLarge?.copyWith( // 使用较小的文本样式
                                        fontWeight: FontWeight.bold,
                                        color: Colors.blue[800],
                                      ),
                                    ),
                                    const SizedBox(height: 6),
                                    Container(
                                      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                                      decoration: BoxDecoration(
                                        color: _getTypeColor(selectedNavPoint!.type),
                                        borderRadius: BorderRadius.circular(12),
                                        boxShadow: [
                                          BoxShadow(
                                            color: _getTypeColor(selectedNavPoint!.type).withOpacity(0.3),
                                            blurRadius: 3,
                                            offset: Offset(0, 1),
                                          ),
                                        ],
                                      ),
                                      child: Text(
                                        _getTypeText(selectedNavPoint!.type),
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
                        
                        const SizedBox(height: 16), // 减少间距
                        
                        // 坐标信息
                        _buildInfoSection(
                          context,
                          theme,
                          '位置坐标',
                          Icons.gps_fixed,
                          [
                            _buildInfoRow('X坐标', '${selectedNavPoint!.x.toStringAsFixed(2)} m'),
                            _buildInfoRow('Y坐标', '${selectedNavPoint!.y.toStringAsFixed(2)} m'),
                            _buildInfoRow('方向', '${(selectedNavPoint!.theta * 180 / 3.14159).toStringAsFixed(1)}°'),
                          ],
                        ),
                        
                        const SizedBox(height: 16), // 减少间距
                        
                        // 导航按钮
                        Container(
                          width: double.infinity,
                          decoration: BoxDecoration(
                            borderRadius: BorderRadius.circular(10),
                            boxShadow: [
                              BoxShadow(
                                color: Colors.blue[400]!.withOpacity(0.3),
                                blurRadius: 6,
                                offset: Offset(0, 3),
                              ),
                            ],
                          ),
                          child: ElevatedButton.icon(
                            onPressed: () async {
                              if(Provider.of<GlobalState>(context, listen: false).isManualCtrl.value){
                                toastification.show(
                                  context: context,
                                  title: Text('请先停止手动控制'),
                                  autoCloseDuration: const Duration(seconds: 3),
                                );
                                return;
                              }
                              
                              // 使用RosChannel发送导航目标
                              Provider.of<RosChannel>(context, listen: false).sendNavigationGoal(
                                RobotPose(
                                  selectedNavPoint!.x, 
                                  selectedNavPoint!.y, 
                                  selectedNavPoint!.theta
                                )
                              );
                              
                              // 使用fluttertoast显示成功消息
                              toastification.show(
                                context: context,
                                title: Text('已发送导航目标到 ${selectedNavPoint!.name}'),
                                autoCloseDuration: const Duration(seconds: 3),
                              );
                              
                              // 发送导航目标后自动关闭信息面板
                              game.hideInfoPanel();
                              setState(() {
                                selectedNavPoint = null;
                              });
                            },
                            icon: const Icon(Icons.navigation, size: 20),
                            label: const Text(
                              '发送导航目标',
                              style: TextStyle(fontSize: 15, fontWeight: FontWeight.w600), // 减少按钮文字大小
                            ),
                            style: ElevatedButton.styleFrom(
                              backgroundColor: Colors.blue[600],
                              foregroundColor: Colors.white,
                              padding: const EdgeInsets.symmetric(vertical: 14), // 减少按钮高度
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(10),
                              ),
                              elevation: 0,
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            ),
          
          // 原有的工具栏按钮
          Column(
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
                              // 地图编辑界面退出时，重新加载数据
                              _reloadData();
                            },
                          ),
                        ),
                      );
                    setState(() {});
                  },
                  tooltip: '地图编辑',
                ),
              ),
              
              const SizedBox(height: 8),
              
              // 放大按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    game.zoomIn();
                  },
                  icon: Icon(
                    Icons.zoom_in,
                    color: theme.iconTheme.color,
                  ),
                  tooltip: '放大',
                ),
              ),
              // 缩小按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    game.zoomOut();
                  },
                  icon: Icon(
                    Icons.zoom_out,
                    color: theme.iconTheme.color,
                  ),
                  tooltip: '缩小',
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
                      game.centerOnRobot(false);
                    } else {
                      globalState.mode.value = Mode.robotFixedCenter;
                      game.centerOnRobot(true);
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
                             // 移除这里的GamepadWidget，我们将把它移到屏幕底部
            ],
          ),
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
                          title: Text('急停已触发'),
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
                                    title: Text('导航已停止'),
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
        return '导航目标';
      case NavPointType.chargeStation:
        return '充电站';
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

                  return Mjpeg(
                    stream: 'http://${globalSetting.robotIp}:${globalSetting.imagePort}/stream?topic=${globalSetting.imageTopic}',
                    isLive: true,
                    width: containerWidth,
                    height: containerHeight,
                    fit: BoxFit.fill,
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
  
  // 构建游戏手柄组件
  Widget _buildGamepadWidget(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 0,
      right: 0,
      bottom: 0,
      child: Container(
        height: 150, // 给GamepadWidget一个明确的高度
        child: GamepadWidget(),
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
              _buildCompactLegendItem('自由', _getFreeAreaColor()),
              const SizedBox(width: 12),
              _buildCompactLegendItem('障碍', _getOccupiedAreaColor()),
              const SizedBox(width: 12),
              _buildCompactLegendItem('未知', _getUnknownAreaColor()),
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

  // 获取自由区域颜色 - 与地图渲染中的RGB值完全匹配
  Color _getFreeAreaColor() {
    final isDarkMode = Theme.of(context).brightness == Brightness.dark;
    return isDarkMode ? const Color(0xFF1E1E1E) : const Color(0xFFFFFFFF);
  }

  // 获取占据区域颜色 - 与地图渲染中的RGB值完全匹配
  Color _getOccupiedAreaColor() {
    final isDarkMode = Theme.of(context).brightness == Brightness.dark;
    return isDarkMode ? const Color(0xFFC8C8C8) : const Color(0xFF3C3C3C);
  }

  // 获取未知区域颜色 - 与地图渲染中的RGB值完全匹配
  Color _getUnknownAreaColor() {
    final isDarkMode = Theme.of(context).brightness == Brightness.dark;
    return isDarkMode ? const Color(0xFF505050) : const Color(0xFFC8C8C8);
  }

  @override
  void dispose() {
    super.dispose();
  }
}




