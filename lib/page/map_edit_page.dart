import 'package:flutter/material.dart';
import 'package:flutter/gestures.dart';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/page/map_edit_flame.dart';
import 'package:ros_flutter_gui_app/provider/nav_point_manager.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/display/waypoint.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:toastification/toastification.dart';
import 'dart:math';
import 'package:vector_math/vector_math_64.dart' as vm;

class MapEditPage extends StatefulWidget {
  final VoidCallback? onExit;
  
  const MapEditPage({super.key, this.onExit});

  @override
  State<MapEditPage> createState() => _MapEditPageState();
}

class _MapEditPageState extends State<MapEditPage> {
  late MapEditFlame game;
  late GlobalState globalState;
  late RosChannel rosChannel;

  OccupancyMap? occupancyMap;
  
  // 当前选中的编辑工具
  String? selectedTool;
  
  // 导航点列表
  List<NavPoint> navPoints = [];
  
  // 当前选中的点位信息
  NavPoint? selectedWayPointInfo;


  @override
  void initState() {
    super.initState();
    globalState = Provider.of<GlobalState>(context, listen: false);
    rosChannel = Provider.of<RosChannel>(context, listen: false);
    
    // 设置地图编辑模式
    globalState.mode.value = Mode.mapEdit;
 
    // 监听地图数据
    rosChannel.map_.addListener(() {
      occupancyMap=rosChannel.map_.value;
    });
      
    // 立即更新地图数据
    occupancyMap=rosChannel.map_.value;
    

    
    // 创建专门的地图编辑Flame组件，传入回调函数
    game = MapEditFlame(
      rosChannel: rosChannel,
      onAddNavPoint: (x, y) async {
        final wayPointInfo = await _addNavPoint(x, y);
        return wayPointInfo;
      },
      onWayPointSelectionChanged: _onWayPointSelectionChanged,
    );
    // 拖拽/旋转实时回调，刷新右侧信息
    game.currentSelectPointUpdate = () {
      setState(() {
        final info = game.getSelectedWayPointInfo();
        selectedWayPointInfo = info;
      });
    };
    
    // 加载导航点
    _loadNavPoints();
  }
  
  @override
  void dispose() {
    // 退出地图编辑模式时，重置为正常模式
    globalState.mode.value = Mode.normal;
    super.dispose();
  }

  // 导航点选择状态变化回调
  void _onWayPointSelectionChanged() {
    setState(() {
      final info = game.getSelectedWayPointInfo();
      selectedWayPointInfo = info;
    });
  }
  
  // 加载导航点
  Future<void> _loadNavPoints() async {
    final navPointManager = Provider.of<NavPointManager>(context, listen: false);
    navPoints = await navPointManager.loadNavPoints();
    for(var navPoint in navPoints){
      vm.Vector2 occPose = occupancyMap!.xy2idx(vm.Vector2(navPoint.x, navPoint.y));
      game.addWayPoint(navPoint,occPose.x,occPose.y,navPoint.theta);
    }
    setState(() {
    });
  }
  
  // 添加导航点
  Future<NavPoint?> _addNavPoint(double x, double y) async {
    final name = await _showAddNavPointDialog(x, y);
    if (name != null && name.isNotEmpty) {
      // 用户输入了名称并点击确定，创建导航点
      final navPointManager = Provider.of<NavPointManager>(context, listen: false);
      final navPoint = await navPointManager.addNavPoint(x, y, 0.0, name);
      return navPoint;
    } else {
      // 用户点击取消或没有输入名称
      return null;
    }
  }
  
  // 显示添加导航点对话框
  Future<String?> _showAddNavPointDialog(double x, double y) async {
    final TextEditingController nameController = TextEditingController();
    final navPointManager = Provider.of<NavPointManager>(context, listen: false);
    int id = await navPointManager.getNextId();
    nameController.text = 'POINT_${id.toString()}';
    
    return showDialog<String>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Text('添加导航点'),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text('位置: (${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)})'),
              const SizedBox(height: 16),
              TextField(
                controller: nameController,
                decoration: const InputDecoration(
                  labelText: '导航点名称',
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
              onPressed: () => Navigator.of(context).pop(nameController.text),
              child: const Text('确定'),
            ),
          ],
        );
      },
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
              onTapDown: (details) async {
                // 使用局部坐标，避免受栈内其他控件偏移影响
                final position = Vector2(details.localPosition.dx, details.localPosition.dy);
                await game.onTapDown(position);
              },
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
              child: MouseRegion(
                cursor: (selectedTool == 'addNavPoint')
                    ? SystemMouseCursors.precise
                    : SystemMouseCursors.basic,
                child: GameWidget(game: game),
              ),
            ),
          ),
          
          // 顶部工具栏
          Positioned(
            top: 0,
            left: 0,
            right: 0,
            child: _buildTopToolbar(context, theme),
          ),
          
          // 左侧编辑工具栏
          Positioned(
            left: 10,
            top: 100,
            child: _buildEditToolbar(context, theme),
          ),
          
          // 右侧信息面板
          Positioned(
            right: 10,
            top: 100,
            child: _buildInfoPanel(context, theme),
          ),
        ],
      ),
    );
  }

  Widget _buildTopToolbar(BuildContext context, ThemeData theme) {
    return Container(
      height: 60,
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
          // 左侧工具栏按钮
          Expanded(
            child: Row(
              children: [
                // 打开文件按钮
                IconButton(
                  icon: const Icon(Icons.folder_open, color: Colors.white, size: 28),
                  onPressed: () {
                    // TODO: 实现打开文件功能
                    print('打开文件');
                  },
                  tooltip: '打开文件',
                ),
                
                const SizedBox(width: 16),
                
                // 保存按钮
                IconButton(
                  icon: const Icon(Icons.save, color: Colors.white, size: 28),
                  onPressed: () async {
                    List<NavPoint> navPoints = game.getAllWayPoint();
                    final navPointManager = Provider.of<NavPointManager>(context, listen: false);
                    await navPointManager.saveNavPoints(navPoints);
                    
                    // 显示保存成功提示
                    if (mounted) {
                      toastification.show(
                        context: context,
                        type: ToastificationType.success,
                        style: ToastificationStyle.flatColored,
                        title:const Text('保存成功！'),
                        autoCloseDuration: const Duration(seconds: 2),
                      );
                    }
                  },
                  tooltip: '保存',
                ),
                
                const SizedBox(width: 16),
                
                // 撤销按钮
                IconButton(
                  icon: const Icon(Icons.undo, color: Colors.white, size: 28),
                  onPressed: () {
                    // TODO: 实现撤销功能
                    print('撤销');
                  },
                  tooltip: '撤销',
                ),
                
                const SizedBox(width: 16),
                
                // 重做按钮
                IconButton(
                  icon: const Icon(Icons.redo, color: Colors.white, size: 28),
                  onPressed: () {
                    // TODO: 实现重做功能
                    print('重做');
                  },
                  tooltip: '重做',
                ),
              ],
            ),
          ),
          
          // 中间标题
          Expanded(
            child: Center(
              child: Text(
                '地图编辑',
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ),
          
          // 右侧退出按钮
          Expanded(
            child: Align(
              alignment: Alignment.centerRight,
              child: Padding(
                padding: const EdgeInsets.only(right: 20),
                child: IconButton(
                  icon: const Icon(Icons.close, color: Colors.white, size: 28),
                  onPressed: () async {
                    // 退出前保存当前状态
                    List<NavPoint> navPoints = game.getAllWayPoint();
                    final navPointManager = Provider.of<NavPointManager>(context, listen: false);
                    await navPointManager.saveNavPoints(navPoints);
                    
                    // 调用退出回调
                    widget.onExit?.call();
                    
                    // 退出地图编辑模式
                    Navigator.pop(context);
                  },
                  tooltip: '退出地图编辑模式',
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEditToolbar(BuildContext context, ThemeData theme) {
    return Card(
      elevation: 8,
      child: Container(
        padding: const EdgeInsets.all(8),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            // 导航点添加工具
            _buildEditTool(
              icon: Icons.add_location,
              label: '导航点编辑',
              toolName: 'addNavPoint',
              color: Colors.blue,
            ),
            
            const SizedBox(height: 8),
            
            // 障碍物绘制工具
            _buildEditTool(
              icon: Icons.brush,
              label: '绘制障碍物',
              toolName: 'drawObstacle',
              color: Colors.red,
            ),
            
            const SizedBox(height: 8),
            
            // 障碍物擦除工具
            _buildEditTool(
              icon: Icons.auto_fix_high,
              label: '擦除障碍物',
              toolName: 'eraseObstacle',
              color: Colors.green,
            ),
            
            const SizedBox(height: 8),
            

          ],
        ),
      ),
    );
  }
  


  Widget _buildEditTool({
    required IconData icon,
    required String label,
    required String toolName,
    required Color color,
  }) {
    final isActive = selectedTool == toolName;
    
    return Container(
      width: 120,
      child: Column(
        children: [
          IconButton(
            icon: Icon(icon, size: 24),
            color: isActive ? color : Colors.grey,
            onPressed: () {
               if (isActive) {
                selectedTool = null; // 取消选择
                game.setSelectedTool(null);
                // 退出所有点位编辑模式
                for (final wp in game.wayPoints) {
                  wp.setSelected(false);
                  wp.setEditMode(false);
                }
              } else {
                selectedTool = toolName; // 选择工具
                game.setSelectedTool(toolName);
                // 切换到其他工具时退出所有点位编辑模式
                if (toolName != 'addNavPoint') {
                  for (final wp in game.wayPoints) {
                    wp.setSelected(false);
                    wp.setEditMode(false);
                  }
                }
              }
              setState(() {});
            },
          ),
          Text(
            label,
            style: TextStyle(
              fontSize: 12,
              color: isActive ? color : Colors.grey,
              fontWeight: isActive ? FontWeight.bold : FontWeight.normal,
            ),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  Widget _buildInfoPanel(BuildContext context, ThemeData theme) {
    return Card(
      elevation: 8,
      child: Container(
        width: 200,
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: [
            // 如果有选中的导航点，显示导航点信息
            if (selectedWayPointInfo != null) ...[
              _buildWayPointInfo(theme),
              const SizedBox(height: 16),
            ] else ...[
              // 否则显示编辑模式说明
              Text(
                '编辑模式说明',
                style: theme.textTheme.titleMedium?.copyWith(
                  fontWeight: FontWeight.bold,
                ),
              ),
              const SizedBox(height: 16),
              
              _buildInstructionItem(
                icon: Icons.add_location,
                title: '添加导航点',
                description: '选择工具后双击地图添加导航点',
                color: Colors.blue,
              ),
              
              const SizedBox(height: 12),
              
              _buildInstructionItem(
                icon: Icons.brush,
                title: '绘制障碍物',
                description: '拖拽鼠标绘制障碍物区域',
                color: Colors.red,
              ),
              
              const SizedBox(height: 12),
              
              _buildInstructionItem(
                icon: Icons.auto_fix_high,
                title: '擦除障碍物',
                description: '拖拽鼠标擦除障碍物区域',
                color: Colors.green,
              ),
              
              const SizedBox(height: 12),
              
              _buildInstructionItem(
                icon: Icons.touch_app,
                title: '导航点操作',
                description: '单击选中，拖拽移动，拖拽红点旋转，删除按钮删除选中',
                color: Colors.orange,
              ),
              
              const SizedBox(height: 16),
            ],
            
            // 导航点计数显示
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.blue.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue.withOpacity(0.3)),
              ),
              child: Row(
                children: [
                  Icon(Icons.add_location, color: Colors.blue, size: 16),
                  const SizedBox(width: 8),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          '导航点数量',
                          style: TextStyle(
                            fontWeight: FontWeight.bold,
                            color: Colors.blue,
                          ),
                        ),
                        Text(
                          '${game.wayPointCount} 个',
                          style: const TextStyle(fontSize: 12),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
            
            Container(
              padding: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: Colors.orange.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.orange.withOpacity(0.3)),
              ),
              child: Row(
                children: [
                  Icon(Icons.info, color: Colors.orange, size: 16),
                  const SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      '提示：使用鼠标滚轮缩放，拖拽移动地图',
                      style: TextStyle(
                        fontSize: 12,
                        color: Colors.orange.shade700,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
  
  // 构建导航点信息显示
  Widget _buildWayPointInfo(ThemeData theme) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.blue.withOpacity(0.1),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.blue.withOpacity(0.5)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.location_on, color: Colors.blue, size: 20),
              const SizedBox(width: 8),
              Text(
                '导航点信息',
                style: theme.textTheme.titleMedium?.copyWith(
                  fontWeight: FontWeight.bold,
                  color: Colors.blue,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          
          _buildInfoRow('名称', selectedWayPointInfo!.name),
          _buildInfoRow('X坐标', '${selectedWayPointInfo!.x.toStringAsFixed(2)} m'),
          _buildInfoRow('Y坐标', '${selectedWayPointInfo!.y.toStringAsFixed(2)} m'),
          _buildInfoRow('方向', '${(selectedWayPointInfo!.theta * 180 / 3.14159).toStringAsFixed(1)}°'),
          
          const SizedBox(height: 12),
          
          // 操作区：删除选中
          Row(
            children: [
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () {
                    String name = game.deleteSelectedWayPoint();
                    final navPointManager = Provider.of<NavPointManager>(context, listen: false);
                    navPointManager.removeNavPoint(name);
                    setState(() {
                      selectedWayPointInfo = null;
                    });
                  },
                  icon: const Icon(Icons.delete, size: 16),
                  label: const Text('删除'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red,
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(vertical: 10),
                  ),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
  
  // 构建信息行
  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          SizedBox(
            width: 50,
            child: Text(
              '$label:',
              style: const TextStyle(
                fontWeight: FontWeight.bold,
                fontSize: 12,
              ),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: const TextStyle(fontSize: 12),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildInstructionItem({
    required IconData icon,
    required String title,
    required String description,
    required Color color,
  }) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Icon(icon, color: color, size: 16),
        const SizedBox(width: 8),
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                title,
                style: TextStyle(
                  fontWeight: FontWeight.bold,
                  color: color,
                ),
              ),
              Text(
                description,
                style: const TextStyle(fontSize: 12),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
