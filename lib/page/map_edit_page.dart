import 'package:flutter/material.dart';
import 'package:flame/game.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/page/main_page.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/page/main_flame.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/provider/nav_point_manager.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';

class MapEditPage extends StatefulWidget {
  const MapEditPage({super.key});

  @override
  State<MapEditPage> createState() => _MapEditPageState();
}

class _MapEditPageState extends State<MapEditPage> {
  late MainFlame game;
  late GlobalState globalState;
  late RosChannel rosChannel;
  late NavPointManager navPointManager;
  
  // 当前选中的编辑工具
  String? selectedTool;
  
  // 导航点列表
  List<NavPoint> navPoints = [];

  @override
  void initState() {
    super.initState();
    globalState = Provider.of<GlobalState>(context, listen: false);
    rosChannel = Provider.of<RosChannel>(context, listen: false);
    navPointManager = NavPointManager();
    
    // 创建Flame游戏实例，只显示地图和网格
    game = MainFlame(rosChannel: rosChannel, globalState: globalState);
    
    // 加载导航点
    _loadNavPoints();
  }
  
  // 加载导航点
  Future<void> _loadNavPoints() async {
    await navPointManager.loadNavPoints();
    setState(() {
      navPoints = navPointManager.navPoints;
    });
  }
  
  // 添加导航点
  Future<void> _addNavPoint(double x, double y) async {
    final name = await _showAddNavPointDialog(x, y);
    if (name != null && name.isNotEmpty) {
      await navPointManager.addNavPoint(x, y, 0.0, name);
      await _loadNavPoints();
    }
  }
  
  // 显示添加导航点对话框
  Future<String?> _showAddNavPointDialog(double x, double y) async {
    final TextEditingController nameController = TextEditingController();
    nameController.text = '导航点_${navPoints.length + 1}';
    
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
          GestureDetector(
            onTapDown: (details) {
              if (selectedTool == 'addNavPoint') {
                // 将屏幕坐标转换为地图坐标
                final screenPoint = details.localPosition;
                // TODO: 这里需要将屏幕坐标转换为地图坐标
                // 暂时使用屏幕坐标作为示例
                _addNavPoint(screenPoint.dx, screenPoint.dy);
              }
            },
            child: GameWidget(game: game),
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
      height: 80,
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
                  onPressed: () {
                    // TODO: 实现保存功能
                    print('保存');
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
                '地图编辑模式',
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
                  onPressed: () {
                    globalState.exitMapEditMode();
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
              label: '添加导航点',
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
            
            // 导航点列表
            if (navPoints.isNotEmpty) ...[
              Container(
                width: 120,
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: Colors.grey.withOpacity(0.1),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      '导航点列表 (${navPoints.length})',
                      style: const TextStyle(
                        fontSize: 12,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    const SizedBox(height: 8),
                    ...navPoints.take(3).map((point) => Padding(
                      padding: const EdgeInsets.only(bottom: 4),
                      child: Row(
                        children: [
                          Expanded(
                            child: Text(
                              point.name,
                              style: const TextStyle(fontSize: 10),
                              overflow: TextOverflow.ellipsis,
                            ),
                          ),
                          IconButton(
                            icon: const Icon(Icons.delete, size: 12),
                            onPressed: () => _deleteNavPoint(point.id),
                            padding: EdgeInsets.zero,
                            constraints: const BoxConstraints(
                              minWidth: 20,
                              minHeight: 20,
                            ),
                          ),
                        ],
                      ),
                    )),
                    if (navPoints.length > 3)
                      Text(
                        '...还有${navPoints.length - 3}个',
                        style: const TextStyle(fontSize: 10, color: Colors.grey),
                      ),
                  ],
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }
  
  // 删除导航点
  Future<void> _deleteNavPoint(String id) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Text('确认删除'),
          content: const Text('确定要删除这个导航点吗？'),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(false),
              child: const Text('取消'),
            ),
            ElevatedButton(
              onPressed: () => Navigator.of(context).pop(true),
              style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
              child: const Text('删除'),
            ),
          ],
        );
      },
    );
    
    if (confirmed == true) {
      await navPointManager.removeNavPoint(id);
      await _loadNavPoints();
    }
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
              } else {
                selectedTool = toolName; // 选择工具
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
              description: '点击地图添加导航点，可拖拽旋转方向',
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
            
            const SizedBox(height: 16),
            
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
