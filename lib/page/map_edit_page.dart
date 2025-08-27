import 'package:flutter/material.dart';
import 'package:flutter/gestures.dart';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/page/main_page.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/page/map_edit_flame.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/provider/nav_point_manager.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/display/waypoint.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/basic/math.dart';
import 'dart:math';

// 点位信息类
class WayPointInfo {
  final String name;
  final double x;
  final double y;
  final double direction;
  final bool isSelected;
  
  WayPointInfo({
    required this.name,
    required this.x,
    required this.y,
    required this.direction,
    this.isSelected = false,
  });
  
  // 从Map创建
  factory WayPointInfo.fromMap(Map<String, dynamic> map) {
    return WayPointInfo(
      name: map['name'] ?? '',
      x: double.tryParse(map['x'].toString()) ?? 0.0,
      y: double.tryParse(map['y'].toString()) ?? 0.0,
      direction: double.tryParse(map['direction'].toString()) ?? 0.0,
      isSelected: map['isSelected'] ?? false,
    );
  }
  
  // 转换为Map
  Map<String, dynamic> toMap() {
    return {
      'name': name,
      'x': x.toStringAsFixed(2),
      'y': y.toStringAsFixed(2),
      'direction': direction.toStringAsFixed(1),
      'isSelected': isSelected,
    };
  }
  
  // 复制并更新
  WayPointInfo copyWith({
    String? name,
    double? x,
    double? y,
    double? direction,
    bool? isSelected,
  }) {
    return WayPointInfo(
      name: name ?? this.name,
      x: x ?? this.x,
      y: y ?? this.y,
      direction: direction ?? this.direction,
      isSelected: isSelected ?? this.isSelected,
    );
  }
}

class MapEditPage extends StatefulWidget {
  const MapEditPage({super.key});

  @override
  State<MapEditPage> createState() => _MapEditPageState();
}

class _MapEditPageState extends State<MapEditPage> {
  late MapEditFlame game;
  late GlobalState globalState;
  late RosChannel rosChannel;
  late NavPointManager navPointManager;
  
  // 当前选中的编辑工具
  String? selectedTool;
  
  // 导航点列表
  List<NavPoint> navPoints = [];
  
  // 当前选中的点位信息
  WayPointInfo? selectedWayPointInfo;
  
  // 当前选中点位的动态更新回调
  VoidCallback? _onCurrentSelectPointUpdate;

  // WayPoint手势处理相关变量
  bool _isDraggingWayPoint = false;
  WayPoint? _draggedWayPoint;
  Vector2? _dragStartPosition;
  Vector2? _dragStartWorldPosition;

  @override
  void initState() {
    super.initState();
    globalState = Provider.of<GlobalState>(context, listen: false);
    rosChannel = Provider.of<RosChannel>(context, listen: false);
    navPointManager = NavPointManager();
    
    // 创建专门的地图编辑Flame组件，传入回调函数
    game = MapEditFlame(
      rosChannel: rosChannel,
      onAddNavPoint: _addNavPoint,
      onWayPointSelectionChanged: _onWayPointSelectionChanged,
      currentSelectPointUpdate: _onCurrentSelectPointUpdate,
    );
    
    // 加载导航点
    _loadNavPoints();
  }
  
  // WayPoint拖拽开始处理
  void _handleWayPointDragStart(Vector2 position) {
    // 检查是否点击在WayPoint上
    final worldPosition = game.camera.globalToLocal(position);
    final wayPoint = _findWayPointAtPosition(worldPosition);
    
    if (wayPoint != null) {
      _isDraggingWayPoint = true;
      _draggedWayPoint = wayPoint;
      _dragStartPosition = position;
      _dragStartWorldPosition = wayPoint.position.clone();
      
      // 选中该WayPoint
      _selectWayPoint(wayPoint);
    }
  }
  
  // WayPoint拖拽更新处理
  void _handleWayPointDragUpdate(Vector2 position) {
    if (_isDraggingWayPoint && _draggedWayPoint != null && _dragStartPosition != null && _dragStartWorldPosition != null) {
      // 计算拖拽增量
      final delta = position - _dragStartPosition!;
      
      // 将屏幕增量转换为世界坐标增量
      final worldDelta = delta / game.camera.viewfinder.zoom;
      
      // 更新WayPoint位置
      _draggedWayPoint!.position = _dragStartWorldPosition! + worldDelta;
      
      // 调用拖拽更新回调
      _draggedWayPoint!.onDragUpdate?.call();
      
      // 通知外部更新
      _onCurrentSelectPointUpdate?.call();
    }
  }
  
  // WayPoint拖拽结束处理
  void _handleWayPointDragEnd() {
    _isDraggingWayPoint = false;
    _draggedWayPoint = null;
    _dragStartPosition = null;
    _dragStartWorldPosition = null;
  }

  // WayPoint缩放开始处理
  void _handleWayPointScaleStart(Vector2 position) {
    final worldPosition = game.camera.globalToLocal(position);
    final wayPoint = _findWayPointAtPosition(worldPosition);

    if (wayPoint != null) {
      // 如果触摸在WayPoint上，开始WayPoint手势
      _isDraggingWayPoint = true;
      _draggedWayPoint = wayPoint;
      _dragStartPosition = position;
      _dragStartWorldPosition = wayPoint.position.clone();
      
      // 选中该WayPoint
      _selectWayPoint(wayPoint);
    } else {
      // 如果触摸不在WayPoint上，开始地图手势
      game.onScaleStart(position);
    }
  }

  // WayPoint缩放更新处理（同时处理地图缩放、WayPoint拖拽和旋转）
  void _handleWayPointScaleUpdate(double scale, Vector2 position) {
    if (_isDraggingWayPoint && _draggedWayPoint != null && _dragStartPosition != null && _dragStartWorldPosition != null) {
      // 检查是否是旋转手势（scale接近1.0，但位置有变化）
      final delta = position - _dragStartPosition!;
      final isRotation = (scale - 1.0).abs() < 0.1 && delta.length > 5.0;
      
      if (isRotation) {
        // 处理WayPoint旋转
        final center = _draggedWayPoint!.position;
        final startVector = _dragStartPosition! - center;
        final currentVector = position - center;
        
        // 计算角度变化
        final startAngle = atan2(startVector.y, startVector.x);
        final currentAngle = atan2(currentVector.y, currentVector.x);
        final angleDelta = currentAngle - startAngle;
        
        // 更新WayPoint方向角度
        _draggedWayPoint!.directionAngle += angleDelta;
        
        // 同时更新DirectionControl的角度
        final directionControl = _draggedWayPoint!.children.whereType<DirectionControl>().firstOrNull;
        if (directionControl != null) {
          directionControl.updateAngle(_draggedWayPoint!.directionAngle);
        }
        
        // 调用拖拽更新回调
        _draggedWayPoint!.onDragUpdate?.call();
        
        // 通知外部更新
        _onCurrentSelectPointUpdate?.call();
        
        // 更新起始位置，用于连续旋转
        _dragStartPosition = position;
      } else {
        // 处理WayPoint拖拽 - 使用累积增量
        final worldDelta = delta / game.camera.viewfinder.zoom;
        _draggedWayPoint!.position = _dragStartWorldPosition! + worldDelta;
        
        // 调用拖拽更新回调
        _draggedWayPoint!.onDragUpdate?.call();
        
        // 通知外部更新
        _onCurrentSelectPointUpdate?.call();
      }
    } else {
      // 处理地图手势（拖拽和缩放）
      game.onScaleUpdate(scale, position);
    }
  }

  // WayPoint缩放结束处理
  void _handleWayPointScaleEnd() {
    if (_isDraggingWayPoint) {
      // 结束WayPoint手势
      _isDraggingWayPoint = false;
      _draggedWayPoint = null;
      _dragStartPosition = null;
      _dragStartWorldPosition = null;
    } else {
      // 结束地图手势
      game.onScaleEnd();
    }
  }
  
  // 查找指定位置的WayPoint
  WayPoint? _findWayPointAtPosition(Vector2 worldPosition) {
    for (final wayPoint in game.wayPoints) {
      final distance = (wayPoint.position - worldPosition).length;
      if (distance <= wayPoint.waypointSize / 2) {
        return wayPoint;
      }
    }
    return null;
  }
  
  // 选中WayPoint
  void _selectWayPoint(WayPoint wayPoint) {
    // 取消其他WayPoint的选中状态
    for (final wp in game.wayPoints) {
      wp.isSelected = false;
    }
    wayPoint.isSelected = true;
    
    // 通知外部WayPoint选择状态变化
    _onWayPointSelectionChanged();
  }
  
  // 导航点选择状态变化回调
  void _onWayPointSelectionChanged() {
    setState(() {
      final info = game.getSelectedWayPointInfo();
      selectedWayPointInfo = info != null ? WayPointInfo.fromMap(info) : null;
    });
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
      
      // 清除选中状态
      setState(() {
        selectedWayPointInfo = null;
      });
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
          Listener(
            onPointerSignal: (pointerSignal) {
              if (pointerSignal is PointerScrollEvent) {
                final position = Vector2(pointerSignal.position.dx, pointerSignal.position.dy);
                game.onScroll(pointerSignal.scrollDelta.dy, position);
              }
            },
            child: GestureDetector(
              onTapDown: (details) {
                final position = Vector2(details.globalPosition.dx, details.globalPosition.dy);
                game.onTapDown(position);
              },
              onScaleStart: (details) {
                final position = Vector2(details.localFocalPoint.dx, details.localFocalPoint.dy);
                _handleWayPointScaleStart(position);
              },
              onScaleUpdate: (details) {
                final position = Vector2(details.localFocalPoint.dx, details.localFocalPoint.dy);
                _handleWayPointScaleUpdate(details.scale, position);
              },
              onScaleEnd: (details) {
                _handleWayPointScaleEnd();
              },
              child: GameWidget(game: game),
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
            
            // 删除选中的导航点
            if (selectedTool == 'addNavPoint') ...[
              _buildEditTool(
                icon: Icons.delete,
                label: '删除选中',
                toolName: 'deleteSelected',
                color: Colors.red,
              ),
              
              const SizedBox(height: 8),
            ],

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
              if (toolName == 'deleteSelected') {
                // 删除选中的导航点
                game.deleteSelectedWayPoint();
                setState(() {});
              } else if (isActive) {
                selectedTool = null; // 取消选择
                game.setSelectedTool(null);
              } else {
                selectedTool = toolName; // 选择工具
                game.setSelectedTool(toolName);
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
          _buildInfoRow('方向', '${(selectedWayPointInfo!.direction * 180 / 3.14159).toStringAsFixed(1)}°'),
          
          const SizedBox(height: 12),
          
          // 关闭按钮
          SizedBox(
            width: double.infinity,
            child: TextButton.icon(
              onPressed: () {
                setState(() {
                  selectedWayPointInfo = null;
                });
              },
              icon: const Icon(Icons.close, size: 16),
              label: const Text('关闭'),
              style: TextButton.styleFrom(
                backgroundColor: Colors.grey.withOpacity(0.2),
                padding: const EdgeInsets.symmetric(vertical: 8),
              ),
            ),
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
