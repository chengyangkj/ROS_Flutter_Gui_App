import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/display/tile_map.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:toastification/toastification.dart';

enum EditToolType {
  Move,
  AddNavPoint,
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

  @override
  void initState() {
    super.initState();
    selectedTool = EditToolType.Move;
    context.read<GlobalState>().mode.value = Mode.mapEdit;
  }
  
  @override
  void dispose() {
    context.read<GlobalState>().mode.value = Mode.normal;
    super.dispose();
  }

//   // 导航点选择状态变化回调
//   void _onWayPointSelectionChanged() {
//     setState(() {
//       final info = game.getSelectedWayPointInfo();
//       selectedWayPointInfo = info;
//     });
//   }
  
//   Future<void> _loadNavPoints() async {
//     final mapManager = rosChannel.mapManager;
//     navPoints = List.from(mapManager.navPoints);
//     for(var navPoint in navPoints){
//       game.addWayPoint(navPoint);
//     }
//     setState(() {
//     });
//   }
  
//   Future<NavPoint?> _addNavPoint(double x, double y, double theta) async {
//     final name = await _showAddNavPointDialog(x, y);
//     if (name != null && name.isNotEmpty) {
//       final navPoint = NavPoint(
//         x: x,
//         y: y,
//         theta: theta,
//         name: name,
//         type: NavPointType.navGoal,
//       );
//       return navPoint;
//     }
//     return null;
//   }
  
//   Future<String?> _showAddNavPointDialog(double x, double y) async {
//     final TextEditingController nameController = TextEditingController();
//     final mapManager = rosChannel.mapManager;
//     int id = await mapManager.getNextPointId();
//     nameController.text = 'POINT_${id.toString()}';
    
//     return showDialog<String>(
//       context: context,
//       builder: (BuildContext context) {
//         return AlertDialog(
//           title: const Text('添加导航点'),
//           content: Column(
//             mainAxisSize: MainAxisSize.min,
//             children: [
//               Text('位置: (${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)})'),
//               const SizedBox(height: 16),
//               TextField(
//                 controller: nameController,
//                 decoration: const InputDecoration(
//                   labelText: '导航点名称',
//                   border: OutlineInputBorder(),
//                 ),
//                 autofocus: true,
//               ),
//             ],
//           ),
//           actions: [
//             TextButton(
//               onPressed: () => Navigator.of(context).pop(),
//               child: const Text('取消'),
//             ),
//             ElevatedButton(
//               onPressed: () => Navigator.of(context).pop(nameController.text),
//               child: const Text('确定'),
//             ),
//           ],
//         );
//       },
//     );
//   }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final rosChannel = context.read<RosChannel>();
    final mapManager = rosChannel.mapManager;

    return Scaffold(
      body: Stack(
        children: [
          TileMap(
            key: _tileMapKey,
            enableMapInteraction: selectedTool == EditToolType.Move,
            enableTopologyEdit: true,
            selectedNavPointName: selectedNavPoint?.name,
            onNavPointTap: (p) {
              setState(() {
                selectedNavPoint = p;
              });
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
        
        ],
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
          ],
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

//   Widget _buildTopToolbar(BuildContext context, ThemeData theme) {
//     return Container(
//       height: 60,
//       decoration: BoxDecoration(
//         color: Colors.orange,
//         boxShadow: [
//           BoxShadow(
//             color: Colors.black.withOpacity(0.2),
//             blurRadius: 4,
//             offset: const Offset(0, 2),
//           ),
//         ],
//       ),
//       child: Row(
//         children: [
//           // 左侧工具栏按钮
//           Expanded(
//             child: Row(
//               children: [
//                 // 打开文件按钮
//                 IconButton(
//                   icon: const Icon(Icons.folder_open, color: Colors.white, size: 28),
//                   onPressed: () {
//                     // TODO: 实现打开文件功能
//                     print('打开文件');
//                   },
//                   tooltip: '打开文件',
//                 ),
                
//                 const SizedBox(width: 16),
                
//                 IconButton(
//                   icon: const Icon(Icons.save, color: Colors.white, size: 28),
//                   onPressed: () async {
//                     try {
//                       final mapManager = rosChannel.mapManager;
//                       List<NavPoint> navPoints = game.getAllWayPoint();
                      
//                       final currentTopologyMap = mapManager.topologyMap.value;
//                       TopologyMap topologyMap = TopologyMap(
//                         mapName: currentTopologyMap.mapName,
//                         mapProperty: currentTopologyMap.mapProperty,
//                         points: navPoints,
//                         routes: currentTopologyMap.routes,
//                       );
                      
//                       await rosChannel.updateTopologyMap(topologyMap);
//                       await mapManager.saveLocalTopologyMap();
//                       await rosChannel.publishOccupancyGrid();

//                       if (mounted) {
//                         toastification.show(
//                           context: context,
//                           type: ToastificationType.success,
//                           style: ToastificationStyle.flatColored,
//                           title: const Text('保存成功！'),
//                           description: Text('已发布拓扑地图至/map/topology/update话题，栅格地图至/map/update话题'),
//                           autoCloseDuration: const Duration(seconds: 3),
//                         );
//                       }
//                     } catch (e) {
//                       print("保存失败: $e");
//                       if (mounted) {
//                         toastification.show(
//                           context: context,
//                           type: ToastificationType.error,
//                           style: ToastificationStyle.flatColored,
//                           title: const Text('保存失败'),
//                           description: Text('错误: $e'),
//                           autoCloseDuration: const Duration(seconds: 3),
//                         );
//                       }
//                     }
//                   },
//                   tooltip: '保存并发布到ROS',
//                 ),
                
//                 const SizedBox(width: 16),
                
//                 // 撤销按钮
//                 IconButton(
//                   icon: const Icon(Icons.undo, color: Colors.white, size: 28),
//                   onPressed: game.canUndo() ? () {
//                     game.undo();
//                     setState(() {});
//                   } : null,
//                   tooltip: '撤销',
//                 ),
//               ],
//             ),
//           ),
          
//           // 中间标题
//           Expanded(
//             child: Center(
//               child: Text(
//                 '地图编辑',
//                 style: const TextStyle(
//                   color: Colors.white,
//                   fontSize: 20,
//                   fontWeight: FontWeight.bold,
//                 ),
//               ),
//             ),
//           ),
          
//           // 右侧退出按钮
//           Expanded(
//             child: Align(
//               alignment: Alignment.centerRight,
//               child: Padding(
//                 padding: const EdgeInsets.only(right: 20),
//                 child: IconButton(
//                   icon: const Icon(Icons.close, color: Colors.white, size: 28),
//                   onPressed: () async {
//                     final mapManager = rosChannel.mapManager;
//                     List<NavPoint> navPoints = game.getAllWayPoint();
//                     mapManager.setNavPoints(navPoints);
//                     await mapManager.saveLocalTopologyMap();
                    
//                     widget.onExit?.call();
//                     Navigator.pop(context);
//                   },
//                   tooltip: '退出地图编辑模式',
//                 ),
//               ),
//             ),
//           ),
//         ],
//       ),
//     );
//   }

//   Widget _buildEditToolbar(BuildContext context, ThemeData theme) {
//     return Card(
//       elevation: 8,
//       child: Container(
//         padding: const EdgeInsets.all(8),
//         child: Column(
//           mainAxisSize: MainAxisSize.min,
//           children: [
//             // 移动工具
//             _buildEditTool(
//               icon: Icons.open_with,
//               label: '移动',
//               toolName: EditToolType.move,
//               color: Colors.grey,
//             ),
            
//             const SizedBox(height: 8),
            
//             // 导航点添加工具
//             _buildEditTool(
//               icon: Icons.add_location,
//               label: '添加点位',
//               toolName: EditToolType.addNavPoint,
//               color: Colors.blue,
//             ),
            
//             const SizedBox(height: 8),
            
//             // 拓扑连线工具
//             _buildEditTool(
//               icon: Icons.link,
//               label: '拓扑路径',
//               toolName: EditToolType.addRoute,
//               color: Colors.purple,
//             ),
            
//             const SizedBox(height: 8),
            
//             // 障碍物绘制工具
//             _buildEditTool(
//               icon: Icons.brush,
//               label: '绘制障碍物',
//               toolName: EditToolType.drawObstacle,
//               color: Colors.red,
//             ),
            
//             const SizedBox(height: 8),
            
//             // 障碍物擦除工具
//             _buildEditTool(
//               icon: Icons.auto_fix_high,
//               label: '擦除障碍物',
//               toolName: EditToolType.eraseObstacle,
//               color: Colors.green,
//             ),
            
//             const SizedBox(height: 8),
            
//             // 直线绘制工具
//             _buildEditTool(
//               icon: Icons.straighten,
//               label: '直线绘制',
//               toolName: EditToolType.drawLine,
//               color: Colors.orange,
//             ),
            
//             // 画笔大小调整（仅在绘制/擦除/直线绘制工具时显示）
//             if (selectedTool == EditToolType.drawObstacle || 
//                 selectedTool == EditToolType.eraseObstacle ||
//                 selectedTool == EditToolType.drawLine) ...[
//               const SizedBox(height: 16),
//               const Divider(),
//               const SizedBox(height: 8),
//               Padding(
//                 padding: const EdgeInsets.symmetric(horizontal: 8.0),
//                 child: Column(
//                   crossAxisAlignment: CrossAxisAlignment.start,
//                   children: [
//                     Text(
//                       '画笔大小: ${brushSize.toStringAsFixed(2)}m',
//                       style: const TextStyle(fontSize: 12, fontWeight: FontWeight.bold),
//                     ),
//                     const SizedBox(height: 4),
//                     Slider(
//                       value: brushSize,
//                       min: 0.05,
//                       max: 1.0,
//                       divisions: 19,
//                       label: '${brushSize.toStringAsFixed(2)}m',
//                       onChanged: (value) {
//                         setState(() {
//                           brushSize = value;
//                           game.setBrushSize(value);
//                         });
//                       },
//                     ),
//                   ],
//                 ),
//               ),
//             ],
//           ],
//         ),
//       ),
//     );
//   }
  


//   Widget _buildEditTool({
//     required IconData icon,
//     required String label,
//     required EditToolType toolName,
//     required Color color,
//   }) {
//     final isActive = selectedTool == toolName;
    
//     return GestureDetector(
//       onTap: () {
//         if (isActive) {
//           selectedTool = null; // 取消选择
//           game.setSelectedTool(null);
//           // 退出所有点位编辑模式
//           for (final wp in game.wayPoints) {
//             wp.setEditMode(false);
//           }
//         } else {
//           selectedTool = toolName; // 选择工具
//           game.setSelectedTool(toolName);
//           // 切换到其他工具时退出所有点位编辑模式
//           if (toolName != EditToolType.addNavPoint) {
//             for (final wp in game.wayPoints) {
//               wp.setEditMode(false);
//             }
//           }
//         }
//         setState(() {});
//       },
//       child: Container(
//         width: 120,
//         padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 4),
//         decoration: BoxDecoration(
//           color: isActive ? color.withOpacity(0.2) : Colors.transparent,
//           borderRadius: BorderRadius.circular(8),
//           border: isActive ? Border.all(color: color, width: 2) : null,
//         ),
//         child: Column(
//           children: [
//             Icon(
//               icon,
//               size: 24,
//               color: isActive ? color : Colors.grey,
//             ),
//             const SizedBox(height: 4),
//             Text(
//               label,
//               style: TextStyle(
//                 fontSize: 12,
//                 color: isActive ? color : Colors.grey,
//                 fontWeight: isActive ? FontWeight.bold : FontWeight.normal,
//               ),
//               textAlign: TextAlign.center,
//             ),
//           ],
//         ),
//       ),
//     );
//   }

//   Widget _buildInfoPanel(BuildContext context, ThemeData theme) {
//     return Card(
//       elevation: 8,
//       child: Container(
//         width: 280,
//         padding: const EdgeInsets.all(16),
//         child: Column(
//           crossAxisAlignment: CrossAxisAlignment.start,
//           mainAxisSize: MainAxisSize.min,
//           children: [
//             // 如果有选中的路线，显示路线信息
//             if (selectedRoute != null) ...[
//               _buildRouteInfo(theme),
//               const SizedBox(height: 16),
//             ] else if (selectedWayPointInfo != null || editingPoint != null) ...[
//               // 如果有选中的导航点，显示导航点信息
//               _buildWayPointInfo(theme),
//               const SizedBox(height: 16),
//             ] else ...[
//               // 否则显示编辑模式说明
//               Text(
//                 '编辑模式说明',
//                 style: theme.textTheme.titleMedium?.copyWith(
//                   fontWeight: FontWeight.bold,
//                 ),
//               ),
//               const SizedBox(height: 16),
              
//               _buildInstructionItem(
//                 icon: Icons.add_location,
//                 title: '添加导航点',
//                 description: '选择工具后双击地图添加导航点',
//                 color: Colors.blue,
//               ),
              
//               const SizedBox(height: 12),
              
//               _buildInstructionItem(
//                 icon: Icons.brush,
//                 title: '绘制障碍物',
//                 description: '拖拽鼠标绘制障碍物区域',
//                 color: Colors.red,
//               ),
              
//               const SizedBox(height: 12),
              
//               _buildInstructionItem(
//                 icon: Icons.auto_fix_high,
//                 title: '擦除障碍物',
//                 description: '拖拽鼠标擦除障碍物区域',
//                 color: Colors.green,
//               ),
              
//               const SizedBox(height: 12),
              
//               _buildInstructionItem(
//                 icon: Icons.touch_app,
//                 title: '导航点操作',
//                 description: '单击选中，拖拽移动，拖拽红点旋转，删除按钮删除选中',
//                 color: Colors.orange,
//               ),
              
//               const SizedBox(height: 16),
//             ],
            
//             // 导航点计数显示
//             Container(
//               padding: const EdgeInsets.all(12),
//               decoration: BoxDecoration(
//                 color: Colors.blue.withOpacity(0.1),
//                 borderRadius: BorderRadius.circular(8),
//                 border: Border.all(color: Colors.blue.withOpacity(0.3)),
//               ),
//               child: Row(
//                 children: [
//                   Icon(Icons.add_location, color: Colors.blue, size: 16),
//                   const SizedBox(width: 8),
//                   Expanded(
//                     child: Column(
//                       crossAxisAlignment: CrossAxisAlignment.start,
//                       children: [
//                         Text(
//                           '导航点数量',
//                           style: TextStyle(
//                             fontWeight: FontWeight.bold,
//                             color: Colors.blue,
//                           ),
//                         ),
//                         Text(
//                           '${game.wayPointCount} 个',
//                           style: const TextStyle(fontSize: 12),
//                         ),
//                       ],
//                     ),
//                   ),
//                 ],
//               ),
//             ),
            
//             Container(
//               padding: const EdgeInsets.all(8),
//               decoration: BoxDecoration(
//                 color: Colors.orange.withOpacity(0.1),
//                 borderRadius: BorderRadius.circular(8),
//                 border: Border.all(color: Colors.orange.withOpacity(0.3)),
//               ),
//               child: Row(
//                 children: [
//                   Icon(Icons.info, color: Colors.orange, size: 16),
//                   const SizedBox(width: 8),
//                   Expanded(
//                     child: Text(
//                       '提示：使用鼠标滚轮缩放，拖拽移动地图',
//                       style: TextStyle(
//                         fontSize: 12,
//                         color: Colors.orange.shade700,
//                       ),
//                     ),
//                   ),
//                 ],
//               ),
//             ),
//           ],
//         ),
//       ),
//     );
//   }
  
//   // 构建导航点信息显示
//   Widget _buildWayPointInfo(ThemeData theme) {
//     final point = editingPoint ?? selectedWayPointInfo;
//     if (point == null) return const SizedBox.shrink();
    
//     return Container(
//       padding: const EdgeInsets.all(16),
//       decoration: BoxDecoration(
//         color: Colors.blue.withOpacity(0.1),
//         borderRadius: BorderRadius.circular(8),
//         border: Border.all(color: Colors.blue.withOpacity(0.5)),
//       ),
//       child: Column(
//         crossAxisAlignment: CrossAxisAlignment.start,
//         children: [
//           Row(
//             children: [
//               Icon(Icons.location_on, color: Colors.blue, size: 20),
//               const SizedBox(width: 8),
//               Text(
//                 '点位属性',
//                 style: theme.textTheme.titleMedium?.copyWith(
//                   fontWeight: FontWeight.bold,
//                   color: Colors.blue,
//                 ),
//               ),
//             ],
//           ),
//           const SizedBox(height: 12),
          
//           // 名称编辑
//           TextField(
//             controller: TextEditingController(text: point.name),
//             decoration: const InputDecoration(
//               labelText: '名称',
//               border: OutlineInputBorder(),
//               isDense: true,
//             ),
//             onChanged: (value) {
//               setState(() {
//                 editingPoint = NavPoint(
//                   name: value,
//                   x: point.x,
//                   y: point.y,
//                   theta: point.theta,
//                   type: point.type,
//                 );
//               });
//             },
//           ),
//           const SizedBox(height: 8),
          
//           // X坐标编辑
//           TextField(
//             controller: TextEditingController(text: point.x.toStringAsFixed(3)),
//             decoration: const InputDecoration(
//               labelText: 'X坐标 (m)',
//               border: OutlineInputBorder(),
//               isDense: true,
//             ),
//             keyboardType: const TextInputType.numberWithOptions(decimal: true),
//             onChanged: (value) {
//               final x = double.tryParse(value) ?? point.x;
//               setState(() {
//                 editingPoint = NavPoint(
//                   name: point.name,
//                   x: x,
//                   y: point.y,
//                   theta: point.theta,
//                   type: point.type,
//                 );
//               });
//             },
//           ),
//           const SizedBox(height: 8),
          
//           // Y坐标编辑
//           TextField(
//             controller: TextEditingController(text: point.y.toStringAsFixed(3)),
//             decoration: const InputDecoration(
//               labelText: 'Y坐标 (m)',
//               border: OutlineInputBorder(),
//               isDense: true,
//             ),
//             keyboardType: const TextInputType.numberWithOptions(decimal: true),
//             onChanged: (value) {
//               final y = double.tryParse(value) ?? point.y;
//               setState(() {
//                 editingPoint = NavPoint(
//                   name: point.name,
//                   x: point.x,
//                   y: y,
//                   theta: point.theta,
//                   type: point.type,
//                 );
//               });
//             },
//           ),
//           const SizedBox(height: 8),
          
//           // Theta编辑
//           TextField(
//             controller: TextEditingController(text: (point.theta * 180 / 3.14159).toStringAsFixed(2)),
//             decoration: const InputDecoration(
//               labelText: '方向角 (°)',
//               border: OutlineInputBorder(),
//               isDense: true,
//             ),
//             keyboardType: const TextInputType.numberWithOptions(decimal: true),
//             onChanged: (value) {
//               final theta = (double.tryParse(value) ?? point.theta * 180 / 3.14159) * 3.14159 / 180;
//               setState(() {
//                 editingPoint = NavPoint(
//                   name: point.name,
//                   x: point.x,
//                   y: point.y,
//                   theta: theta,
//                   type: point.type,
//                 );
//               });
//             },
//           ),
          
//           const SizedBox(height: 12),
          
//           // 操作按钮
//           Row(
//             children: [
//               Expanded(
//                 child: ElevatedButton.icon(
//                   onPressed: () {
//                     if (editingPoint != null && selectedWayPointInfo != null) {
//                       // 应用修改
//                       game.modifySelectedPoint(editingPoint!);
//                       setState(() {
//                         selectedWayPointInfo = editingPoint;
//                         editingPoint = null;
//                       });
//                     }
//                   },
//                   icon: const Icon(Icons.check, size: 16),
//                   label: const Text('确定'),
//                   style: ElevatedButton.styleFrom(
//                     backgroundColor: Colors.green,
//                     foregroundColor: Colors.white,
//                     padding: const EdgeInsets.symmetric(vertical: 10),
//                   ),
//                 ),
//               ),
//               const SizedBox(width: 8),
//               Expanded(
//                 child: ElevatedButton.icon(
//                   onPressed: () {
//                     String name = game.deleteSelectedWayPoint();
//                     final mapManager = rosChannel.mapManager;
//                     mapManager.removeNavPoint(name);
//                     setState(() {
//                       selectedWayPointInfo = null;
//                       editingPoint = null;
//                     });
//                   },
//                   icon: const Icon(Icons.delete, size: 16),
//                   label: const Text('删除'),
//                   style: ElevatedButton.styleFrom(
//                     backgroundColor: Colors.red,
//                     foregroundColor: Colors.white,
//                     padding: const EdgeInsets.symmetric(vertical: 10),
//                   ),
//                 ),
//               ),
//             ],
//           ),
//         ],
//       ),
//     );
//   }
  
//   // 构建路线信息显示
//   Widget _buildRouteInfo(ThemeData theme) {
//     if (selectedRoute == null) return const SizedBox.shrink();
    
//     return Container(
//       padding: const EdgeInsets.all(16),
//       decoration: BoxDecoration(
//         color: Colors.green.withOpacity(0.1),
//         borderRadius: BorderRadius.circular(8),
//         border: Border.all(color: Colors.green.withOpacity(0.5)),
//       ),
//       child: Column(
//         crossAxisAlignment: CrossAxisAlignment.start,
//         children: [
//           Row(
//             children: [
//               Icon(Icons.route, color: Colors.green, size: 20),
//               const SizedBox(width: 8),
//               Text(
//                 '路线属性',
//                 style: theme.textTheme.titleMedium?.copyWith(
//                   fontWeight: FontWeight.bold,
//                   color: Colors.green,
//                 ),
//               ),
//             ],
//           ),
//           const SizedBox(height: 12),
          
//           _buildInfoRow('起点', selectedRoute!.fromPoint),
//           _buildInfoRow('终点', selectedRoute!.toPoint),
          
//           const SizedBox(height: 8),
          
//           DropdownButtonFormField<String>(
//             value: selectedRoute!.routeInfo.controller,
//             decoration: const InputDecoration(
//               labelText: '控制器',
//               border: OutlineInputBorder(),
//               isDense: true,
//             ),
//             items: supportControllers.map((controller) {
//               return DropdownMenuItem(
//                 value: controller,
//                 child: Text(controller),
//               );
//             }).toList(),
//             onChanged: (value) {
//               if (value != null) {
//                 setState(() {
//                   selectedRoute = TopologyRoute(
//                     fromPoint: selectedRoute!.fromPoint,
//                     toPoint: selectedRoute!.toPoint,
//                     routeInfo: RouteInfo(controller: value),
//                   );
//                   _updateRoute();
//                 });
//               }
//             },
//           ),
          
//           const SizedBox(height: 12),
          
//           // 删除按钮
//           ElevatedButton.icon(
//             onPressed: () {
//               _deleteRoute();
//             },
//             icon: const Icon(Icons.delete, size: 16),
//             label: const Text('删除路线'),
//             style: ElevatedButton.styleFrom(
//               backgroundColor: Colors.red,
//               foregroundColor: Colors.white,
//               padding: const EdgeInsets.symmetric(vertical: 10),
//             ),
//           ),
//         ],
//       ),
//     );
//   }
  
//   void _updateRoute() {
//     if (selectedRoute == null) return;
    
//     final mapManager = rosChannel.mapManager;
//     mapManager.updateRoute(
//       selectedRoute!.fromPoint,
//       selectedRoute!.toPoint,
//       selectedRoute!,
//     );
//     rosChannel.updateTopologyMap(mapManager.topologyMap.value);
//     setState(() {});
//   }
  
//   void _deleteRoute() {
//     if (selectedRoute == null) return;
    
//     final mapManager = rosChannel.mapManager;
//     mapManager.removeRoute(selectedRoute!.fromPoint, selectedRoute!.toPoint);
//     rosChannel.updateTopologyMap(mapManager.topologyMap.value);
    
//     setState(() {
//       selectedRoute = null;
//     });
//   }
  
//   // 构建信息行
//   Widget _buildCoordinateDisplay(BuildContext context, ThemeData theme) {
//     return Card(
//       elevation: 4,
//       child: Container(
//         padding: const EdgeInsets.all(8),
//         child: Column(
//           crossAxisAlignment: CrossAxisAlignment.start,
//           mainAxisSize: MainAxisSize.min,
//           children: [
//             _buildCoordinateRow(
//               '鼠标',
//               mouseWorldPos != null
//                   ? 'X: ${mouseWorldPos!['x']!.toStringAsFixed(3)}, Y: ${mouseWorldPos!['y']!.toStringAsFixed(3)}'
//                   : '-',
//             ),
//             const SizedBox(height: 4),
//             _buildCoordinateRow(
//               '机器人',
//               robotPos != null
//                   ? 'X: ${robotPos!['x']!.toStringAsFixed(3)}, Y: ${robotPos!['y']!.toStringAsFixed(3)}, θ: ${robotPos!['theta']!.toStringAsFixed(3)}'
//                   : '-',
//             ),
//           ],
//         ),
//       ),
//     );
//   }
  
//   Widget _buildCoordinateRow(String label, String value) {
//     return Row(
//       mainAxisSize: MainAxisSize.min,
//       children: [
//         SizedBox(
//           width: 60,
//           child: Text(
//             '$label:',
//             style: const TextStyle(
//               fontWeight: FontWeight.bold,
//               fontSize: 12,
//             ),
//           ),
//         ),
//         const SizedBox(width: 4),
//         Text(
//           value,
//           style: const TextStyle(fontSize: 12),
//         ),
//       ],
//     );
//   }

//   Widget _buildInfoRow(String label, String value) {
//     return Padding(
//       padding: const EdgeInsets.symmetric(vertical: 4),
//       child: Row(
//         children: [
//           SizedBox(
//             width: 50,
//             child: Text(
//               '$label:',
//               style: const TextStyle(
//                 fontWeight: FontWeight.bold,
//                 fontSize: 12,
//               ),
//             ),
//           ),
//           Expanded(
//             child: Text(
//               value,
//               style: const TextStyle(fontSize: 12),
//             ),
//           ),
//         ],
//       ),
//     );
//   }

//   Widget _buildInstructionItem({
//     required IconData icon,
//     required String title,
//     required String description,
//     required Color color,
//   }) {
//     return Row(
//       crossAxisAlignment: CrossAxisAlignment.start,
//       children: [
//         Icon(icon, color: color, size: 16),
//         const SizedBox(width: 8),
//         Expanded(
//           child: Column(
//             crossAxisAlignment: CrossAxisAlignment.start,
//             children: [
//               Text(
//                 title,
//                 style: TextStyle(
//                   fontWeight: FontWeight.bold,
//                   color: color,
//                 ),
//               ),
//               Text(
//                 description,
//                 style: const TextStyle(fontSize: 12),
//               ),
//             ],
//           ),
//         ),
//       ],
//     );
//   }
  
//   // 构建添加机器人位置按钮
//   Widget _buildAddRobotPositionButton(BuildContext context, ThemeData theme) {
//     return AnimatedOpacity(
//       opacity: selectedTool == EditToolType.addNavPoint ? 1.0 : 0.0,
//       duration: const Duration(milliseconds: 200),
//       child: ElevatedButton.icon(
//         onPressed: selectedTool == EditToolType.addNavPoint ? _onAddRobotPositionButtonPressed : null,
//         icon: const Icon(Icons.my_location, size: 18),
//         label: const Text('使用当前位置'),
//         style: ElevatedButton.styleFrom(
//           backgroundColor: Colors.blue,
//           foregroundColor: Colors.white,
//           padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
//           shape: RoundedRectangleBorder(
//             borderRadius: BorderRadius.circular(8),
//           ),
//           elevation: 4,
//         ),
//       ),
//     );
//   }
  
//   // 按钮点击处理
//   void _onAddRobotPositionButtonPressed() async {
//     await game.addNavPointAtRobotPosition();
//   }
  
//   // 根据工具类型获取鼠标光标
//   MouseCursor _getCursorForTool(EditToolType? tool) {
//     switch (tool) {
//       case EditToolType.move:
//         return SystemMouseCursors.move;
//       case EditToolType.addNavPoint:
//         return SystemMouseCursors.precise;
//       case EditToolType.addRoute:
//         return SystemMouseCursors.precise;
//       case EditToolType.drawObstacle:
//         return SystemMouseCursors.precise;
//       case EditToolType.eraseObstacle:
//         return SystemMouseCursors.precise;
//       case EditToolType.drawLine:
//         return SystemMouseCursors.precise;
//       case null:
//         return SystemMouseCursors.basic;
//     }
//   }
// }
