import 'dart:async';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/display_waypoint.dart';
import 'package:ros_flutter_gui_app/display/display_pose_direction.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:provider/provider.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class DisplayNavPoint extends StatefulWidget {
  final NavPoint point;
  final double size;
  final RobotPose? currentNavGoal;
  final Matrix4 globalTransform;
  final Offset originPose;
  final double scaleValue;
  final Function(RobotPose) onSendNavigationGoal;
  final Function(String) onSendTopologyGoal;
  final Function(NavPoint) onDeletePoint;
  final Function(NavPoint, double, double) onUpdatePointPosition;
  final Function(NavPoint, double) onUpdatePointAngle;

  const DisplayNavPoint({
    Key? key,
    required this.point,
    required this.size,
    required this.currentNavGoal,
    required this.globalTransform,
    required this.originPose,
    required this.scaleValue,
    required this.onSendNavigationGoal,
    required this.onSendTopologyGoal,
    required this.onDeletePoint,
    required this.onUpdatePointPosition,
    required this.onUpdatePointAngle,
  }) : super(key: key);

  @override
  State<DisplayNavPoint> createState() => _DisplayNavPointState();
}

class _DisplayNavPointState extends State<DisplayNavPoint> {
  OverlayEntry? _overlayEntry;

  @override
  void dispose() {
    _removeOverlay();
    super.dispose();
  }

  void _removeOverlay() {
    _overlayEntry?.remove();
    _overlayEntry = null;
  }

  void _showPointInfoDialog(bool isEditMode) {
    _removeOverlay();
    
    // 获取当前组件在屏幕上的位置
    final RenderBox? renderBox = context.findRenderObject() as RenderBox?;
    if (renderBox == null) return;
    
    final position = renderBox.localToGlobal(Offset.zero);
    final size = renderBox.size;
    
    _overlayEntry = OverlayEntry(
      builder: (context) => Material(
        color: Colors.transparent,
        child: Stack(
          children: [
            // 透明覆盖层，用于检测点击外部关闭
            Positioned.fill(
              child: GestureDetector(
                onTap: _removeOverlay,
                child: Container(
                  color: Colors.transparent,
                ),
              ),
            ),
            // 右侧窗体
            Positioned(
              right: 0,
              top: 0,
              bottom: 0,
              child: Container(
                width: 200,
                decoration: BoxDecoration(
                  color: Theme.of(context).brightness == Brightness.dark
                      ? Colors.grey[900]!.withOpacity(0.95)
                      : Colors.white.withOpacity(0.95),
                  border: Border(
                    left: BorderSide(
                      color: Theme.of(context).brightness == Brightness.dark
                          ? Colors.grey[700]!
                          : Colors.grey[300]!,
                      width: 1,
                    ),
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.1),
                      blurRadius: 8,
                      offset: const Offset(-2, 0),
                    ),
                  ],
                ),
                child: Column(
                  children: [
                    // 标题栏
                    Container(
                      padding: const EdgeInsets.all(16),
                      decoration: BoxDecoration(
                        color: Theme.of(context).brightness == Brightness.dark
                            ? Colors.grey[800]!.withOpacity(0.8)
                            : Colors.grey[50]!.withOpacity(0.8),
                        border: Border(
                          bottom: BorderSide(
                            color: Theme.of(context).brightness == Brightness.dark
                                ? Colors.grey[700]!
                                : Colors.grey[300]!,
                            width: 1,
                          ),
                        ),
                      ),
                      child: Row(
                        children: [
                          const Icon(Icons.location_on, size: 20),
                          const SizedBox(width: 8),
                                                     Expanded(
                             child: Text(
                               '导航点信息',
                               style: TextStyle(
                                 fontWeight: FontWeight.bold,
                                 fontSize: 16,
                                 color: Theme.of(context).brightness == Brightness.dark
                                     ? Colors.white
                                     : Colors.black87,
                               ),
                             ),
                           ),
                          IconButton(
                            onPressed: _removeOverlay,
                            icon: const Icon(Icons.close, size: 20),
                            padding: EdgeInsets.zero,
                            constraints: const BoxConstraints(),
                          ),
                        ],
                      ),
                    ),
                    // 内容区域
                    Expanded(
                      child: Padding(
                        padding: const EdgeInsets.all(16),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                                                         Text(
                               '名称: ${widget.point.name}',
                               style: TextStyle(
                                 fontSize: 14,
                                 color: Theme.of(context).brightness == Brightness.dark
                                     ? Colors.white70
                                     : Colors.black87,
                               ),
                             ),
                             const SizedBox(height: 8),
                             Text(
                               'X: ${widget.point.x.toStringAsFixed(2)}',
                               style: TextStyle(
                                 fontSize: 14,
                                 color: Theme.of(context).brightness == Brightness.dark
                                     ? Colors.white70
                                     : Colors.black87,
                               ),
                             ),
                             Text(
                               'Y: ${widget.point.y.toStringAsFixed(2)}',
                               style: TextStyle(
                                 fontSize: 14,
                                 color: Theme.of(context).brightness == Brightness.dark
                                     ? Colors.white70
                                     : Colors.black87,
                               ),
                             ),
                             Text(
                               '角度: ${(widget.point.theta * 180 / 3.14159).toStringAsFixed(1)}°',
                               style: TextStyle(
                                 fontSize: 14,
                                 color: Theme.of(context).brightness == Brightness.dark
                                     ? Colors.white70
                                     : Colors.black87,
                               ),
                             ),
                            const Spacer(),
                            // 按钮区域
                            Column(
                              children: [
                                SizedBox(
                                  width: double.infinity,
                                  child: ElevatedButton(
                                    onPressed: () {
                                      widget.onSendNavigationGoal(RobotPose(
                                        widget.point.x,
                                        widget.point.y,
                                        widget.point.theta,
                                      ));
                                      _removeOverlay();
                                    },
                                    style: ElevatedButton.styleFrom(
                                      backgroundColor: Colors.blue,
                                      foregroundColor: Colors.white,
                                      padding: const EdgeInsets.symmetric(vertical: 12),
                                    ),
                                    child: const Text('单点导航'),
                                  ),
                                ),
                                SizedBox(
                                  width: double.infinity,
                                  child: ElevatedButton(
                                    onPressed: () {
                                      widget.onSendTopologyGoal(widget.point.name);
                                      _removeOverlay();
                                    },
                                    style: ElevatedButton.styleFrom(
                                      backgroundColor: Colors.green,
                                      foregroundColor: Colors.white,
                                      padding: const EdgeInsets.symmetric(vertical: 12),
                                    ),
                                    child: const Text('拓扑点导航'),
                                  ),
                                ),
                                if (isEditMode) ...[
                                  const SizedBox(height: 8),
                                  SizedBox(
                                    width: double.infinity,
                                    child: ElevatedButton(
                                      onPressed: () {
                                        widget.onDeletePoint(widget.point);
                                        _removeOverlay();
                                      },
                                      style: ElevatedButton.styleFrom(
                                        backgroundColor: Colors.red,
                                        foregroundColor: Colors.white,
                                        padding: const EdgeInsets.symmetric(vertical: 12),
                                      ),
                                      child: const Text('删除'),
                                    ),
                                  ),
                                ],
                              ],
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
    
    Overlay.of(context).insert(_overlayEntry!);
  }

  @override
  Widget build(BuildContext context) {
    final globalState = Provider.of<GlobalState>(context, listen: false);
    final isAddNavPointMode = globalState.mode.value == Mode.addNavPoint;
    
    return Transform(
      transform: widget.globalTransform,
      origin: widget.originPose,
      child: Transform(
        alignment: Alignment.center,
        transform: Matrix4.identity()
          ..translate(
            widget.point.x - widget.size / 2,
            widget.point.y - widget.size / 2,
          )
          ..rotateZ(-widget.point.theta),
        child: GestureDetector(
          onTap: () {
              _showPointInfoDialog(isAddNavPointMode);
          },
          onDoubleTap: () {
            if (isAddNavPointMode) {
              widget.onDeletePoint(widget.point);
            }
          },
          onPanUpdate: isAddNavPointMode ? (details) {
            final dx = details.delta.dx / widget.scaleValue;
            final dy = details.delta.dy / widget.scaleValue;
            widget.onUpdatePointPosition(widget.point, dx, dy);
          } : null,
          child: Container(
            width: widget.size,
            height: widget.size,
            child: Stack(
              alignment: Alignment.center,
              children: [
                // 方向指示器（仅在编辑模式下显示）
                if (isAddNavPointMode)
                  DisplayPoseDirection(
                    size: widget.size,
                    initAngle: -widget.point.theta,
                    resetAngle: false,
                    onRotateCallback: (angle) {
                      widget.onUpdatePointAngle(widget.point, -angle);
                    },
                  ),
                // 导航点图标
                DisplayWayPoint(
                  size: widget.size * 0.6,
                  color: widget.currentNavGoal?.x == widget.point.x &&
                          widget.currentNavGoal?.y == widget.point.y &&
                          widget.currentNavGoal?.theta == widget.point.theta
                      ? Colors.pink
                      : Colors.green,
                  count: 4,
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
} 