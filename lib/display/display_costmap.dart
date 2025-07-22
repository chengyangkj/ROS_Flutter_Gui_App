import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayCostMap extends StatefulWidget {
  final double opacity;
  final bool isGlobal;

  const DisplayCostMap({
    super.key,
    this.opacity = 0.5,
    this.isGlobal = false,
  });

  @override
  _DisplayCostMapState createState() => _DisplayCostMapState();
}

class _DisplayCostMapState extends State<DisplayCostMap> {
  ui.Image? _cachedImage;
  OccupancyMap? _lastProcessedMap;
  double _lastOpacity = 0.5;
  bool _isProcessing = false;

  @override
  void initState() {
    super.initState();
    _lastOpacity = widget.opacity;
  }

  @override
  void dispose() {
    _cachedImage?.dispose();
    super.dispose();
  }

  Future<void> _processCostMapData(OccupancyMap costMap) async {
    // 检查是否需要重新处理
    if (_lastProcessedMap == costMap && _lastOpacity == widget.opacity && _cachedImage != null) {
      return;
    }

    if (_isProcessing) return;
    _isProcessing = true;

    try {
      if (costMap.data.isEmpty || costMap.mapConfig.width == 0 || costMap.mapConfig.height == 0) {
        _cachedImage?.dispose();
        _cachedImage = null;
        _lastProcessedMap = costMap;
        _lastOpacity = widget.opacity;
        return;
      }

      final int width = costMap.mapConfig.width;
      final int height = costMap.mapConfig.height;

      // 获取代价地图的颜色数据
      List<int> costMapColors = costMap.getCostMapData();
      
      // 应用透明度到像素数据
      for (int i = 0; i < costMapColors.length; i += 4) {
        if (i + 3 < costMapColors.length) {
          // 应用透明度到alpha通道
          costMapColors[i + 3] = (costMapColors[i + 3] * widget.opacity).round();
        }
      }
      
      // 创建图像数据
      final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(
        Uint8List.fromList(costMapColors),
      );
      
      // 创建图像描述符
      final ui.ImageDescriptor descriptor = ui.ImageDescriptor.raw(
        buffer,
        width: width,
        height: height,
        pixelFormat: ui.PixelFormat.rgba8888,
      );
      
      // 解码图像
      final ui.Codec codec = await descriptor.instantiateCodec();
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      final ui.Image image = frameInfo.image;
      
      // 释放资源
      buffer.dispose();
      descriptor.dispose();
      codec.dispose();
      
      // 更新缓存
      _cachedImage?.dispose();
      _cachedImage = image;
      _lastProcessedMap = costMap;
      _lastOpacity = widget.opacity;
      
      if (mounted) {
        setState(() {});
      }
    } catch (e) {
      print('Error processing costmap: $e');
    } finally {
      _isProcessing = false;
    }
  }

  @override
  Widget build(BuildContext context) {
    return RepaintBoundary(
      child: ValueListenableBuilder<OccupancyMap>(
        valueListenable: widget.isGlobal 
          ? Provider.of<RosChannel>(context, listen: true).globalCostmap
          : Provider.of<RosChannel>(context, listen: true).localCostmap,
        builder: (context, costMap, child) {
          _processCostMapData(costMap);
          
          return Container(
            width: costMap.width().toDouble() + 1,
            height: costMap.height().toDouble() + 1,
            child: CustomPaint(
              painter: DisplayCostMapPainter(
                cachedImage: _cachedImage,
              ),
            ),
          );
        },
      ),
    );
  }
}

class DisplayCostMapPainter extends CustomPainter {
  final ui.Image? cachedImage;

  DisplayCostMapPainter({
    this.cachedImage,
  });

  @override
  void paint(Canvas canvas, Size size) {
    if (cachedImage == null) {
      return;
    }

    // 直接绘制缓存的图像
    canvas.drawImage(cachedImage!, Offset.zero, Paint());
  }

  @override
  bool shouldRepaint(covariant DisplayCostMapPainter oldDelegate) {
    return oldDelegate.cachedImage != cachedImage;
  }
}
