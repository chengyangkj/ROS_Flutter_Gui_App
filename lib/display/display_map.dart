import 'dart:ui' as ui;
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class DisplayMap extends StatefulWidget {
  const DisplayMap();

  @override
  _DisplayMapState createState() => _DisplayMapState();
}

class _DisplayMapState extends State<DisplayMap> {
  ui.Image? _cachedImage;
  OccupancyMap? _lastProcessedMap;
  bool _isProcessing = false;
  bool _lastIsDarkMode = false;

  @override
  void initState() {
    super.initState();
  }

  @override
  void dispose() {
    _cachedImage?.dispose();
    super.dispose();
  }

  Future<void> _processMapData(OccupancyMap map, bool isDarkMode) async {
    // 检查是否需要重新处理
    if (_lastProcessedMap == map && _lastIsDarkMode == isDarkMode && _cachedImage != null) {
      return;
    }

    if (_isProcessing) return;
    _isProcessing = true;

    try {
      if (map.data.isEmpty || map.mapConfig.width == 0 || map.mapConfig.height == 0) {
        _cachedImage?.dispose();
        _cachedImage = null;
        _lastProcessedMap = map;
        _lastIsDarkMode = isDarkMode;
        return;
      }

      final int width = map.mapConfig.width;
      final int height = map.mapConfig.height;

      // 创建像素数据缓冲区
      final List<int> pixelData = List.filled(width * height * 4, 0);
      
      // 处理地图数据
      for (int i = 0; i < map.Cols(); i++) {
        for (int j = 0; j < map.Rows(); j++) {
          int mapValue = map.data[j][i];
          
          if (mapValue > 0) {
            int alpha = (mapValue * 2.55).clamp(0, 255).toInt();
            final int pixelIndex = (j * width + i) * 4;
            
            // 设置RGBA值
            if (isDarkMode) {
              // 暗色主题：白色
              pixelData[pixelIndex] = 255;     // R
              pixelData[pixelIndex + 1] = 255; // G
              pixelData[pixelIndex + 2] = 255; // B
              pixelData[pixelIndex + 3] = alpha; // A
            } else {
              // 亮色主题：黑色
              pixelData[pixelIndex] = 0;       // R
              pixelData[pixelIndex + 1] = 0;   // G
              pixelData[pixelIndex + 2] = 0;   // B
              pixelData[pixelIndex + 3] = alpha; // A
            }
          }
        }
      }
      
      // 创建图像数据
      final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(
        Uint8List.fromList(pixelData),
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
      _lastProcessedMap = map;
      _lastIsDarkMode = isDarkMode;
      
      if (mounted) {
        setState(() {});
      }
    } catch (e) {
      print('Error processing map: $e');
    } finally {
      _isProcessing = false;
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;
    
    return RepaintBoundary(
        child: ValueListenableBuilder<OccupancyMap>(
            valueListenable: Provider.of<RosChannel>(context, listen: true).map,
            builder: (context, occMap, child) {
              _processMapData(occMap, isDarkMode);
              return Container(
                width: occMap.width().toDouble() + 1,
                height: occMap.height().toDouble() + 1,
                child: CustomPaint(
                  painter: DisplayMapPainter(
                    cachedImage: _cachedImage,
                  ),
                ),
              );
            }));
  }
}

class DisplayMapPainter extends CustomPainter {
  final ui.Image? cachedImage;

  DisplayMapPainter({
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
  bool shouldRepaint(covariant DisplayMapPainter oldDelegate) {
    return oldDelegate.cachedImage != cachedImage;
  }
}
