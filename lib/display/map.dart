import 'dart:ui' as ui;
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

import 'package:flame/components.dart';


class MapComponent extends PositionComponent {
  OccupancyMap? _currentMap;
  ui.Image? _mapImage;
  bool _isProcessing = false;
  RosChannel? _rosChannel;
  bool _isDarkMode = false;
  
  // 公开访问当前地图数据
  OccupancyMap? get currentMap => _currentMap;
  
  // 构造函数接收RosChannel
  MapComponent({RosChannel? rosChannel}) {
    _rosChannel = rosChannel;
  }
  
  @override
  Future<void> onLoad() async {
    super.onLoad();
    
    // 如果有RosChannel，立即设置监听器
    if (_rosChannel != null) {
      _setupMapListener();
    }
  }
  
  void _setupMapListener() {
    if (_rosChannel != null) {
      // 监听地图数据变化
      _rosChannel!.map_.addListener(_onMapDataChanged);
      
      // 立即更新当前地图数据
      _onMapDataChanged();
    }
  }
  
  void _onMapDataChanged() {
    if (_rosChannel != null) {
      final newMap = _rosChannel!.map_.value;
      updateMapData(newMap);
    }
  }
  
  void updateMapData(OccupancyMap map) async {
    if (_isProcessing || _currentMap == map) return;
    
    _currentMap = map;
    await _processMapToImage(map);
  }
  
  void updateThemeMode(bool isDarkMode) {
    if (_isDarkMode != isDarkMode) {
      _isDarkMode = isDarkMode;
      // 重新渲染地图以应用新的主题
      if (_currentMap != null) {
        _processMapToImage(_currentMap!);
      }
    }
  }
  
  Future<void> _processMapToImage(OccupancyMap map) async {
    if (_isProcessing) return;
    _isProcessing = true;
    
    try {
      if (map.data.isEmpty || map.mapConfig.width == 0 || map.mapConfig.height == 0) {
        _mapImage?.dispose();
        _mapImage = null;
        return;
      }

      final int width = map.mapConfig.width;
      final int height = map.mapConfig.height;

      // 创建像素数据缓冲区 (RGBA格式)
      final List<int> pixelData = List.filled(width * height * 4, 0);
      
      // 处理地图数据
      for (int i = 0; i < map.Cols(); i++) {
        for (int j = 0; j < map.Rows(); j++) {
          int mapValue = map.data[j][i];
          final int pixelIndex = (j * width + i) * 4;
          
          if (mapValue > 0) {
            // 占据区域
            int alpha = (mapValue * 2.55).clamp(0, 255).toInt();
            if (_isDarkMode) {
              // 暗色主题 - 浅灰色障碍物
              pixelData[pixelIndex] = 200;     // R
              pixelData[pixelIndex + 1] = 200; // G
              pixelData[pixelIndex + 2] = 200; // B
              pixelData[pixelIndex + 3] = alpha; // A
            } else {
              // 亮色主题 - 深灰色障碍物
              pixelData[pixelIndex] = 60;      // R
              pixelData[pixelIndex + 1] = 60;  // G
              pixelData[pixelIndex + 2] = 60;  // B
              pixelData[pixelIndex + 3] = alpha; // A
            }
          } else if (mapValue == 0) {
            // 自由区域
            if (_isDarkMode) {
              // 暗色主题 - 深色背景
              pixelData[pixelIndex] = 30;      // R
              pixelData[pixelIndex + 1] = 30;  // G
              pixelData[pixelIndex + 2] = 30;  // B
              pixelData[pixelIndex + 3] = 255; // A
            } else {
              // 亮色主题 - 白色背景
              pixelData[pixelIndex] = 255;     // R
              pixelData[pixelIndex + 1] = 255; // G
              pixelData[pixelIndex + 2] = 255; // B
              pixelData[pixelIndex + 3] = 255; // A
            }
          } else {
            // 未知区域
            if (_isDarkMode) {
              // 暗色主题 - 中等灰色
              pixelData[pixelIndex] = 80;      // R
              pixelData[pixelIndex + 1] = 80;  // G
              pixelData[pixelIndex + 2] = 80;  // B
              pixelData[pixelIndex + 3] = 128; // A
            } else {
              // 亮色主题 - 浅灰色
              pixelData[pixelIndex] = 200;     // R
              pixelData[pixelIndex + 1] = 200; // G
              pixelData[pixelIndex + 2] = 200; // B
              pixelData[pixelIndex + 3] = 128; // A
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
      
      // 更新图像
      _mapImage?.dispose();
      _mapImage = image;
      
    } catch (e) {
      print('Error processing map in Flame: $e');
    } finally {
      _isProcessing = false;
    }
  }
  
  @override
  void render(Canvas canvas) {
    super.render(canvas);
    
    if (_mapImage != null && _currentMap != null) {
      // 绘制地图图像
      canvas.drawImage(_mapImage!, Offset.zero, Paint());
    }
  }
  
  @override
  void onRemove() {
    // 移除监听器
    if (_rosChannel != null) {
      _rosChannel!.map_.removeListener(_onMapDataChanged);
    }
    
    // 释放图像资源
    _mapImage?.dispose();
    super.onRemove();
  }
}
