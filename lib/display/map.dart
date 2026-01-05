import 'dart:ui' as ui;
import 'dart:typed_data';
import 'package:flutter/material.dart';

import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

import 'package:flame/components.dart';
import 'package:flame/sprite.dart';

const int OBSTACLE_COLOR = 100;
const int FREE_COLOR = 0;
const int UNKNOWN_COLOR = -1;

class MapComponent extends SpriteComponent {
  OccupancyMap? _currentMap;
  RosChannel? _rosChannel;
  bool _isDarkMode = false;
  
  Uint8List? _pixelBuffer;
  int _bufferWidth = 0;
  int _bufferHeight = 0;
  
  // 公开访问当前地图数据
  OccupancyMap? get currentMap => _currentMap;
  
  // 构造函数接收RosChannel
  MapComponent({RosChannel? rosChannel}) {
    _rosChannel = rosChannel;
  }
  
  @override
  Future<void> onLoad() async {
    await super.onLoad();
    
    // 创建一个占位的透明 sprite，避免 SpriteComponent 断言失败
    final placeholderImage = await _createPlaceholderImage();
    sprite = Sprite(placeholderImage);
    
    // 如果有RosChannel，立即设置监听器
    if (_rosChannel != null) {
      _setupMapListener();
    }
  }
  
  // 创建一个 1x1 像素的透明占位图像
  Future<ui.Image> _createPlaceholderImage() async {
    final recorder = ui.PictureRecorder();
    final canvas = Canvas(recorder);
    final paint = Paint()..color = const Color.fromARGB(0, 0, 0, 0);
    canvas.drawRect(const Rect.fromLTWH(0, 0, 1, 1), paint);
    final picture = recorder.endRecording();
    return await picture.toImage(1, 1);
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
  
  OccupancyMap? getMapData() {
    return _currentMap;
  }
  
  void updateMapData(OccupancyMap map) async {
    if (_currentMap == map) return;
    
    _currentMap = map;
    await _processMapToSprite(map);
  }
  
  void updateThemeMode(bool isDarkMode) {
    _isDarkMode = isDarkMode;
    // 重新渲染地图以应用新的主题
    if (_currentMap != null) {
      _processMapToSprite(_currentMap!);
    }
  }
  
  void modifyCells(List<MapEntry<int, int>> cells, int value) {
    if (_currentMap == null || _pixelBuffer == null) {
      return;
    }
    
    for (var cell in cells) {
      final row = cell.key;
      final col = cell.value;
      if (row >= 0 && row < _currentMap!.Rows() && col >= 0 && col < _currentMap!.Cols()) {
        _currentMap!.data[row][col] = value;
        _updatePixel(row, col, value);
      }
    }
    
    _rebuildSpriteFromBuffer();
  }
  
  void _updatePixel(int row, int col, int mapValue) {
    if (_pixelBuffer == null) return;
    
    final int pixelIndex = (row * _bufferWidth + col) * 4;
    if (pixelIndex + 3 >= _pixelBuffer!.length) return;
    
    if (mapValue > 0) {
      int alpha = (mapValue * 2.55).clamp(0, 255).toInt();
      if (_isDarkMode) {
        _pixelBuffer![pixelIndex] = 200;
        _pixelBuffer![pixelIndex + 1] = 200;
        _pixelBuffer![pixelIndex + 2] = 200;
        _pixelBuffer![pixelIndex + 3] = alpha;
      } else {
        _pixelBuffer![pixelIndex] = 60;
        _pixelBuffer![pixelIndex + 1] = 60;
        _pixelBuffer![pixelIndex + 2] = 60;
        _pixelBuffer![pixelIndex + 3] = alpha;
      }
    } else if (mapValue == 0) {
      if (_isDarkMode) {
        _pixelBuffer![pixelIndex] = 30;
        _pixelBuffer![pixelIndex + 1] = 30;
        _pixelBuffer![pixelIndex + 2] = 30;
        _pixelBuffer![pixelIndex + 3] = 255;
      } else {
        _pixelBuffer![pixelIndex] = 255;
        _pixelBuffer![pixelIndex + 1] = 255;
        _pixelBuffer![pixelIndex + 2] = 255;
        _pixelBuffer![pixelIndex + 3] = 255;
      }
    } else {
      if (_isDarkMode) {
        _pixelBuffer![pixelIndex] = 80;
        _pixelBuffer![pixelIndex + 1] = 80;
        _pixelBuffer![pixelIndex + 2] = 80;
        _pixelBuffer![pixelIndex + 3] = 128;
      } else {
        _pixelBuffer![pixelIndex] = 200;
        _pixelBuffer![pixelIndex + 1] = 200;
        _pixelBuffer![pixelIndex + 2] = 200;
        _pixelBuffer![pixelIndex + 3] = 128;
      }
    }
  }
  
  Future<void> _rebuildSpriteFromBuffer() async {
    if (_pixelBuffer == null || _bufferWidth == 0 || _bufferHeight == 0) return;
    
    try {
      final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(_pixelBuffer!);
      final ui.ImageDescriptor descriptor = ui.ImageDescriptor.raw(
        buffer,
        width: _bufferWidth,
        height: _bufferHeight,
        pixelFormat: ui.PixelFormat.rgba8888,
      );
      final ui.Codec codec = await descriptor.instantiateCodec();
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      final ui.Image image = frameInfo.image;
      
      buffer.dispose();
      descriptor.dispose();
      codec.dispose();
      
      sprite = Sprite(image);
    } catch (e) {
      print('Error rebuilding sprite: $e');
    }
  }
  
  Future<void> _processMapToSprite(OccupancyMap map) async {
    try {
      if (map.data.isEmpty || map.mapConfig.width == 0 || map.mapConfig.height == 0) {
        sprite = null;
        _pixelBuffer = null;
        return;
      }

      final int width = map.mapConfig.width;
      final int height = map.mapConfig.height;

      if (_pixelBuffer == null || _bufferWidth != width || _bufferHeight != height) {
        _pixelBuffer = Uint8List(width * height * 4);
        _bufferWidth = width;
        _bufferHeight = height;
      }
      
      for (int i = 0; i < map.Cols(); i++) {
        for (int j = 0; j < map.Rows(); j++) {
          int mapValue = map.data[j][i];
          _updatePixel(j, i, mapValue);
        }
      }
      
      final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(_pixelBuffer!);
      final ui.ImageDescriptor descriptor = ui.ImageDescriptor.raw(
        buffer,
        width: width,
        height: height,
        pixelFormat: ui.PixelFormat.rgba8888,
      );
      final ui.Codec codec = await descriptor.instantiateCodec();
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      final ui.Image image = frameInfo.image;
      
      buffer.dispose();
      descriptor.dispose();
      codec.dispose();
      
      sprite = Sprite(image);
      size = Vector2(width.toDouble(), height.toDouble());
      position = Vector2(map.mapConfig.originX, map.mapConfig.originY);
      
    } catch (e) {
      print('Error processing map in Flame: $e');
    } 
  }
  
  @override
  void onRemove() {
    // 移除监听器
    if (_rosChannel != null) {
      _rosChannel!.map_.removeListener(_onMapDataChanged);
    }
    
    super.onRemove();
  }
}
