import 'package:flutter/material.dart';
import 'dart:math';
import 'dart:io';
import 'dart:convert';

class MapConfig {
  String image = "./";
  double resolution = 0.1;
  double originX = 0;
  double originY = 0;
  double originTheta = 0;
  int width = 0;
  int height = 0;
  double freeThresh = 0.196;
  double occupiedThresh = 0.65;
  int negate = 0;
  
  // 保存YAML配置文件
  void saveYaml(String filePath) {
    String yamlContent = '''
image: $image
resolution: ${resolution.toStringAsFixed(6)}
origin: [${originX.toStringAsFixed(6)}, ${originY.toStringAsFixed(6)}, ${originTheta.toStringAsFixed(6)}]
negate: $negate
occupied_thresh: ${occupiedThresh.toStringAsFixed(2)}
free_thresh: ${freeThresh.toStringAsFixed(3)}
''';
    
    File(filePath).writeAsStringSync(yamlContent);
  }
}

class OccupancyMap {
  MapConfig mapConfig = MapConfig();
  List<List<int>> data = [[]];
  int Rows() {
    return mapConfig.height;
  }

  int Cols() {
    return mapConfig.width;
  }

  int width() {
    return mapConfig.width;
  }

  int height() {
    return mapConfig.height;
  }

  double widthMap() { return mapConfig.width * mapConfig.resolution; }
  double heightMap() { return mapConfig.height * mapConfig.resolution; }

  void setFlip() {
    data = List.from(data.reversed);
  }

  void setZero() {
    for (int i = 0; i < data.length; i++) {
      for (int j = 0; j < data[i].length; j++) {
        data[i][j] = 0;
      }
    }
  }

  List<int> getCostMapData() {
    List<int> costMapData = [];
    
    for (int x = 0; x < mapConfig.height; x++) {
      for (int y = 0; y < mapConfig.width; y++) {
        // 计算像素值
        int pixelValue = data[x][y];
        //默认透明
        List<int> colorRgba = [0,0,0,0]; 
        
        if (pixelValue >= 100) {
          colorRgba = [0xff, 0x00, 0xff, 50]; // 紫色
        } else if (pixelValue >= 90 && pixelValue < 100) {
          colorRgba = [0x66, 0xff, 0xff, 50]; // 青色
        } else if (pixelValue >= 70 && pixelValue <= 90) {
          colorRgba = [0xff, 0x00, 0x33, 50]; // 深红色
        } else if (pixelValue >= 60 && pixelValue <= 70) {
          colorRgba = [0xbe, 0x28, 0x1a, 50]; // 红棕色
        } else if (pixelValue >= 50 && pixelValue < 60) {
          colorRgba = [0xBE, 0x1F, 0x58, 50]; // 粉红色
        } else if (pixelValue >= 40 && pixelValue < 50) {
          colorRgba = [0xBE, 0x25, 0x76, 50]; // 紫红色
        } else if (pixelValue >= 30 && pixelValue < 40) {
          colorRgba = [0xBE, 0x2A, 0x99, 50]; // 紫色
        } else if (pixelValue >= 20 && pixelValue < 30) {
          colorRgba = [0xBE, 0x35, 0xB3, 50]; // 深紫色
        } else if (pixelValue >= 10 && pixelValue < 20) {
          colorRgba = [0xB0, 0x3C, 0xbE, 50]; // 紫蓝色
        } else {
          // 其他值保持透明
          colorRgba =  [0,0,0,0]; 
        }
        
        // 将 RGBA 值添加到结果数组
        costMapData.addAll(colorRgba);
      }
    }
    
    return costMapData;
  }

  /**
   * @description: 输入栅格地图的坐标，返回该位置的全局坐标
   * @return {*}
   */
  Offset idx2xy(Offset occPoint) {
    double y =
        (height() - occPoint.dy) * mapConfig.resolution + mapConfig.originY;
    double x = occPoint.dx * mapConfig.resolution + mapConfig.originX;
    return Offset(x, y);
  }

  /**
   * @description: 输入全局坐标，返回栅格地图的行与列号
   * @return {*}
   */
  Offset xy2idx(Offset mapPoint) {
    double x = (mapPoint.dx - mapConfig.originX) / mapConfig.resolution;
    double y =
        height() - (mapPoint.dy - mapConfig.originY) / mapConfig.resolution;
    return Offset(x, y);
  }

  void saveMap(String path) {
    String mapdatafile = path + ".pgm";
    print("Writing map occupancy data to $mapdatafile");
    
    // 创建PGM文件头
    String header = "P5\n# CREATOR: map_saver.cpp ${mapConfig.resolution.toStringAsFixed(3)} m/pix\n${width()} ${height()}\n255\n";
    
    // 创建二进制数据
    List<int> binaryData = [];
    for (int y = 0; y < height(); y++) {
      for (int x = 0; x < width(); x++) {
        int mapValue = data[y][x];
        int pixelValue;
        
        if (mapValue >= 0 && mapValue <= (mapConfig.freeThresh * 100).round()) {
          // 自由区域 [0, free_thresh)
          pixelValue = 254;
        } else if (mapValue >= (mapConfig.occupiedThresh * 100).round()) {
          // 占据区域 (occupied_thresh, 255]
          pixelValue = 0;
        } else {
          // 未知区域 [free_thresh, occupied_thresh]
          pixelValue = 205;
        }
        
        binaryData.add(pixelValue);
      }
    }
    
    // 写入文件：先写入文本头部，再写入二进制数据
    File file = File(mapdatafile);
    file.writeAsStringSync(header);
    file.writeAsBytesSync(binaryData, mode: FileMode.append);
    
    // 保存YAML配置文件
    String mapmetadatafile = path + ".yaml";
    print("Writing map metadata to $mapmetadatafile");
    
    // 设置图片路径
    String fileName = path.split('/').last;
    mapConfig.image = "./$fileName.pgm";
    
    // 保存YAML配置
    mapConfig.saveYaml(mapmetadatafile);
  }
  
  // 便捷保存方法，指定目录和文件名
  void saveMapToDirectory(String directory, String fileName) {
    // 确保目录存在
    Directory dir = Directory(directory);
    if (!dir.existsSync()) {
      dir.createSync(recursive: true);
    }
    
    String fullPath = '$directory/$fileName';
    saveMap(fullPath);
  }
  
  // 保存到默认目录
  void saveMapDefault(String fileName) {
    String defaultDir = './maps';
    saveMapToDirectory(defaultDir, fileName);
  }
}