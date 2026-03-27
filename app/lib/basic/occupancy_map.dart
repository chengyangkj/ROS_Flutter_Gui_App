import 'package:flutter/material.dart';
import 'dart:math';
import 'dart:io';
import 'dart:convert';
import 'package:vector_math/vector_math_64.dart' as vm;

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


  //  // special values:
  //   cost_translation_table_[0] = 0;      // NO obstacle
  //   cost_translation_table_[253] = 99;   // INSCRIBED obstacle
  //   cost_translation_table_[254] = 100;  // LETHAL obstacle
  //   cost_translation_table_[255] = -1;   // UNKNOWN

  List<int> getCostMapData() {
    List<int> costMapData = [];
    
    for (int x = 0; x < mapConfig.height; x++) {
      for (int y = 0; y < mapConfig.width; y++) {
        // 计算像素值
        int pixelValue = data[x][y];
        //默认透明
        List<int> colorRgba = [0,0,0,0]; 
        
        //内切障碍，机器人内切半径，所能触及的区域，机器人进入此区域，一定会发生碰撞
        if(pixelValue==99){
          // colorRgba = [0x80, 0x80, 0x80, 10];// 浅灰色
        }
        else if(pixelValue==100){
          //实际障碍物点 致命障碍
          colorRgba = [0x80, 0x80, 0x80, 255];// 灰色
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
  vm.Vector2 idx2xy(vm.Vector2 occPoint) {
    double y =
        (height() - occPoint.y) * mapConfig.resolution + mapConfig.originY;
    double x = occPoint.x * mapConfig.resolution + mapConfig.originX;
    return vm.Vector2(x, y);
  }

  /**
   * @description: 输入全局坐标，返回栅格地图的行与列号
   * @return {*}
   */
  vm.Vector2 xy2idx(vm.Vector2 mapPoint) {
    double x = (mapPoint.x - mapConfig.originX) / mapConfig.resolution;
    double y =
        height() - (mapPoint.y - mapConfig.originY) / mapConfig.resolution;
    return vm.Vector2(x, y);
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
  
  // 创建深拷贝
  OccupancyMap copy() {
    OccupancyMap newMap = OccupancyMap();
    
    // 复制 MapConfig
    newMap.mapConfig.image = mapConfig.image;
    newMap.mapConfig.resolution = mapConfig.resolution;
    newMap.mapConfig.originX = mapConfig.originX;
    newMap.mapConfig.originY = mapConfig.originY;
    newMap.mapConfig.originTheta = mapConfig.originTheta;
    newMap.mapConfig.width = mapConfig.width;
    newMap.mapConfig.height = mapConfig.height;
    newMap.mapConfig.freeThresh = mapConfig.freeThresh;
    newMap.mapConfig.occupiedThresh = mapConfig.occupiedThresh;
    newMap.mapConfig.negate = mapConfig.negate;
    
    // 深拷贝数据数组
    newMap.data = List.generate(
      data.length,
      (i) => List<int>.from(data[i]),
    );
    
    return newMap;
  }
}