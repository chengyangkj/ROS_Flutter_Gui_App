import 'package:flutter/material.dart';
import 'dart:math';

class MapConfig {
  String image = "./";
  double resolution = 0.1;
  double originX = 0;
  double originY = 0;
  double originTheta = 0;
  int width = 0;
  int height = 0;
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

  void setFlip() {
    data = List.from(data.reversed);
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
}


// class OccupancyMap {
//  public:
//   MapConfig map_config;
//   int rows{0};               // 行(高)
//   int cols{0};               // 列(宽)
//   Eigen::MatrixXi map_data;  // 地图数据,数据的地图已经被上下翻转
//  public:
//   OccupancyMap() {}
//   OccupancyMap(int rows_, int cols_, Eigen::Vector3d origin, double res)
//       : map_config({origin[0], origin[1], origin[2]}, res), rows(rows_), cols(cols_), map_data(rows_, cols) {}
//   OccupancyMap(int rows_, int cols_, Eigen::Vector3d origin, double res,
//                Eigen::MatrixXi data)
//       : map_config({origin[0], origin[1], origin[2]}, res), rows(rows_), cols(cols_), map_data(data) {}
//   ~OccupancyMap() = default;
//   Eigen::MatrixXi GetMapData() { return map_data; }
//   Eigen::MatrixXi flip() { return map_data.colwise().reverse(); }
//   void SetFlip() { map_data = flip(); }
//   auto &operator()(int r, int c) { return map_data(r, c); }
//   // 输入原始栅格地图数据(地图未翻转)
//   void SetMapData(const Eigen::MatrixXi &data) {
//     map_data = data;
//     map_data = flip();
//   }
//   int Rows() { return rows; }
//   int Cols() { return cols; }
//   //宽高地图坐标系下的长度
//   int width() { return cols; }
//   int height() { return rows; }
//   //宽map坐标系下的长度
//   int widthMap() { return cols * map_config.resolution; }
//   int heightMap() { return rows * map_config.resolution; }
//   /**
//    * @description: 输入栅格地图的行与列号，返回该位置的全局坐标
//    * @param {int&} c 列号
//    * @param {int&} r 行号
//    * @param {double&} x x坐标
//    * @param {double&} y y坐标
//    * @return {*}
//    */
//   void idx2xy(const int &c, const int &r, double &x, double &y) {
//     x = map_config.origin[0] + c * map_config.resolution;
//     y = map_config.origin[1] + r * map_config.resolution;
//   }
//   /**
//    * @description: 输入全局坐标，返回栅格地图的行与列号
//    * @param {double&} x
//    * @param {double&} y
//    * @param {int&} c 列号map_y
//    * @param {int&} r 行号
//    * @return {*}
//    */
//   void xy2idx(const double &x, const double &y, int &c, int &r) {
//     c = round(x - map_config.origin[0]) / map_config.resolution;
//     r = round(y - map_config.origin[1]) / map_config.resolution;
//   }
//   /**
//    * @description:输入全局坐标，判断是否在栅格地图内
//    * @param {double&} x
//    * @param {double&} y
//    * @return {*}
//    */
//   bool inMap(const double &x, const double &y) {
//     int c_idx = round((x - map_config.origin[0]) / map_config.resolution);
//     int r_idx = round((y - map_config.origin[1]) / map_config.resolution);
//     return (c_idx >= 0 && r_idx >= 0 && c_idx < cols && r_idx < rows);
//   }
//   /**
//    * @description:输入行与列号，返回该位置是否在栅格地图内
//    * @param {int} r_idx
//    * @param {int} c_idx
//    * @return {*}
//    */
//   bool inMap(int r_idx, int c_idx) const {
//     return (c_idx >= 0 && r_idx >= 0 && c_idx < cols && r_idx < rows);
//   }
//   /**
//    * @description:输入原始(地图图片)图元坐标,返回世界坐标
//    * @return {*}
//    */
//   void ScenePose2xy(const double &scene_x, const double &scene_y,
//                     double &word_x, double &word_y) {
//     word_x = scene_x * map_config.resolution + map_config.origin[0];
//     word_y = (height() - scene_y) * map_config.resolution + map_config.origin[1];
//   }
//   /**
//    * @description:输入栅格坐标,返回世界坐标
//    * @return {*}
//    */
//   void OccPose2xy(const double &scene_x, const double &scene_y,
//                   double &word_x, double &word_y) {
//     word_y = scene_x * map_config.resolution + map_config.origin[0];
//     word_x = (height() - scene_y) * map_config.resolution + map_config.origin[1];
//   }
//   /**
//    * @description:
//    * 输入世界坐标,返回原始(地图图片)图元坐标。坐标已经进行了上下翻转
//    * @param {double&} word_x
//    * @param {double&} word_x
//    * @param {int&} scene_x
//    * @param {int&} scene_y
//    * @return {*}
//    */
//   void xy2ScenePose(const double &word_x, const double &word_y, double &scene_x,
//                     double &scene_y) {
//     scene_x = (word_x - map_config.origin[0]) / map_config.resolution;
//     scene_y = height() - (word_y - map_config.origin[1]) / map_config.resolution;
//   }
//   /**
//    * @description:
//    * 输入世界坐标,返回栅格坐标。坐标已经进行了上下翻转
//    * @param {double&} word_x
//    * @param {double&} word_x
//    * @param {int&} scene_x
//    * @param {int&} scene_y
//    * @return {*}
//    */
//   void xy2OccPose(const double &word_x, const double &word_y, double &scene_x,
//                   double &scene_y) {
//     scene_y = (word_x - map_config.origin[0]) / map_config.resolution;
//     scene_x = height() - (word_y - map_config.origin[1]) / map_config.resolution;
//   }
//   /**
//    * @description: 获取带rgba颜色值的代价地图
//    * @return {*}
//    */
//   Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic>
//   GetCostMapData() {
//     Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic> res =
//         Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic>(
//             map_data.rows(), map_data.cols());
//     for (int x = 0; x < map_data.rows(); x++) {
//       for (int y = 0; y < map_data.cols(); y++) {
//         // 计算像素值
//         Eigen::Vector4i color_rgba;
//         int data = map_data(x, y);
//         if (data >= 100) {
//           color_rgba = Eigen::Vector4i(0xff, 0x00, 0xff, 50);

//         } else if (data >= 90 && data < 100) {
//           color_rgba = Eigen::Vector4i(0x66, 0xff, 0xff, 50);

//         } else if (data >= 70 && data <= 90) {
//           color_rgba = Eigen::Vector4i(0xff, 0x00, 0x33, 50);

//         } else if (data >= 60 && data <= 70) {
//           color_rgba = Eigen::Vector4i(0xbe, 0x28, 0x1a, 50);

//         } else if (data >= 50 && data < 60) {
//           color_rgba = Eigen::Vector4i(0xBE, 0x1F, 0x58, 50);

//         } else if (data >= 40 && data < 50) {
//           color_rgba = Eigen::Vector4i(0xBE, 0x25, 0x76, 50);

//         } else if (data >= 30 && data < 40) {
//           color_rgba = Eigen::Vector4i(0xBE, 0x2A, 0x99, 50);

//         } else if (data >= 20 && data < 30) {
//           color_rgba = Eigen::Vector4i(0xBE, 0x35, 0xB3, 50);

//         } else if (data >= 10 && data < 20) {
//           color_rgba = Eigen::Vector4i(0xB0, 0x3C, 0xbE, 50);

//         } else {
//           // 其他 透明
//           color_rgba = Eigen::Vector4i(0, 0, 0, 0);
//         }
//         res(x, y) = color_rgba;
//       }
//     }
//     return res;
//   }
//   //保存地图到路径
//   void Save(std::string map_name) {
//     std::string mapdatafile = map_name + ".pgm";
//     printf("Writing map occupancy data to %s", mapdatafile.c_str());
//     FILE *out = fopen(mapdatafile.c_str(), "w");
//     if (!out) {
//       printf("Couldn't save map file to %s", mapdatafile.c_str());
//       return;
//     }

//     fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
//             map_config.resolution, width(), height());
//     for (unsigned int y = 0; y < height(); y++) {
//       for (unsigned int x = 0; x < width(); x++) {
//         // unsigned int i = x + (height() - y - 1) * map->info.width;
//         if (map_data(y, x) >= 0 && map_data(y, x) <= map_config.free_thresh) {  // [0,free)
//           fputc(254, out);
//         } else if (map_data(y, x) >= map_config.occupied_thresh) {  // (occ,255]
//           fputc(000, out);
//         } else {  //occ [0.25,0.65]
//           fputc(205, out);
//         }
//       }
//     }

//     fclose(out);

//     std::string mapmetadatafile = map_name + ".yaml";
//     printf("Writing map occupancy data to %s", mapmetadatafile.c_str());

//     /*
// map_config.resolution: 0.100000
// origin: [0.000000, 0.000000, 0.000000]
// #
// negate: 0
// occupied_thresh: 0.65
// free_thresh: 0.196

//        */
//     boost::filesystem::path filepath(map_name);
//     std::string map_name_rel = filepath.stem().string();
//     map_config.image = "./" + map_name_rel + ".pgm";
//     map_config.Save(mapmetadatafile);
//   }
//   bool Load(const std::string &yaml_path) {
//     //解析yaml
//     if (!map_config.Load(yaml_path)) return false;
//     //解析yaml
//     std::ifstream file(map_config.image);
//     if (file.is_open()) {
//       std::string line;
//       int width, height, maxVal;

//       // 读取 PGM 文件头信息
//       getline(file, line);      // 第一行是 "P5"，表示文件类型
//       getline(file, line);      // 第二行是注释，可以忽略
//       file >> width >> height;  // 第三行是宽度和高度
//       file >> maxVal;           // 第四行是最大像素值
//       //赋值行与列
//       rows = height;
//       cols = width;
//       LOG_INFO("reade from pgm width:" << width << " height:" << height << " maxVal:" << maxVal);
//       map_data = Eigen::MatrixXi(height, width);
//       // 读取地图数据
//       for (int i = 0; i < height; i++) {
//         for (int j = 0; j < width; j++) {
//           int8_t pixel = -2;
//           int8_t map_cell;
//           file >> pixel;
//           if (pixel == 0) {
//             map_cell = OCC_GRID_OCCUPIED;  //占用
//           } else if (pixel == 205) {
//             map_cell = OCC_GRID_UNKNOWN;
//           } else if (pixel == 254) {
//             map_cell = OCC_GRID_FREE;
//           } else {
//             map_cell = OCC_GRID_UNKNOWN;
//           }
//           map_data(i, j) = map_cell;
//         }
//       }
//       LOG_INFO("load map over");
//       file.close();
//     } else {
//       LOG_INFO("无法打开地图文件 " << map_config.image);
//       return true;
//     }
//     return true;
//   }
// };