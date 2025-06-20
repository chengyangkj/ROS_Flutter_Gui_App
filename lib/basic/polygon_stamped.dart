import 'package:ros_flutter_gui_app/basic/robot_path.dart';
import 'package:ros_flutter_gui_app/basic/transform.dart';

/// 表示一个点
class Point32 {
  final double x;
  final double y;
  final double z;

  Point32({required this.x, required this.y, required this.z});

  factory Point32.fromJson(Map<String, dynamic> json) {
    return Point32(
      x: json['x']?.toDouble() ?? 0.0,
      y: json['y']?.toDouble() ?? 0.0,
      z: json['z']?.toDouble() ?? 0.0,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'z': z,
    };
  }

  @override
  String toString() {
    return 'Point32(x: $x, y: $y, z: $z)';
  }
}

/// 表示一个多边形
class Polygon {
  final List<Point32> points;

  Polygon({required this.points});

  factory Polygon.fromJson(Map<String, dynamic> json) {
    List<Point32> pointsList = [];
    if (json['points'] != null) {
      pointsList = (json['points'] as List)
          .map((point) => Point32.fromJson(point))
          .toList();
    }
    return Polygon(points: pointsList);
  }

  Map<String, dynamic> toJson() {
    return {
      'points': points.map((point) => point.toJson()).toList(),
    };
  }

  @override
  String toString() {
    return 'Polygon(points: $points)';
  }
}

/// 表示带时间戳的多边形
class PolygonStamped {
  final Header? header;
  final Polygon? polygon;

  PolygonStamped({this.header, this.polygon});

  factory PolygonStamped.fromJson(Map<String, dynamic> json) {
    return PolygonStamped(
      header: json['header'] != null ? Header.fromJson(json['header']) : null,
      polygon: json['polygon'] != null ? Polygon.fromJson(json['polygon']) : null,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'header': header?.toJson(),
      'polygon': polygon?.toJson(),
    };
  }

  @override
  String toString() {
    return 'PolygonStamped(header: $header, polygon: $polygon)';
  }
}
