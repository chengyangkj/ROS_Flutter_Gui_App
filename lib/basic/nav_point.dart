import 'dart:convert';

enum NavPointType {
  navGoal,
  chargeStation
}

class NavPoint {
  double x; // 地图坐标
  double y; // 地图坐标
  double theta; // 方向角度（弧度）
  String name;
  NavPointType type;

  NavPoint({
    required this.x,
    required this.y,
    required this.theta,
    required this.name,
    required this.type,
  });

  // 从JSON创建NavPoint
  factory NavPoint.fromJson(Map<String, dynamic> json) {
    return NavPoint(
      x: json['x'] as double,
      y: json['y'] as double,
      theta: json['theta'] as double,
      name: json['name'] as String,
      type: NavPointType.values[json['type'] as int],
    );
  }

  // 转换为JSON
  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'theta': theta,
      'name': name,
      'type': type.index,
    };
  }

  // 复制并修改某些属性
  NavPoint copyWith({
    double? x,
    double? y,
    double? theta,
    String? name,
    DateTime? createdAt,
  }) {
    return NavPoint(
      x: x ?? this.x,
      y: y ?? this.y,
      theta: theta ?? this.theta,
      name: name ?? this.name,
      type: type ?? this.type,
    );
  }

  @override
  String toString() {
    return 'NavPoint(x: $x, y: $y, theta: $theta, name: $name, type: $type)';
  }
}
