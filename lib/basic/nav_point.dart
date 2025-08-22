import 'dart:convert';

class NavPoint {
  final String id;
  final double x;
  final double y;
  final double theta; // 方向角度（弧度）
  final String name;
  final DateTime createdAt;

  NavPoint({
    required this.id,
    required this.x,
    required this.y,
    required this.theta,
    required this.name,
    required this.createdAt,
  });

  // 从JSON创建NavPoint
  factory NavPoint.fromJson(Map<String, dynamic> json) {
    return NavPoint(
      id: json['id'] as String,
      x: json['x'] as double,
      y: json['y'] as double,
      theta: json['theta'] as double,
      name: json['name'] as String,
      createdAt: DateTime.parse(json['createdAt'] as String),
    );
  }

  // 转换为JSON
  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'x': x,
      'y': y,
      'theta': theta,
      'name': name,
      'createdAt': createdAt.toIso8601String(),
    };
  }

  // 复制并修改某些属性
  NavPoint copyWith({
    String? id,
    double? x,
    double? y,
    double? theta,
    String? name,
    DateTime? createdAt,
  }) {
    return NavPoint(
      id: id ?? this.id,
      x: x ?? this.x,
      y: y ?? this.y,
      theta: theta ?? this.theta,
      name: name ?? this.name,
      createdAt: createdAt ?? this.createdAt,
    );
  }

  @override
  String toString() {
    return 'NavPoint(id: $id, x: $x, y: $y, theta: $theta, name: $name)';
  }
}
