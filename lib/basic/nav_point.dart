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

  static NavPointType _parseType(dynamic v) {
    if (v is int) return NavPointType.values[v];
    final s = v.toString();
    switch (s) {
      case 'NavGoal':
      case 'navGoal':
        return NavPointType.navGoal;
      case 'ChargeStation':
      case 'chargeStation':
        return NavPointType.chargeStation;
      default:
        return NavPointType.navGoal;
    }
  }

  factory NavPoint.fromJson(Map<String, dynamic> json) {
    return NavPoint(
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
      theta: (json['theta'] as num).toDouble(),
      name: json['name'] as String,
      type: _parseType(json['type']),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'theta': theta,
      'name': name,
      'type': type == NavPointType.navGoal ? 'NavGoal' : 'ChargeStation',
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
      type: this.type,
    );
  }

  @override
  String toString() {
    return 'NavPoint(x: $x, y: $y, theta: $theta, name: $name, type: $type)';
  }
}
