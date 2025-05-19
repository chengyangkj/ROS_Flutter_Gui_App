enum NavPointType {
  navGoal,
  chargeStation
}

class NavPoint {
  double x;
  double y;
  double theta;
  String name;
  NavPointType type;

  NavPoint({
    required this.x,
    required this.y,
    required this.theta,
    required this.name,
    required this.type,
  });

  factory NavPoint.fromJson(Map<String, dynamic> json) {
    return NavPoint(
      x: json['x'] as double,
      y: json['y'] as double,
      theta: json['theta'] as double,
      name: json['name'] as String,
      type: NavPointType.values[json['type'] as int],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'theta': theta,
      'name': name,
      'type': type,
    };
  }
}

class TopologyMap {
  List<NavPoint> points;

  TopologyMap({
    required this.points,
  });

  factory TopologyMap.fromJson(Map<String, dynamic> json) {
    return TopologyMap(
      points: (json['points'] as List<dynamic>)
          .map((e) => NavPoint.fromJson(e as Map<String, dynamic>))
          .toList(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'points': points.map((e) => e.toJson()).toList(),
    };
  }
}
