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
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
      theta: (json['theta'] as num).toDouble(),
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
      'type': type.index,
    };
  }
}

class MapProperty {
  List<String> supportControllers;

  MapProperty({
    required this.supportControllers,
  });

  factory MapProperty.fromJson(Map<String, dynamic> json) {
    return MapProperty(
      supportControllers: (json['support_controllers'] as List<dynamic>)
          .map((e) => e as String)
          .toList(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'support_controllers': supportControllers,
    };
  }
}

class RouteInfo {
  String controller;

  RouteInfo({
    required this.controller,
  });

  factory RouteInfo.fromJson(Map<String, dynamic> json) {
    return RouteInfo(
      controller: json['controller'] as String,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'controller': controller,
    };
  }
}

class TopologyRoute {
  String fromPoint;
  String toPoint;
  RouteInfo routeInfo;

  TopologyRoute({
    required this.fromPoint,
    required this.toPoint,
    required this.routeInfo,
  });

  factory TopologyRoute.fromJson(Map<String, dynamic> json) {
    return TopologyRoute(
      fromPoint: json['from_point'] as String,
      toPoint: json['to_point'] as String,
      routeInfo: RouteInfo.fromJson(json['route_info'] as Map<String, dynamic>),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'from_point': fromPoint,
      'to_point': toPoint,
      'route_info': routeInfo.toJson(),
    };
  }
}

class TopologyMap {
  String mapName;
  MapProperty mapProperty;
  List<NavPoint> points;
  List<TopologyRoute> routes;

  TopologyMap({
    this.mapName = '',
    MapProperty? mapProperty,
    required this.points,
    List<TopologyRoute>? routes,
  })  : mapProperty = mapProperty ?? MapProperty(supportControllers: []),
        routes = routes ?? [];

  factory TopologyMap.fromJson(Map<String, dynamic> json) {
    return TopologyMap(
      mapName: json['map_name'] as String? ?? '',
      mapProperty: json['map_property'] != null
          ? MapProperty.fromJson(json['map_property'] as Map<String, dynamic>)
          : MapProperty(supportControllers: []),
      points: (json['points'] as List<dynamic>)
          .map((e) => NavPoint.fromJson(e as Map<String, dynamic>))
          .toList(),
      routes: json['routes'] != null
          ? (json['routes'] as List<dynamic>)
              .map((e) => TopologyRoute.fromJson(e as Map<String, dynamic>))
              .toList()
          : [],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'map_name': mapName,
      'map_property': mapProperty.toJson(),
      'points': points.map((e) => e.toJson()).toList(),
      'routes': routes.map((e) => e.toJson()).toList(),
    };
  }
}
