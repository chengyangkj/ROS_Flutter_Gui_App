import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'dart:math' as math;
import 'dart:ui' as ui;
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/display/pose_marker.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';

typedef WorldToLatLngFn = LatLng Function(double worldX, double worldY);

String _ConnectionKey(String from, String to) {
  final sorted = [from, to]..sort();
  return '${sorted[0]}_${sorted[1]}';
}

class _RouteArrow extends StatelessWidget {
  final double size;
  final Color color;

  const _RouteArrow({
    required this.size,
    required this.color,
  });

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: Size.square(size),
      painter: _RouteArrowPainter(color: color),
    );
  }
}

class _RouteArrowPainter extends CustomPainter {
  final Color color;

  _RouteArrowPainter({required this.color});

  @override
  void paint(Canvas canvas, Size size) {
    final w = size.width;
    final h = size.height;
    final center = Offset(w * 0.5, h * 0.5);

    final forward = Offset(w * 0.4, 0);
    final back = Offset(-w * 0.2, 0);
    final baseHalf = math.min(w, h) * 0.35;
    final perp = Offset(0, baseHalf.clamp(0.0, 1.8));

    final path = ui.Path()
      ..moveTo(center.dx + forward.dx, center.dy + forward.dy)
      ..lineTo(center.dx + back.dx + perp.dx, center.dy + back.dy + perp.dy)
      ..lineTo(center.dx + back.dx - perp.dx, center.dy + back.dy - perp.dy)
      ..close();

    final paint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant _RouteArrowPainter oldDelegate) => oldDelegate.color != color;
}

Widget buildTopologyLineLayer(
  List<NavPoint> points,
  List<TopologyRoute> routes,
  WorldToLatLngFn worldToLatLng, {
  void Function(NavPoint?)? onNavPointTap,
  ValueChanged<TopologyRoute>? onRouteTap,
  bool isEditMode = false,
  ValueChanged<NavPoint>? onNavPointChanged,
  void Function(NavPoint point, Offset delta)? onNavPointMoveDelta,
  ValueChanged<NavPoint>? onNavPointThetaEnd,
  ValueChanged<NavPoint>? onNavPointMoveEnd,
  String? selectedNavPointName,
  TopologyRoute? selectedRoute,
  bool enlargeNavPointMarkers = false,
}) {
  return _TopologyLineLayer(
    points: points,
    routes: routes,
    worldToLatLng: worldToLatLng,
    onNavPointTap: onNavPointTap,
    onRouteTap: onRouteTap,
    isEditMode: isEditMode,
    enlargeNavPointMarkers: enlargeNavPointMarkers,
    onNavPointChanged: onNavPointChanged,
    onNavPointMoveDelta: onNavPointMoveDelta,
    onNavPointThetaEnd: onNavPointThetaEnd,
    onNavPointMoveEnd: onNavPointMoveEnd,
    selectedNavPointName: selectedNavPointName,
    selectedRoute: selectedRoute,
  );
}

class _TopologyLineLayer extends StatefulWidget {
  final List<NavPoint> points;
  final List<TopologyRoute> routes;
  final WorldToLatLngFn worldToLatLng;
  final void Function(NavPoint?)? onNavPointTap;
  final ValueChanged<TopologyRoute>? onRouteTap;
  final bool isEditMode;
  final bool enlargeNavPointMarkers;
  final ValueChanged<NavPoint>? onNavPointChanged;
  final void Function(NavPoint point, Offset delta)? onNavPointMoveDelta;
  final ValueChanged<NavPoint>? onNavPointThetaEnd;
  final ValueChanged<NavPoint>? onNavPointMoveEnd;
  final String? selectedNavPointName;
  final TopologyRoute? selectedRoute;

  const _TopologyLineLayer({
    required this.points,
    required this.routes,
    required this.worldToLatLng,
    this.onNavPointTap,
    this.onRouteTap,
    required this.isEditMode,
    this.enlargeNavPointMarkers = false,
    this.onNavPointChanged,
    this.onNavPointMoveDelta,
    this.onNavPointThetaEnd,
    this.onNavPointMoveEnd,
    this.selectedNavPointName,
    this.selectedRoute,
  });

  @override
  State<_TopologyLineLayer> createState() => _TopologyLineLayerState();
}

class _TopologyLineLayerState extends State<_TopologyLineLayer> {
  @override
  Widget build(BuildContext context) {
    final polylines = <Polyline>[];
    final routeMarkers = <Marker>[];
    final pointByName = <String, NavPoint>{for (final p in widget.points) p.name: p};

    final connectionMap = <String, List<TopologyRoute>>{};
    for (final r in widget.routes) {
      final key = _ConnectionKey(r.fromPoint, r.toPoint);
      (connectionMap[key] ??= []).add(r);
    }

    const routeColor = Color(0xFF2563EB);
    const selectedColor = Color(0xFFFF0000);
    const parallelOffsetMeters = 0.05;
    const arrowSpacingMeters = 0.55;
    const pointRadiusMeters = 0.10;
    const pipeInnerWidth = 5.0;
    const lineOpacity = 0.30;
    const arrowSize = 10.0;
    const arrowHitSize = 22.0;

    void addPipeAndFlowArrows({
      required double x1,
      required double y1,
      required double x2,
      required double y2,
      required TopologyRoute route,
      required double dirX,
      required double dirY,
      required double len,
    }) {
      final isSelected = widget.selectedRoute != null &&
          widget.selectedRoute!.fromPoint == route.fromPoint &&
          widget.selectedRoute!.toPoint == route.toPoint;
      final color = isSelected ? selectedColor : routeColor;

      final fromLatLng = widget.worldToLatLng(x1, y1);
      final toLatLng = widget.worldToLatLng(x2, y2);

      polylines.add(Polyline(
        points: [fromLatLng, toLatLng],
        color: color.withOpacity(isSelected ? 0.80 : lineOpacity),
        strokeWidth: pipeInnerWidth,
      ));

      final angle = -math.atan2(dirY, dirX);
      var current = arrowSpacingMeters * 0.5;
      while (current < len - arrowSpacingMeters * 0.2) {
        final px = x1 + dirX * current;
        final py = y1 + dirY * current;

        routeMarkers.add(Marker(
          point: widget.worldToLatLng(px, py),
          width: arrowHitSize,
          height: arrowHitSize,
          alignment: Alignment.center,
          child: GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTap: widget.onRouteTap != null ? () => widget.onRouteTap!(route) : null,
            child: Transform.rotate(
              angle: angle,
              child: Opacity(
                opacity: isSelected ? 0.95 : 0.55,
                child: _RouteArrow(size: arrowSize, color: color),
              ),
            ),
          ),
        ));

        current += arrowSpacingMeters;
      }
    }

    for (final entry in connectionMap.entries) {
      final routeList = entry.value;
      if (routeList.isEmpty) continue;

      final first = routeList.first;
      final fromPoint = pointByName[first.fromPoint];
      final toPoint = pointByName[first.toPoint];
      if (fromPoint == null || toPoint == null) continue;

      final dx = toPoint.x - fromPoint.x;
      final dy = toPoint.y - fromPoint.y;
      final distance = math.sqrt(dx * dx + dy * dy);
      if (distance < 1e-6) continue;

      final dirX = dx / distance;
      final dirY = dy / distance;
      final perpX = -dirY;
      final perpY = dirX;

      final shrink = math.min(pointRadiusMeters, distance * 0.25);
      final adjustedX1 = fromPoint.x + dirX * shrink;
      final adjustedY1 = fromPoint.y + dirY * shrink;
      final adjustedX2 = toPoint.x - dirX * shrink;
      final adjustedY2 = toPoint.y - dirY * shrink;

      final adjDx = adjustedX2 - adjustedX1;
      final adjDy = adjustedY2 - adjustedY1;
      final adjDistance = math.sqrt(adjDx * adjDx + adjDy * adjDy);
      if (adjDistance < 1e-6) continue;

      TopologyRoute? forwardRoute;
      TopologyRoute? backwardRoute;
      for (final r in routeList) {
        if (r.fromPoint == first.fromPoint && r.toPoint == first.toPoint) {
          forwardRoute = r;
        } else if (r.fromPoint == first.toPoint && r.toPoint == first.fromPoint) {
          backwardRoute = r;
        }
      }

      final isBidirectional = forwardRoute != null && backwardRoute != null;
      if (isBidirectional) {
        final ox = perpX * parallelOffsetMeters;
        final oy = perpY * parallelOffsetMeters;
        addPipeAndFlowArrows(
          x1: adjustedX1 + ox,
          y1: adjustedY1 + oy,
          x2: adjustedX2 + ox,
          y2: adjustedY2 + oy,
          route: forwardRoute,
          dirX: dirX,
          dirY: dirY,
          len: adjDistance,
        );
        addPipeAndFlowArrows(
          x1: adjustedX2 - ox,
          y1: adjustedY2 - oy,
          x2: adjustedX1 - ox,
          y2: adjustedY1 - oy,
          route: backwardRoute,
          dirX: -dirX,
          dirY: -dirY,
          len: adjDistance,
        );
      } else {
        addPipeAndFlowArrows(
          x1: adjustedX1,
          y1: adjustedY1,
          x2: adjustedX2,
          y2: adjustedY2,
          route: forwardRoute ?? first,
          dirX: dirX,
          dirY: dirY,
          len: adjDistance,
        );
      }
    }

    final navScale = widget.enlargeNavPointMarkers ? 1.5 : 1.0;
    final baseRobotSize = globalSetting.robotSize.toDouble();
    final poseSize = baseRobotSize * navScale;

    final markers = widget.points
        .map((p) => Marker(
            point: widget.worldToLatLng(p.x, p.y),
            width: (baseRobotSize + 90) * navScale,
            height: (baseRobotSize + 16) * navScale,
            alignment: Alignment.center,
            child: GestureDetector(
              onTap: widget.onNavPointTap != null ? () => widget.onNavPointTap!(p) : null,
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  PoseMarkerWidget(
                    size: poseSize,
                    theta: -p.theta,
                    type: PoseMarkerType.NavPoint,
                    color: widget.selectedNavPointName != null &&
                            widget.selectedNavPointName == p.name
                        ? Colors.red
                        : Colors.green,
                    isEditMode: widget.isEditMode,
                    onThetaChanged: widget.onNavPointChanged != null
                        ? (theta) {
                            widget.onNavPointChanged!(
                              NavPoint(
                                name: p.name,
                                x: p.x,
                                y: p.y,
                                theta: theta,
                                type: p.type,
                              ),
                            );
                          }
                        : null,
                    onThetaEnd: widget.onNavPointThetaEnd != null
                        ? (theta) => widget.onNavPointThetaEnd!(
                              NavPoint(
                                name: p.name,
                                x: p.x,
                                y: p.y,
                                theta: theta,
                                type: p.type,
                              ),
                            )
                        : null,
                    onMoveDelta: widget.onNavPointMoveDelta != null
                        ? (delta) => widget.onNavPointMoveDelta!(p, delta)
                        : null,
                    onMoveEnd: widget.onNavPointMoveEnd != null
                        ? () => widget.onNavPointMoveEnd!(p)
                        : null,
                  ),
                  SizedBox(height: 2 * navScale),
                  IgnorePointer(
                    child: Text(
                      p.name,
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: TextStyle(
                        fontSize: 8 * navScale,
                        color: Colors.white,
                        height: 1.0,
                        fontWeight: FontWeight.w600,
                        shadows: [
                          Shadow(
                            offset: Offset(0, 1),
                            blurRadius: 2,
                            color: Color(0xCC000000),
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ))
        .toList();

    return Stack(
      children: [
        if (polylines.isNotEmpty) PolylineLayer(polylines: polylines),
        if (routeMarkers.isNotEmpty) MarkerLayer(markers: routeMarkers),
        MarkerLayer(markers: markers),
      ],
    );
  }
}
