// This is a generated file - do not edit.
//
// Generated from topology_msgs.proto.

// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_relative_imports

import 'dart:core' as $core;

import 'package:protobuf/protobuf.dart' as $pb;

export 'package:protobuf/protobuf.dart' show GeneratedMessageGenericExtensions;

class TopologyRouteInfoPb extends $pb.GeneratedMessage {
  factory TopologyRouteInfoPb({
    $core.String? controller,
    $core.String? goalChecker,
    $core.double? speedLimit,
  }) {
    final result = create();
    if (controller != null) result.controller = controller;
    if (goalChecker != null) result.goalChecker = goalChecker;
    if (speedLimit != null) result.speedLimit = speedLimit;
    return result;
  }

  TopologyRouteInfoPb._();

  factory TopologyRouteInfoPb.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TopologyRouteInfoPb.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TopologyRouteInfoPb',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'controller')
    ..aOS(2, _omitFieldNames ? '' : 'goalChecker')
    ..aD(3, _omitFieldNames ? '' : 'speedLimit', fieldType: $pb.PbFieldType.OF)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyRouteInfoPb clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyRouteInfoPb copyWith(void Function(TopologyRouteInfoPb) updates) =>
      super.copyWith((message) => updates(message as TopologyRouteInfoPb))
          as TopologyRouteInfoPb;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TopologyRouteInfoPb create() => TopologyRouteInfoPb._();
  @$core.override
  TopologyRouteInfoPb createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TopologyRouteInfoPb getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TopologyRouteInfoPb>(create);
  static TopologyRouteInfoPb? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get controller => $_getSZ(0);
  @$pb.TagNumber(1)
  set controller($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasController() => $_has(0);
  @$pb.TagNumber(1)
  void clearController() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get goalChecker => $_getSZ(1);
  @$pb.TagNumber(2)
  set goalChecker($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasGoalChecker() => $_has(1);
  @$pb.TagNumber(2)
  void clearGoalChecker() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.double get speedLimit => $_getN(2);
  @$pb.TagNumber(3)
  set speedLimit($core.double value) => $_setFloat(2, value);
  @$pb.TagNumber(3)
  $core.bool hasSpeedLimit() => $_has(2);
  @$pb.TagNumber(3)
  void clearSpeedLimit() => $_clearField(3);
}

class TopologyRoutePb extends $pb.GeneratedMessage {
  factory TopologyRoutePb({
    $core.String? fromPoint,
    $core.String? toPoint,
    TopologyRouteInfoPb? routeInfo,
  }) {
    final result = create();
    if (fromPoint != null) result.fromPoint = fromPoint;
    if (toPoint != null) result.toPoint = toPoint;
    if (routeInfo != null) result.routeInfo = routeInfo;
    return result;
  }

  TopologyRoutePb._();

  factory TopologyRoutePb.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TopologyRoutePb.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TopologyRoutePb',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'fromPoint')
    ..aOS(2, _omitFieldNames ? '' : 'toPoint')
    ..aOM<TopologyRouteInfoPb>(3, _omitFieldNames ? '' : 'routeInfo',
        subBuilder: TopologyRouteInfoPb.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyRoutePb clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyRoutePb copyWith(void Function(TopologyRoutePb) updates) =>
      super.copyWith((message) => updates(message as TopologyRoutePb))
          as TopologyRoutePb;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TopologyRoutePb create() => TopologyRoutePb._();
  @$core.override
  TopologyRoutePb createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TopologyRoutePb getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TopologyRoutePb>(create);
  static TopologyRoutePb? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get fromPoint => $_getSZ(0);
  @$pb.TagNumber(1)
  set fromPoint($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasFromPoint() => $_has(0);
  @$pb.TagNumber(1)
  void clearFromPoint() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get toPoint => $_getSZ(1);
  @$pb.TagNumber(2)
  set toPoint($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasToPoint() => $_has(1);
  @$pb.TagNumber(2)
  void clearToPoint() => $_clearField(2);

  @$pb.TagNumber(3)
  TopologyRouteInfoPb get routeInfo => $_getN(2);
  @$pb.TagNumber(3)
  set routeInfo(TopologyRouteInfoPb value) => $_setField(3, value);
  @$pb.TagNumber(3)
  $core.bool hasRouteInfo() => $_has(2);
  @$pb.TagNumber(3)
  void clearRouteInfo() => $_clearField(3);
  @$pb.TagNumber(3)
  TopologyRouteInfoPb ensureRouteInfo() => $_ensure(2);
}

class TopologyPointPb extends $pb.GeneratedMessage {
  factory TopologyPointPb({
    $core.double? x,
    $core.double? y,
    $core.double? theta,
    $core.String? name,
    $core.String? type,
  }) {
    final result = create();
    if (x != null) result.x = x;
    if (y != null) result.y = y;
    if (theta != null) result.theta = theta;
    if (name != null) result.name = name;
    if (type != null) result.type = type;
    return result;
  }

  TopologyPointPb._();

  factory TopologyPointPb.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TopologyPointPb.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TopologyPointPb',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aD(1, _omitFieldNames ? '' : 'x')
    ..aD(2, _omitFieldNames ? '' : 'y')
    ..aD(3, _omitFieldNames ? '' : 'theta')
    ..aOS(4, _omitFieldNames ? '' : 'name')
    ..aOS(5, _omitFieldNames ? '' : 'type')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyPointPb clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyPointPb copyWith(void Function(TopologyPointPb) updates) =>
      super.copyWith((message) => updates(message as TopologyPointPb))
          as TopologyPointPb;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TopologyPointPb create() => TopologyPointPb._();
  @$core.override
  TopologyPointPb createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TopologyPointPb getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TopologyPointPb>(create);
  static TopologyPointPb? _defaultInstance;

  @$pb.TagNumber(1)
  $core.double get x => $_getN(0);
  @$pb.TagNumber(1)
  set x($core.double value) => $_setDouble(0, value);
  @$pb.TagNumber(1)
  $core.bool hasX() => $_has(0);
  @$pb.TagNumber(1)
  void clearX() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.double get y => $_getN(1);
  @$pb.TagNumber(2)
  set y($core.double value) => $_setDouble(1, value);
  @$pb.TagNumber(2)
  $core.bool hasY() => $_has(1);
  @$pb.TagNumber(2)
  void clearY() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.double get theta => $_getN(2);
  @$pb.TagNumber(3)
  set theta($core.double value) => $_setDouble(2, value);
  @$pb.TagNumber(3)
  $core.bool hasTheta() => $_has(2);
  @$pb.TagNumber(3)
  void clearTheta() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.String get name => $_getSZ(3);
  @$pb.TagNumber(4)
  set name($core.String value) => $_setString(3, value);
  @$pb.TagNumber(4)
  $core.bool hasName() => $_has(3);
  @$pb.TagNumber(4)
  void clearName() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.String get type => $_getSZ(4);
  @$pb.TagNumber(5)
  set type($core.String value) => $_setString(4, value);
  @$pb.TagNumber(5)
  $core.bool hasType() => $_has(4);
  @$pb.TagNumber(5)
  void clearType() => $_clearField(5);
}

class TopologyMapPropertyPb extends $pb.GeneratedMessage {
  factory TopologyMapPropertyPb({
    $core.Iterable<$core.String>? supportControllers,
    $core.Iterable<$core.String>? supportGoalCheckers,
  }) {
    final result = create();
    if (supportControllers != null)
      result.supportControllers.addAll(supportControllers);
    if (supportGoalCheckers != null)
      result.supportGoalCheckers.addAll(supportGoalCheckers);
    return result;
  }

  TopologyMapPropertyPb._();

  factory TopologyMapPropertyPb.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TopologyMapPropertyPb.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TopologyMapPropertyPb',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..pPS(1, _omitFieldNames ? '' : 'supportControllers')
    ..pPS(2, _omitFieldNames ? '' : 'supportGoalCheckers')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyMapPropertyPb clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyMapPropertyPb copyWith(
          void Function(TopologyMapPropertyPb) updates) =>
      super.copyWith((message) => updates(message as TopologyMapPropertyPb))
          as TopologyMapPropertyPb;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TopologyMapPropertyPb create() => TopologyMapPropertyPb._();
  @$core.override
  TopologyMapPropertyPb createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TopologyMapPropertyPb getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TopologyMapPropertyPb>(create);
  static TopologyMapPropertyPb? _defaultInstance;

  @$pb.TagNumber(1)
  $pb.PbList<$core.String> get supportControllers => $_getList(0);

  @$pb.TagNumber(2)
  $pb.PbList<$core.String> get supportGoalCheckers => $_getList(1);
}

class TopologyMapPb extends $pb.GeneratedMessage {
  factory TopologyMapPb({
    $core.String? mapName,
    TopologyMapPropertyPb? mapProperty,
    $core.Iterable<TopologyPointPb>? points,
    $core.Iterable<TopologyRoutePb>? routes,
  }) {
    final result = create();
    if (mapName != null) result.mapName = mapName;
    if (mapProperty != null) result.mapProperty = mapProperty;
    if (points != null) result.points.addAll(points);
    if (routes != null) result.routes.addAll(routes);
    return result;
  }

  TopologyMapPb._();

  factory TopologyMapPb.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TopologyMapPb.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TopologyMapPb',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'mapName')
    ..aOM<TopologyMapPropertyPb>(2, _omitFieldNames ? '' : 'mapProperty',
        subBuilder: TopologyMapPropertyPb.create)
    ..pPM<TopologyPointPb>(3, _omitFieldNames ? '' : 'points',
        subBuilder: TopologyPointPb.create)
    ..pPM<TopologyRoutePb>(4, _omitFieldNames ? '' : 'routes',
        subBuilder: TopologyRoutePb.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyMapPb clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TopologyMapPb copyWith(void Function(TopologyMapPb) updates) =>
      super.copyWith((message) => updates(message as TopologyMapPb))
          as TopologyMapPb;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TopologyMapPb create() => TopologyMapPb._();
  @$core.override
  TopologyMapPb createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TopologyMapPb getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TopologyMapPb>(create);
  static TopologyMapPb? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get mapName => $_getSZ(0);
  @$pb.TagNumber(1)
  set mapName($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasMapName() => $_has(0);
  @$pb.TagNumber(1)
  void clearMapName() => $_clearField(1);

  @$pb.TagNumber(2)
  TopologyMapPropertyPb get mapProperty => $_getN(1);
  @$pb.TagNumber(2)
  set mapProperty(TopologyMapPropertyPb value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasMapProperty() => $_has(1);
  @$pb.TagNumber(2)
  void clearMapProperty() => $_clearField(2);
  @$pb.TagNumber(2)
  TopologyMapPropertyPb ensureMapProperty() => $_ensure(1);

  @$pb.TagNumber(3)
  $pb.PbList<TopologyPointPb> get points => $_getList(2);

  @$pb.TagNumber(4)
  $pb.PbList<TopologyRoutePb> get routes => $_getList(3);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
