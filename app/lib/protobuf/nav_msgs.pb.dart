// This is a generated file - do not edit.
//
// Generated from nav_msgs.proto.

// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_relative_imports

import 'dart:core' as $core;

import 'package:protobuf/protobuf.dart' as $pb;

import 'geometry_msgs.pb.dart' as $0;

export 'package:protobuf/protobuf.dart' show GeneratedMessageGenericExtensions;

class MapMetaData extends $pb.GeneratedMessage {
  factory MapMetaData({
    $0.Time? mapLoadTime,
    $core.double? resolution,
    $core.int? width,
    $core.int? height,
    $0.Pose? origin,
  }) {
    final result = create();
    if (mapLoadTime != null) result.mapLoadTime = mapLoadTime;
    if (resolution != null) result.resolution = resolution;
    if (width != null) result.width = width;
    if (height != null) result.height = height;
    if (origin != null) result.origin = origin;
    return result;
  }

  MapMetaData._();

  factory MapMetaData.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory MapMetaData.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'MapMetaData',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Time>(1, _omitFieldNames ? '' : 'mapLoadTime',
        subBuilder: $0.Time.create)
    ..aD(2, _omitFieldNames ? '' : 'resolution', fieldType: $pb.PbFieldType.OF)
    ..aI(3, _omitFieldNames ? '' : 'width', fieldType: $pb.PbFieldType.OU3)
    ..aI(4, _omitFieldNames ? '' : 'height', fieldType: $pb.PbFieldType.OU3)
    ..aOM<$0.Pose>(5, _omitFieldNames ? '' : 'origin',
        subBuilder: $0.Pose.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  MapMetaData clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  MapMetaData copyWith(void Function(MapMetaData) updates) =>
      super.copyWith((message) => updates(message as MapMetaData))
          as MapMetaData;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static MapMetaData create() => MapMetaData._();
  @$core.override
  MapMetaData createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static MapMetaData getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<MapMetaData>(create);
  static MapMetaData? _defaultInstance;

  @$pb.TagNumber(1)
  $0.Time get mapLoadTime => $_getN(0);
  @$pb.TagNumber(1)
  set mapLoadTime($0.Time value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasMapLoadTime() => $_has(0);
  @$pb.TagNumber(1)
  void clearMapLoadTime() => $_clearField(1);
  @$pb.TagNumber(1)
  $0.Time ensureMapLoadTime() => $_ensure(0);

  @$pb.TagNumber(2)
  $core.double get resolution => $_getN(1);
  @$pb.TagNumber(2)
  set resolution($core.double value) => $_setFloat(1, value);
  @$pb.TagNumber(2)
  $core.bool hasResolution() => $_has(1);
  @$pb.TagNumber(2)
  void clearResolution() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get width => $_getIZ(2);
  @$pb.TagNumber(3)
  set width($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasWidth() => $_has(2);
  @$pb.TagNumber(3)
  void clearWidth() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.int get height => $_getIZ(3);
  @$pb.TagNumber(4)
  set height($core.int value) => $_setUnsignedInt32(3, value);
  @$pb.TagNumber(4)
  $core.bool hasHeight() => $_has(3);
  @$pb.TagNumber(4)
  void clearHeight() => $_clearField(4);

  @$pb.TagNumber(5)
  $0.Pose get origin => $_getN(4);
  @$pb.TagNumber(5)
  set origin($0.Pose value) => $_setField(5, value);
  @$pb.TagNumber(5)
  $core.bool hasOrigin() => $_has(4);
  @$pb.TagNumber(5)
  void clearOrigin() => $_clearField(5);
  @$pb.TagNumber(5)
  $0.Pose ensureOrigin() => $_ensure(4);
}

class OccupancyGridProto extends $pb.GeneratedMessage {
  factory OccupancyGridProto({
    $0.Header? header,
    MapMetaData? info,
    $core.String? frameId,
    $core.Iterable<$core.int>? data,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (info != null) result.info = info;
    if (frameId != null) result.frameId = frameId;
    if (data != null) result.data.addAll(data);
    return result;
  }

  OccupancyGridProto._();

  factory OccupancyGridProto.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory OccupancyGridProto.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'OccupancyGridProto',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..aOM<MapMetaData>(2, _omitFieldNames ? '' : 'info',
        subBuilder: MapMetaData.create)
    ..aOS(3, _omitFieldNames ? '' : 'frameId')
    ..p<$core.int>(4, _omitFieldNames ? '' : 'data', $pb.PbFieldType.K3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  OccupancyGridProto clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  OccupancyGridProto copyWith(void Function(OccupancyGridProto) updates) =>
      super.copyWith((message) => updates(message as OccupancyGridProto))
          as OccupancyGridProto;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static OccupancyGridProto create() => OccupancyGridProto._();
  @$core.override
  OccupancyGridProto createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static OccupancyGridProto getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<OccupancyGridProto>(create);
  static OccupancyGridProto? _defaultInstance;

  @$pb.TagNumber(1)
  $0.Header get header => $_getN(0);
  @$pb.TagNumber(1)
  set header($0.Header value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasHeader() => $_has(0);
  @$pb.TagNumber(1)
  void clearHeader() => $_clearField(1);
  @$pb.TagNumber(1)
  $0.Header ensureHeader() => $_ensure(0);

  @$pb.TagNumber(2)
  MapMetaData get info => $_getN(1);
  @$pb.TagNumber(2)
  set info(MapMetaData value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasInfo() => $_has(1);
  @$pb.TagNumber(2)
  void clearInfo() => $_clearField(2);
  @$pb.TagNumber(2)
  MapMetaData ensureInfo() => $_ensure(1);

  @$pb.TagNumber(3)
  $core.String get frameId => $_getSZ(2);
  @$pb.TagNumber(3)
  set frameId($core.String value) => $_setString(2, value);
  @$pb.TagNumber(3)
  $core.bool hasFrameId() => $_has(2);
  @$pb.TagNumber(3)
  void clearFrameId() => $_clearField(3);

  @$pb.TagNumber(4)
  $pb.PbList<$core.int> get data => $_getList(3);
}

class Path extends $pb.GeneratedMessage {
  factory Path({
    $0.Header? header,
    $core.Iterable<$0.PoseStamped>? poses,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (poses != null) result.poses.addAll(poses);
    return result;
  }

  Path._();

  factory Path.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Path.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Path',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..pPM<$0.PoseStamped>(2, _omitFieldNames ? '' : 'poses',
        subBuilder: $0.PoseStamped.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Path clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Path copyWith(void Function(Path) updates) =>
      super.copyWith((message) => updates(message as Path)) as Path;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Path create() => Path._();
  @$core.override
  Path createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Path getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Path>(create);
  static Path? _defaultInstance;

  @$pb.TagNumber(1)
  $0.Header get header => $_getN(0);
  @$pb.TagNumber(1)
  set header($0.Header value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasHeader() => $_has(0);
  @$pb.TagNumber(1)
  void clearHeader() => $_clearField(1);
  @$pb.TagNumber(1)
  $0.Header ensureHeader() => $_ensure(0);

  @$pb.TagNumber(2)
  $pb.PbList<$0.PoseStamped> get poses => $_getList(1);
}

class Odometry extends $pb.GeneratedMessage {
  factory Odometry({
    $0.Header? header,
    $core.String? childFrameId,
    $0.PoseWithCovariance? pose,
    $0.Twist? twist,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (childFrameId != null) result.childFrameId = childFrameId;
    if (pose != null) result.pose = pose;
    if (twist != null) result.twist = twist;
    return result;
  }

  Odometry._();

  factory Odometry.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Odometry.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Odometry',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..aOS(2, _omitFieldNames ? '' : 'childFrameId')
    ..aOM<$0.PoseWithCovariance>(3, _omitFieldNames ? '' : 'pose',
        subBuilder: $0.PoseWithCovariance.create)
    ..aOM<$0.Twist>(4, _omitFieldNames ? '' : 'twist',
        subBuilder: $0.Twist.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Odometry clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Odometry copyWith(void Function(Odometry) updates) =>
      super.copyWith((message) => updates(message as Odometry)) as Odometry;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Odometry create() => Odometry._();
  @$core.override
  Odometry createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Odometry getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Odometry>(create);
  static Odometry? _defaultInstance;

  @$pb.TagNumber(1)
  $0.Header get header => $_getN(0);
  @$pb.TagNumber(1)
  set header($0.Header value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasHeader() => $_has(0);
  @$pb.TagNumber(1)
  void clearHeader() => $_clearField(1);
  @$pb.TagNumber(1)
  $0.Header ensureHeader() => $_ensure(0);

  @$pb.TagNumber(2)
  $core.String get childFrameId => $_getSZ(1);
  @$pb.TagNumber(2)
  set childFrameId($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasChildFrameId() => $_has(1);
  @$pb.TagNumber(2)
  void clearChildFrameId() => $_clearField(2);

  @$pb.TagNumber(3)
  $0.PoseWithCovariance get pose => $_getN(2);
  @$pb.TagNumber(3)
  set pose($0.PoseWithCovariance value) => $_setField(3, value);
  @$pb.TagNumber(3)
  $core.bool hasPose() => $_has(2);
  @$pb.TagNumber(3)
  void clearPose() => $_clearField(3);
  @$pb.TagNumber(3)
  $0.PoseWithCovariance ensurePose() => $_ensure(2);

  @$pb.TagNumber(4)
  $0.Twist get twist => $_getN(3);
  @$pb.TagNumber(4)
  set twist($0.Twist value) => $_setField(4, value);
  @$pb.TagNumber(4)
  $core.bool hasTwist() => $_has(3);
  @$pb.TagNumber(4)
  void clearTwist() => $_clearField(4);
  @$pb.TagNumber(4)
  $0.Twist ensureTwist() => $_ensure(3);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
