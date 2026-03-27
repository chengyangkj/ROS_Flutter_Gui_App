// This is a generated file - do not edit.
//
// Generated from robot_message.proto.

// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_relative_imports

import 'dart:core' as $core;

import 'package:fixnum/fixnum.dart' as $fixnum;
import 'package:protobuf/protobuf.dart' as $pb;

import 'action_msgs.pb.dart' as $4;
import 'diagnostic_msgs.pb.dart' as $3;
import 'geometry_msgs.pb.dart' as $0;
import 'nav_msgs.pb.dart' as $2;
import 'sensor_msg.pb.dart' as $1;

export 'package:protobuf/protobuf.dart' show GeneratedMessageGenericExtensions;

class ImageFrame extends $pb.GeneratedMessage {
  factory ImageFrame({
    $core.String? encoding,
    $core.int? width,
    $core.int? height,
    $core.int? step,
    $core.List<$core.int>? data,
    $fixnum.Int64? stampSec,
    $core.int? stampNanosec,
    $core.String? frameId,
    $core.String? topic,
  }) {
    final result = create();
    if (encoding != null) result.encoding = encoding;
    if (width != null) result.width = width;
    if (height != null) result.height = height;
    if (step != null) result.step = step;
    if (data != null) result.data = data;
    if (stampSec != null) result.stampSec = stampSec;
    if (stampNanosec != null) result.stampNanosec = stampNanosec;
    if (frameId != null) result.frameId = frameId;
    if (topic != null) result.topic = topic;
    return result;
  }

  ImageFrame._();

  factory ImageFrame.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory ImageFrame.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'ImageFrame',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'encoding')
    ..aI(2, _omitFieldNames ? '' : 'width', fieldType: $pb.PbFieldType.OU3)
    ..aI(3, _omitFieldNames ? '' : 'height', fieldType: $pb.PbFieldType.OU3)
    ..aI(4, _omitFieldNames ? '' : 'step', fieldType: $pb.PbFieldType.OU3)
    ..a<$core.List<$core.int>>(
        5, _omitFieldNames ? '' : 'data', $pb.PbFieldType.OY)
    ..aInt64(6, _omitFieldNames ? '' : 'stampSec')
    ..aI(7, _omitFieldNames ? '' : 'stampNanosec',
        fieldType: $pb.PbFieldType.OU3)
    ..aOS(8, _omitFieldNames ? '' : 'frameId')
    ..aOS(9, _omitFieldNames ? '' : 'topic')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ImageFrame clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ImageFrame copyWith(void Function(ImageFrame) updates) =>
      super.copyWith((message) => updates(message as ImageFrame)) as ImageFrame;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static ImageFrame create() => ImageFrame._();
  @$core.override
  ImageFrame createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static ImageFrame getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<ImageFrame>(create);
  static ImageFrame? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get encoding => $_getSZ(0);
  @$pb.TagNumber(1)
  set encoding($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasEncoding() => $_has(0);
  @$pb.TagNumber(1)
  void clearEncoding() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get width => $_getIZ(1);
  @$pb.TagNumber(2)
  set width($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasWidth() => $_has(1);
  @$pb.TagNumber(2)
  void clearWidth() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get height => $_getIZ(2);
  @$pb.TagNumber(3)
  set height($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasHeight() => $_has(2);
  @$pb.TagNumber(3)
  void clearHeight() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.int get step => $_getIZ(3);
  @$pb.TagNumber(4)
  set step($core.int value) => $_setUnsignedInt32(3, value);
  @$pb.TagNumber(4)
  $core.bool hasStep() => $_has(3);
  @$pb.TagNumber(4)
  void clearStep() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.List<$core.int> get data => $_getN(4);
  @$pb.TagNumber(5)
  set data($core.List<$core.int> value) => $_setBytes(4, value);
  @$pb.TagNumber(5)
  $core.bool hasData() => $_has(4);
  @$pb.TagNumber(5)
  void clearData() => $_clearField(5);

  @$pb.TagNumber(6)
  $fixnum.Int64 get stampSec => $_getI64(5);
  @$pb.TagNumber(6)
  set stampSec($fixnum.Int64 value) => $_setInt64(5, value);
  @$pb.TagNumber(6)
  $core.bool hasStampSec() => $_has(5);
  @$pb.TagNumber(6)
  void clearStampSec() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.int get stampNanosec => $_getIZ(6);
  @$pb.TagNumber(7)
  set stampNanosec($core.int value) => $_setUnsignedInt32(6, value);
  @$pb.TagNumber(7)
  $core.bool hasStampNanosec() => $_has(6);
  @$pb.TagNumber(7)
  void clearStampNanosec() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.String get frameId => $_getSZ(7);
  @$pb.TagNumber(8)
  set frameId($core.String value) => $_setString(7, value);
  @$pb.TagNumber(8)
  $core.bool hasFrameId() => $_has(7);
  @$pb.TagNumber(8)
  void clearFrameId() => $_clearField(8);

  @$pb.TagNumber(9)
  $core.String get topic => $_getSZ(8);
  @$pb.TagNumber(9)
  set topic($core.String value) => $_setString(8, value);
  @$pb.TagNumber(9)
  $core.bool hasTopic() => $_has(8);
  @$pb.TagNumber(9)
  void clearTopic() => $_clearField(9);
}

class Heartbeat extends $pb.GeneratedMessage {
  factory Heartbeat({
    $fixnum.Int64? serverTimeMs,
    $core.int? seq,
  }) {
    final result = create();
    if (serverTimeMs != null) result.serverTimeMs = serverTimeMs;
    if (seq != null) result.seq = seq;
    return result;
  }

  Heartbeat._();

  factory Heartbeat.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Heartbeat.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Heartbeat',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aInt64(1, _omitFieldNames ? '' : 'serverTimeMs')
    ..aI(2, _omitFieldNames ? '' : 'seq', fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Heartbeat clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Heartbeat copyWith(void Function(Heartbeat) updates) =>
      super.copyWith((message) => updates(message as Heartbeat)) as Heartbeat;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Heartbeat create() => Heartbeat._();
  @$core.override
  Heartbeat createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Heartbeat getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Heartbeat>(create);
  static Heartbeat? _defaultInstance;

  @$pb.TagNumber(1)
  $fixnum.Int64 get serverTimeMs => $_getI64(0);
  @$pb.TagNumber(1)
  set serverTimeMs($fixnum.Int64 value) => $_setInt64(0, value);
  @$pb.TagNumber(1)
  $core.bool hasServerTimeMs() => $_has(0);
  @$pb.TagNumber(1)
  void clearServerTimeMs() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get seq => $_getIZ(1);
  @$pb.TagNumber(2)
  set seq($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasSeq() => $_has(1);
  @$pb.TagNumber(2)
  void clearSeq() => $_clearField(2);
}

class ControlSubscribeImage extends $pb.GeneratedMessage {
  factory ControlSubscribeImage({
    $core.String? topic,
    $core.bool? subscribe,
  }) {
    final result = create();
    if (topic != null) result.topic = topic;
    if (subscribe != null) result.subscribe = subscribe;
    return result;
  }

  ControlSubscribeImage._();

  factory ControlSubscribeImage.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory ControlSubscribeImage.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'ControlSubscribeImage',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'topic')
    ..aOB(2, _omitFieldNames ? '' : 'subscribe')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ControlSubscribeImage clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ControlSubscribeImage copyWith(
          void Function(ControlSubscribeImage) updates) =>
      super.copyWith((message) => updates(message as ControlSubscribeImage))
          as ControlSubscribeImage;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static ControlSubscribeImage create() => ControlSubscribeImage._();
  @$core.override
  ControlSubscribeImage createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static ControlSubscribeImage getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<ControlSubscribeImage>(create);
  static ControlSubscribeImage? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get topic => $_getSZ(0);
  @$pb.TagNumber(1)
  set topic($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasTopic() => $_has(0);
  @$pb.TagNumber(1)
  void clearTopic() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.bool get subscribe => $_getBF(1);
  @$pb.TagNumber(2)
  set subscribe($core.bool value) => $_setBool(1, value);
  @$pb.TagNumber(2)
  $core.bool hasSubscribe() => $_has(1);
  @$pb.TagNumber(2)
  void clearSubscribe() => $_clearField(2);
}

class CancelNav extends $pb.GeneratedMessage {
  factory CancelNav() => create();

  CancelNav._();

  factory CancelNav.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory CancelNav.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'CancelNav',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  CancelNav clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  CancelNav copyWith(void Function(CancelNav) updates) =>
      super.copyWith((message) => updates(message as CancelNav)) as CancelNav;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static CancelNav create() => CancelNav._();
  @$core.override
  CancelNav createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static CancelNav getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<CancelNav>(create);
  static CancelNav? _defaultInstance;
}

class TransformLookupRequest extends $pb.GeneratedMessage {
  factory TransformLookupRequest({
    $core.String? targetFrame,
    $core.String? sourceFrame,
    $core.int? seq,
  }) {
    final result = create();
    if (targetFrame != null) result.targetFrame = targetFrame;
    if (sourceFrame != null) result.sourceFrame = sourceFrame;
    if (seq != null) result.seq = seq;
    return result;
  }

  TransformLookupRequest._();

  factory TransformLookupRequest.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TransformLookupRequest.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TransformLookupRequest',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'targetFrame')
    ..aOS(2, _omitFieldNames ? '' : 'sourceFrame')
    ..aI(3, _omitFieldNames ? '' : 'seq', fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TransformLookupRequest clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TransformLookupRequest copyWith(
          void Function(TransformLookupRequest) updates) =>
      super.copyWith((message) => updates(message as TransformLookupRequest))
          as TransformLookupRequest;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TransformLookupRequest create() => TransformLookupRequest._();
  @$core.override
  TransformLookupRequest createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TransformLookupRequest getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TransformLookupRequest>(create);
  static TransformLookupRequest? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get targetFrame => $_getSZ(0);
  @$pb.TagNumber(1)
  set targetFrame($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasTargetFrame() => $_has(0);
  @$pb.TagNumber(1)
  void clearTargetFrame() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get sourceFrame => $_getSZ(1);
  @$pb.TagNumber(2)
  set sourceFrame($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasSourceFrame() => $_has(1);
  @$pb.TagNumber(2)
  void clearSourceFrame() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get seq => $_getIZ(2);
  @$pb.TagNumber(3)
  set seq($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasSeq() => $_has(2);
  @$pb.TagNumber(3)
  void clearSeq() => $_clearField(3);
}

class TransformLookupResponse extends $pb.GeneratedMessage {
  factory TransformLookupResponse({
    $core.int? seq,
    $core.bool? ok,
    $core.String? error,
    $0.TransformStamped? transform,
  }) {
    final result = create();
    if (seq != null) result.seq = seq;
    if (ok != null) result.ok = ok;
    if (error != null) result.error = error;
    if (transform != null) result.transform = transform;
    return result;
  }

  TransformLookupResponse._();

  factory TransformLookupResponse.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TransformLookupResponse.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TransformLookupResponse',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aI(1, _omitFieldNames ? '' : 'seq', fieldType: $pb.PbFieldType.OU3)
    ..aOB(2, _omitFieldNames ? '' : 'ok')
    ..aOS(3, _omitFieldNames ? '' : 'error')
    ..aOM<$0.TransformStamped>(4, _omitFieldNames ? '' : 'transform',
        subBuilder: $0.TransformStamped.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TransformLookupResponse clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TransformLookupResponse copyWith(
          void Function(TransformLookupResponse) updates) =>
      super.copyWith((message) => updates(message as TransformLookupResponse))
          as TransformLookupResponse;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TransformLookupResponse create() => TransformLookupResponse._();
  @$core.override
  TransformLookupResponse createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TransformLookupResponse getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TransformLookupResponse>(create);
  static TransformLookupResponse? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get seq => $_getIZ(0);
  @$pb.TagNumber(1)
  set seq($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasSeq() => $_has(0);
  @$pb.TagNumber(1)
  void clearSeq() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.bool get ok => $_getBF(1);
  @$pb.TagNumber(2)
  set ok($core.bool value) => $_setBool(1, value);
  @$pb.TagNumber(2)
  $core.bool hasOk() => $_has(1);
  @$pb.TagNumber(2)
  void clearOk() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.String get error => $_getSZ(2);
  @$pb.TagNumber(3)
  set error($core.String value) => $_setString(2, value);
  @$pb.TagNumber(3)
  $core.bool hasError() => $_has(2);
  @$pb.TagNumber(3)
  void clearError() => $_clearField(3);

  @$pb.TagNumber(4)
  $0.TransformStamped get transform => $_getN(3);
  @$pb.TagNumber(4)
  set transform($0.TransformStamped value) => $_setField(4, value);
  @$pb.TagNumber(4)
  $core.bool hasTransform() => $_has(3);
  @$pb.TagNumber(4)
  void clearTransform() => $_clearField(4);
  @$pb.TagNumber(4)
  $0.TransformStamped ensureTransform() => $_ensure(3);
}

enum RobotMessage_Payload {
  image,
  heartbeat,
  laserScan,
  robotPoseMap,
  pathLocal,
  pathGlobal,
  pathTrace,
  odometry,
  battery,
  footprint,
  localCostmap,
  globalCostmap,
  pointcloudMap,
  diagnostic,
  navStatus,
  transformLookupResponse,
  notSet
}

class RobotMessage extends $pb.GeneratedMessage {
  factory RobotMessage({
    ImageFrame? image,
    Heartbeat? heartbeat,
    $1.LaserScanBaseLink? laserScan,
    $0.PoseStamped? robotPoseMap,
    $2.Path? pathLocal,
    $2.Path? pathGlobal,
    $2.Path? pathTrace,
    $2.Odometry? odometry,
    $1.BatteryState? battery,
    $0.PolygonStamped? footprint,
    $2.OccupancyGridProto? localCostmap,
    $2.OccupancyGridProto? globalCostmap,
    $1.PointCloud2MapFrame? pointcloudMap,
    $3.DiagnosticArray? diagnostic,
    $4.GoalStatusArray? navStatus,
    TransformLookupResponse? transformLookupResponse,
  }) {
    final result = create();
    if (image != null) result.image = image;
    if (heartbeat != null) result.heartbeat = heartbeat;
    if (laserScan != null) result.laserScan = laserScan;
    if (robotPoseMap != null) result.robotPoseMap = robotPoseMap;
    if (pathLocal != null) result.pathLocal = pathLocal;
    if (pathGlobal != null) result.pathGlobal = pathGlobal;
    if (pathTrace != null) result.pathTrace = pathTrace;
    if (odometry != null) result.odometry = odometry;
    if (battery != null) result.battery = battery;
    if (footprint != null) result.footprint = footprint;
    if (localCostmap != null) result.localCostmap = localCostmap;
    if (globalCostmap != null) result.globalCostmap = globalCostmap;
    if (pointcloudMap != null) result.pointcloudMap = pointcloudMap;
    if (diagnostic != null) result.diagnostic = diagnostic;
    if (navStatus != null) result.navStatus = navStatus;
    if (transformLookupResponse != null)
      result.transformLookupResponse = transformLookupResponse;
    return result;
  }

  RobotMessage._();

  factory RobotMessage.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory RobotMessage.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static const $core.Map<$core.int, RobotMessage_Payload>
      _RobotMessage_PayloadByTag = {
    1: RobotMessage_Payload.image,
    3: RobotMessage_Payload.heartbeat,
    4: RobotMessage_Payload.laserScan,
    5: RobotMessage_Payload.robotPoseMap,
    6: RobotMessage_Payload.pathLocal,
    7: RobotMessage_Payload.pathGlobal,
    8: RobotMessage_Payload.pathTrace,
    9: RobotMessage_Payload.odometry,
    10: RobotMessage_Payload.battery,
    11: RobotMessage_Payload.footprint,
    12: RobotMessage_Payload.localCostmap,
    13: RobotMessage_Payload.globalCostmap,
    14: RobotMessage_Payload.pointcloudMap,
    15: RobotMessage_Payload.diagnostic,
    17: RobotMessage_Payload.navStatus,
    18: RobotMessage_Payload.transformLookupResponse,
    0: RobotMessage_Payload.notSet
  };
  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'RobotMessage',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..oo(0, [1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18])
    ..aOM<ImageFrame>(1, _omitFieldNames ? '' : 'image',
        subBuilder: ImageFrame.create)
    ..aOM<Heartbeat>(3, _omitFieldNames ? '' : 'heartbeat',
        subBuilder: Heartbeat.create)
    ..aOM<$1.LaserScanBaseLink>(4, _omitFieldNames ? '' : 'laserScan',
        subBuilder: $1.LaserScanBaseLink.create)
    ..aOM<$0.PoseStamped>(5, _omitFieldNames ? '' : 'robotPoseMap',
        subBuilder: $0.PoseStamped.create)
    ..aOM<$2.Path>(6, _omitFieldNames ? '' : 'pathLocal',
        subBuilder: $2.Path.create)
    ..aOM<$2.Path>(7, _omitFieldNames ? '' : 'pathGlobal',
        subBuilder: $2.Path.create)
    ..aOM<$2.Path>(8, _omitFieldNames ? '' : 'pathTrace',
        subBuilder: $2.Path.create)
    ..aOM<$2.Odometry>(9, _omitFieldNames ? '' : 'odometry',
        subBuilder: $2.Odometry.create)
    ..aOM<$1.BatteryState>(10, _omitFieldNames ? '' : 'battery',
        subBuilder: $1.BatteryState.create)
    ..aOM<$0.PolygonStamped>(11, _omitFieldNames ? '' : 'footprint',
        subBuilder: $0.PolygonStamped.create)
    ..aOM<$2.OccupancyGridProto>(12, _omitFieldNames ? '' : 'localCostmap',
        subBuilder: $2.OccupancyGridProto.create)
    ..aOM<$2.OccupancyGridProto>(13, _omitFieldNames ? '' : 'globalCostmap',
        subBuilder: $2.OccupancyGridProto.create)
    ..aOM<$1.PointCloud2MapFrame>(14, _omitFieldNames ? '' : 'pointcloudMap',
        subBuilder: $1.PointCloud2MapFrame.create)
    ..aOM<$3.DiagnosticArray>(15, _omitFieldNames ? '' : 'diagnostic',
        subBuilder: $3.DiagnosticArray.create)
    ..aOM<$4.GoalStatusArray>(17, _omitFieldNames ? '' : 'navStatus',
        subBuilder: $4.GoalStatusArray.create)
    ..aOM<TransformLookupResponse>(
        18, _omitFieldNames ? '' : 'transformLookupResponse',
        subBuilder: TransformLookupResponse.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RobotMessage clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RobotMessage copyWith(void Function(RobotMessage) updates) =>
      super.copyWith((message) => updates(message as RobotMessage))
          as RobotMessage;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static RobotMessage create() => RobotMessage._();
  @$core.override
  RobotMessage createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static RobotMessage getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<RobotMessage>(create);
  static RobotMessage? _defaultInstance;

  @$pb.TagNumber(1)
  @$pb.TagNumber(3)
  @$pb.TagNumber(4)
  @$pb.TagNumber(5)
  @$pb.TagNumber(6)
  @$pb.TagNumber(7)
  @$pb.TagNumber(8)
  @$pb.TagNumber(9)
  @$pb.TagNumber(10)
  @$pb.TagNumber(11)
  @$pb.TagNumber(12)
  @$pb.TagNumber(13)
  @$pb.TagNumber(14)
  @$pb.TagNumber(15)
  @$pb.TagNumber(17)
  @$pb.TagNumber(18)
  RobotMessage_Payload whichPayload() =>
      _RobotMessage_PayloadByTag[$_whichOneof(0)]!;
  @$pb.TagNumber(1)
  @$pb.TagNumber(3)
  @$pb.TagNumber(4)
  @$pb.TagNumber(5)
  @$pb.TagNumber(6)
  @$pb.TagNumber(7)
  @$pb.TagNumber(8)
  @$pb.TagNumber(9)
  @$pb.TagNumber(10)
  @$pb.TagNumber(11)
  @$pb.TagNumber(12)
  @$pb.TagNumber(13)
  @$pb.TagNumber(14)
  @$pb.TagNumber(15)
  @$pb.TagNumber(17)
  @$pb.TagNumber(18)
  void clearPayload() => $_clearField($_whichOneof(0));

  @$pb.TagNumber(1)
  ImageFrame get image => $_getN(0);
  @$pb.TagNumber(1)
  set image(ImageFrame value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasImage() => $_has(0);
  @$pb.TagNumber(1)
  void clearImage() => $_clearField(1);
  @$pb.TagNumber(1)
  ImageFrame ensureImage() => $_ensure(0);

  @$pb.TagNumber(3)
  Heartbeat get heartbeat => $_getN(1);
  @$pb.TagNumber(3)
  set heartbeat(Heartbeat value) => $_setField(3, value);
  @$pb.TagNumber(3)
  $core.bool hasHeartbeat() => $_has(1);
  @$pb.TagNumber(3)
  void clearHeartbeat() => $_clearField(3);
  @$pb.TagNumber(3)
  Heartbeat ensureHeartbeat() => $_ensure(1);

  @$pb.TagNumber(4)
  $1.LaserScanBaseLink get laserScan => $_getN(2);
  @$pb.TagNumber(4)
  set laserScan($1.LaserScanBaseLink value) => $_setField(4, value);
  @$pb.TagNumber(4)
  $core.bool hasLaserScan() => $_has(2);
  @$pb.TagNumber(4)
  void clearLaserScan() => $_clearField(4);
  @$pb.TagNumber(4)
  $1.LaserScanBaseLink ensureLaserScan() => $_ensure(2);

  @$pb.TagNumber(5)
  $0.PoseStamped get robotPoseMap => $_getN(3);
  @$pb.TagNumber(5)
  set robotPoseMap($0.PoseStamped value) => $_setField(5, value);
  @$pb.TagNumber(5)
  $core.bool hasRobotPoseMap() => $_has(3);
  @$pb.TagNumber(5)
  void clearRobotPoseMap() => $_clearField(5);
  @$pb.TagNumber(5)
  $0.PoseStamped ensureRobotPoseMap() => $_ensure(3);

  @$pb.TagNumber(6)
  $2.Path get pathLocal => $_getN(4);
  @$pb.TagNumber(6)
  set pathLocal($2.Path value) => $_setField(6, value);
  @$pb.TagNumber(6)
  $core.bool hasPathLocal() => $_has(4);
  @$pb.TagNumber(6)
  void clearPathLocal() => $_clearField(6);
  @$pb.TagNumber(6)
  $2.Path ensurePathLocal() => $_ensure(4);

  @$pb.TagNumber(7)
  $2.Path get pathGlobal => $_getN(5);
  @$pb.TagNumber(7)
  set pathGlobal($2.Path value) => $_setField(7, value);
  @$pb.TagNumber(7)
  $core.bool hasPathGlobal() => $_has(5);
  @$pb.TagNumber(7)
  void clearPathGlobal() => $_clearField(7);
  @$pb.TagNumber(7)
  $2.Path ensurePathGlobal() => $_ensure(5);

  @$pb.TagNumber(8)
  $2.Path get pathTrace => $_getN(6);
  @$pb.TagNumber(8)
  set pathTrace($2.Path value) => $_setField(8, value);
  @$pb.TagNumber(8)
  $core.bool hasPathTrace() => $_has(6);
  @$pb.TagNumber(8)
  void clearPathTrace() => $_clearField(8);
  @$pb.TagNumber(8)
  $2.Path ensurePathTrace() => $_ensure(6);

  @$pb.TagNumber(9)
  $2.Odometry get odometry => $_getN(7);
  @$pb.TagNumber(9)
  set odometry($2.Odometry value) => $_setField(9, value);
  @$pb.TagNumber(9)
  $core.bool hasOdometry() => $_has(7);
  @$pb.TagNumber(9)
  void clearOdometry() => $_clearField(9);
  @$pb.TagNumber(9)
  $2.Odometry ensureOdometry() => $_ensure(7);

  @$pb.TagNumber(10)
  $1.BatteryState get battery => $_getN(8);
  @$pb.TagNumber(10)
  set battery($1.BatteryState value) => $_setField(10, value);
  @$pb.TagNumber(10)
  $core.bool hasBattery() => $_has(8);
  @$pb.TagNumber(10)
  void clearBattery() => $_clearField(10);
  @$pb.TagNumber(10)
  $1.BatteryState ensureBattery() => $_ensure(8);

  @$pb.TagNumber(11)
  $0.PolygonStamped get footprint => $_getN(9);
  @$pb.TagNumber(11)
  set footprint($0.PolygonStamped value) => $_setField(11, value);
  @$pb.TagNumber(11)
  $core.bool hasFootprint() => $_has(9);
  @$pb.TagNumber(11)
  void clearFootprint() => $_clearField(11);
  @$pb.TagNumber(11)
  $0.PolygonStamped ensureFootprint() => $_ensure(9);

  @$pb.TagNumber(12)
  $2.OccupancyGridProto get localCostmap => $_getN(10);
  @$pb.TagNumber(12)
  set localCostmap($2.OccupancyGridProto value) => $_setField(12, value);
  @$pb.TagNumber(12)
  $core.bool hasLocalCostmap() => $_has(10);
  @$pb.TagNumber(12)
  void clearLocalCostmap() => $_clearField(12);
  @$pb.TagNumber(12)
  $2.OccupancyGridProto ensureLocalCostmap() => $_ensure(10);

  @$pb.TagNumber(13)
  $2.OccupancyGridProto get globalCostmap => $_getN(11);
  @$pb.TagNumber(13)
  set globalCostmap($2.OccupancyGridProto value) => $_setField(13, value);
  @$pb.TagNumber(13)
  $core.bool hasGlobalCostmap() => $_has(11);
  @$pb.TagNumber(13)
  void clearGlobalCostmap() => $_clearField(13);
  @$pb.TagNumber(13)
  $2.OccupancyGridProto ensureGlobalCostmap() => $_ensure(11);

  @$pb.TagNumber(14)
  $1.PointCloud2MapFrame get pointcloudMap => $_getN(12);
  @$pb.TagNumber(14)
  set pointcloudMap($1.PointCloud2MapFrame value) => $_setField(14, value);
  @$pb.TagNumber(14)
  $core.bool hasPointcloudMap() => $_has(12);
  @$pb.TagNumber(14)
  void clearPointcloudMap() => $_clearField(14);
  @$pb.TagNumber(14)
  $1.PointCloud2MapFrame ensurePointcloudMap() => $_ensure(12);

  @$pb.TagNumber(15)
  $3.DiagnosticArray get diagnostic => $_getN(13);
  @$pb.TagNumber(15)
  set diagnostic($3.DiagnosticArray value) => $_setField(15, value);
  @$pb.TagNumber(15)
  $core.bool hasDiagnostic() => $_has(13);
  @$pb.TagNumber(15)
  void clearDiagnostic() => $_clearField(15);
  @$pb.TagNumber(15)
  $3.DiagnosticArray ensureDiagnostic() => $_ensure(13);

  @$pb.TagNumber(17)
  $4.GoalStatusArray get navStatus => $_getN(14);
  @$pb.TagNumber(17)
  set navStatus($4.GoalStatusArray value) => $_setField(17, value);
  @$pb.TagNumber(17)
  $core.bool hasNavStatus() => $_has(14);
  @$pb.TagNumber(17)
  void clearNavStatus() => $_clearField(17);
  @$pb.TagNumber(17)
  $4.GoalStatusArray ensureNavStatus() => $_ensure(14);

  @$pb.TagNumber(18)
  TransformLookupResponse get transformLookupResponse => $_getN(15);
  @$pb.TagNumber(18)
  set transformLookupResponse(TransformLookupResponse value) =>
      $_setField(18, value);
  @$pb.TagNumber(18)
  $core.bool hasTransformLookupResponse() => $_has(15);
  @$pb.TagNumber(18)
  void clearTransformLookupResponse() => $_clearField(18);
  @$pb.TagNumber(18)
  TransformLookupResponse ensureTransformLookupResponse() => $_ensure(15);
}

enum ClientRobotMessage_Payload {
  controlSubscribeImage,
  cmdVel,
  navGoal,
  cancelNav,
  reloc,
  transformLookupRequest,
  notSet
}

class ClientRobotMessage extends $pb.GeneratedMessage {
  factory ClientRobotMessage({
    ControlSubscribeImage? controlSubscribeImage,
    $0.Twist? cmdVel,
    $0.PoseStamped? navGoal,
    CancelNav? cancelNav,
    $0.PoseWithCovarianceStamped? reloc,
    TransformLookupRequest? transformLookupRequest,
  }) {
    final result = create();
    if (controlSubscribeImage != null)
      result.controlSubscribeImage = controlSubscribeImage;
    if (cmdVel != null) result.cmdVel = cmdVel;
    if (navGoal != null) result.navGoal = navGoal;
    if (cancelNav != null) result.cancelNav = cancelNav;
    if (reloc != null) result.reloc = reloc;
    if (transformLookupRequest != null)
      result.transformLookupRequest = transformLookupRequest;
    return result;
  }

  ClientRobotMessage._();

  factory ClientRobotMessage.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory ClientRobotMessage.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static const $core.Map<$core.int, ClientRobotMessage_Payload>
      _ClientRobotMessage_PayloadByTag = {
    1: ClientRobotMessage_Payload.controlSubscribeImage,
    2: ClientRobotMessage_Payload.cmdVel,
    3: ClientRobotMessage_Payload.navGoal,
    4: ClientRobotMessage_Payload.cancelNav,
    5: ClientRobotMessage_Payload.reloc,
    7: ClientRobotMessage_Payload.transformLookupRequest,
    0: ClientRobotMessage_Payload.notSet
  };
  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'ClientRobotMessage',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..oo(0, [1, 2, 3, 4, 5, 7])
    ..aOM<ControlSubscribeImage>(
        1, _omitFieldNames ? '' : 'controlSubscribeImage',
        subBuilder: ControlSubscribeImage.create)
    ..aOM<$0.Twist>(2, _omitFieldNames ? '' : 'cmdVel',
        subBuilder: $0.Twist.create)
    ..aOM<$0.PoseStamped>(3, _omitFieldNames ? '' : 'navGoal',
        subBuilder: $0.PoseStamped.create)
    ..aOM<CancelNav>(4, _omitFieldNames ? '' : 'cancelNav',
        subBuilder: CancelNav.create)
    ..aOM<$0.PoseWithCovarianceStamped>(5, _omitFieldNames ? '' : 'reloc',
        subBuilder: $0.PoseWithCovarianceStamped.create)
    ..aOM<TransformLookupRequest>(
        7, _omitFieldNames ? '' : 'transformLookupRequest',
        subBuilder: TransformLookupRequest.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ClientRobotMessage clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ClientRobotMessage copyWith(void Function(ClientRobotMessage) updates) =>
      super.copyWith((message) => updates(message as ClientRobotMessage))
          as ClientRobotMessage;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static ClientRobotMessage create() => ClientRobotMessage._();
  @$core.override
  ClientRobotMessage createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static ClientRobotMessage getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<ClientRobotMessage>(create);
  static ClientRobotMessage? _defaultInstance;

  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  @$pb.TagNumber(3)
  @$pb.TagNumber(4)
  @$pb.TagNumber(5)
  @$pb.TagNumber(7)
  ClientRobotMessage_Payload whichPayload() =>
      _ClientRobotMessage_PayloadByTag[$_whichOneof(0)]!;
  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  @$pb.TagNumber(3)
  @$pb.TagNumber(4)
  @$pb.TagNumber(5)
  @$pb.TagNumber(7)
  void clearPayload() => $_clearField($_whichOneof(0));

  @$pb.TagNumber(1)
  ControlSubscribeImage get controlSubscribeImage => $_getN(0);
  @$pb.TagNumber(1)
  set controlSubscribeImage(ControlSubscribeImage value) =>
      $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasControlSubscribeImage() => $_has(0);
  @$pb.TagNumber(1)
  void clearControlSubscribeImage() => $_clearField(1);
  @$pb.TagNumber(1)
  ControlSubscribeImage ensureControlSubscribeImage() => $_ensure(0);

  @$pb.TagNumber(2)
  $0.Twist get cmdVel => $_getN(1);
  @$pb.TagNumber(2)
  set cmdVel($0.Twist value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasCmdVel() => $_has(1);
  @$pb.TagNumber(2)
  void clearCmdVel() => $_clearField(2);
  @$pb.TagNumber(2)
  $0.Twist ensureCmdVel() => $_ensure(1);

  @$pb.TagNumber(3)
  $0.PoseStamped get navGoal => $_getN(2);
  @$pb.TagNumber(3)
  set navGoal($0.PoseStamped value) => $_setField(3, value);
  @$pb.TagNumber(3)
  $core.bool hasNavGoal() => $_has(2);
  @$pb.TagNumber(3)
  void clearNavGoal() => $_clearField(3);
  @$pb.TagNumber(3)
  $0.PoseStamped ensureNavGoal() => $_ensure(2);

  @$pb.TagNumber(4)
  CancelNav get cancelNav => $_getN(3);
  @$pb.TagNumber(4)
  set cancelNav(CancelNav value) => $_setField(4, value);
  @$pb.TagNumber(4)
  $core.bool hasCancelNav() => $_has(3);
  @$pb.TagNumber(4)
  void clearCancelNav() => $_clearField(4);
  @$pb.TagNumber(4)
  CancelNav ensureCancelNav() => $_ensure(3);

  @$pb.TagNumber(5)
  $0.PoseWithCovarianceStamped get reloc => $_getN(4);
  @$pb.TagNumber(5)
  set reloc($0.PoseWithCovarianceStamped value) => $_setField(5, value);
  @$pb.TagNumber(5)
  $core.bool hasReloc() => $_has(4);
  @$pb.TagNumber(5)
  void clearReloc() => $_clearField(5);
  @$pb.TagNumber(5)
  $0.PoseWithCovarianceStamped ensureReloc() => $_ensure(4);

  @$pb.TagNumber(7)
  TransformLookupRequest get transformLookupRequest => $_getN(5);
  @$pb.TagNumber(7)
  set transformLookupRequest(TransformLookupRequest value) =>
      $_setField(7, value);
  @$pb.TagNumber(7)
  $core.bool hasTransformLookupRequest() => $_has(5);
  @$pb.TagNumber(7)
  void clearTransformLookupRequest() => $_clearField(7);
  @$pb.TagNumber(7)
  TransformLookupRequest ensureTransformLookupRequest() => $_ensure(5);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
