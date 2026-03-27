// This is a generated file - do not edit.
//
// Generated from geometry_msgs.proto.

// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_relative_imports

import 'dart:core' as $core;

import 'package:protobuf/protobuf.dart' as $pb;

export 'package:protobuf/protobuf.dart' show GeneratedMessageGenericExtensions;

class Time extends $pb.GeneratedMessage {
  factory Time({
    $core.int? sec,
    $core.int? nanosec,
  }) {
    final result = create();
    if (sec != null) result.sec = sec;
    if (nanosec != null) result.nanosec = nanosec;
    return result;
  }

  Time._();

  factory Time.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Time.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Time',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aI(1, _omitFieldNames ? '' : 'sec')
    ..aI(2, _omitFieldNames ? '' : 'nanosec', fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Time clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Time copyWith(void Function(Time) updates) =>
      super.copyWith((message) => updates(message as Time)) as Time;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Time create() => Time._();
  @$core.override
  Time createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Time getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Time>(create);
  static Time? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get sec => $_getIZ(0);
  @$pb.TagNumber(1)
  set sec($core.int value) => $_setSignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasSec() => $_has(0);
  @$pb.TagNumber(1)
  void clearSec() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get nanosec => $_getIZ(1);
  @$pb.TagNumber(2)
  set nanosec($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasNanosec() => $_has(1);
  @$pb.TagNumber(2)
  void clearNanosec() => $_clearField(2);
}

class Header extends $pb.GeneratedMessage {
  factory Header({
    Time? stamp,
    $core.String? frameId,
  }) {
    final result = create();
    if (stamp != null) result.stamp = stamp;
    if (frameId != null) result.frameId = frameId;
    return result;
  }

  Header._();

  factory Header.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Header.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Header',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Time>(1, _omitFieldNames ? '' : 'stamp', subBuilder: Time.create)
    ..aOS(2, _omitFieldNames ? '' : 'frameId')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Header clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Header copyWith(void Function(Header) updates) =>
      super.copyWith((message) => updates(message as Header)) as Header;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Header create() => Header._();
  @$core.override
  Header createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Header getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Header>(create);
  static Header? _defaultInstance;

  @$pb.TagNumber(1)
  Time get stamp => $_getN(0);
  @$pb.TagNumber(1)
  set stamp(Time value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasStamp() => $_has(0);
  @$pb.TagNumber(1)
  void clearStamp() => $_clearField(1);
  @$pb.TagNumber(1)
  Time ensureStamp() => $_ensure(0);

  @$pb.TagNumber(2)
  $core.String get frameId => $_getSZ(1);
  @$pb.TagNumber(2)
  set frameId($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasFrameId() => $_has(1);
  @$pb.TagNumber(2)
  void clearFrameId() => $_clearField(2);
}

class Point extends $pb.GeneratedMessage {
  factory Point({
    $core.double? x,
    $core.double? y,
    $core.double? z,
  }) {
    final result = create();
    if (x != null) result.x = x;
    if (y != null) result.y = y;
    if (z != null) result.z = z;
    return result;
  }

  Point._();

  factory Point.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Point.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Point',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aD(1, _omitFieldNames ? '' : 'x')
    ..aD(2, _omitFieldNames ? '' : 'y')
    ..aD(3, _omitFieldNames ? '' : 'z')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Point clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Point copyWith(void Function(Point) updates) =>
      super.copyWith((message) => updates(message as Point)) as Point;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Point create() => Point._();
  @$core.override
  Point createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Point getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Point>(create);
  static Point? _defaultInstance;

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
  $core.double get z => $_getN(2);
  @$pb.TagNumber(3)
  set z($core.double value) => $_setDouble(2, value);
  @$pb.TagNumber(3)
  $core.bool hasZ() => $_has(2);
  @$pb.TagNumber(3)
  void clearZ() => $_clearField(3);
}

class Quaternion extends $pb.GeneratedMessage {
  factory Quaternion({
    $core.double? x,
    $core.double? y,
    $core.double? z,
    $core.double? w,
  }) {
    final result = create();
    if (x != null) result.x = x;
    if (y != null) result.y = y;
    if (z != null) result.z = z;
    if (w != null) result.w = w;
    return result;
  }

  Quaternion._();

  factory Quaternion.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Quaternion.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Quaternion',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aD(1, _omitFieldNames ? '' : 'x')
    ..aD(2, _omitFieldNames ? '' : 'y')
    ..aD(3, _omitFieldNames ? '' : 'z')
    ..aD(4, _omitFieldNames ? '' : 'w')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Quaternion clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Quaternion copyWith(void Function(Quaternion) updates) =>
      super.copyWith((message) => updates(message as Quaternion)) as Quaternion;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Quaternion create() => Quaternion._();
  @$core.override
  Quaternion createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Quaternion getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<Quaternion>(create);
  static Quaternion? _defaultInstance;

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
  $core.double get z => $_getN(2);
  @$pb.TagNumber(3)
  set z($core.double value) => $_setDouble(2, value);
  @$pb.TagNumber(3)
  $core.bool hasZ() => $_has(2);
  @$pb.TagNumber(3)
  void clearZ() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.double get w => $_getN(3);
  @$pb.TagNumber(4)
  set w($core.double value) => $_setDouble(3, value);
  @$pb.TagNumber(4)
  $core.bool hasW() => $_has(3);
  @$pb.TagNumber(4)
  void clearW() => $_clearField(4);
}

class Vector3 extends $pb.GeneratedMessage {
  factory Vector3({
    $core.double? x,
    $core.double? y,
    $core.double? z,
  }) {
    final result = create();
    if (x != null) result.x = x;
    if (y != null) result.y = y;
    if (z != null) result.z = z;
    return result;
  }

  Vector3._();

  factory Vector3.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Vector3.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Vector3',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aD(1, _omitFieldNames ? '' : 'x')
    ..aD(2, _omitFieldNames ? '' : 'y')
    ..aD(3, _omitFieldNames ? '' : 'z')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Vector3 clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Vector3 copyWith(void Function(Vector3) updates) =>
      super.copyWith((message) => updates(message as Vector3)) as Vector3;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Vector3 create() => Vector3._();
  @$core.override
  Vector3 createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Vector3 getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Vector3>(create);
  static Vector3? _defaultInstance;

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
  $core.double get z => $_getN(2);
  @$pb.TagNumber(3)
  set z($core.double value) => $_setDouble(2, value);
  @$pb.TagNumber(3)
  $core.bool hasZ() => $_has(2);
  @$pb.TagNumber(3)
  void clearZ() => $_clearField(3);
}

class Pose extends $pb.GeneratedMessage {
  factory Pose({
    Point? position,
    Quaternion? orientation,
    $core.double? yaw,
    $core.double? pitch,
    $core.double? roll,
  }) {
    final result = create();
    if (position != null) result.position = position;
    if (orientation != null) result.orientation = orientation;
    if (yaw != null) result.yaw = yaw;
    if (pitch != null) result.pitch = pitch;
    if (roll != null) result.roll = roll;
    return result;
  }

  Pose._();

  factory Pose.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Pose.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Pose',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Point>(1, _omitFieldNames ? '' : 'position', subBuilder: Point.create)
    ..aOM<Quaternion>(2, _omitFieldNames ? '' : 'orientation',
        subBuilder: Quaternion.create)
    ..aD(3, _omitFieldNames ? '' : 'yaw')
    ..aD(4, _omitFieldNames ? '' : 'pitch')
    ..aD(5, _omitFieldNames ? '' : 'roll')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Pose clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Pose copyWith(void Function(Pose) updates) =>
      super.copyWith((message) => updates(message as Pose)) as Pose;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Pose create() => Pose._();
  @$core.override
  Pose createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Pose getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Pose>(create);
  static Pose? _defaultInstance;

  @$pb.TagNumber(1)
  Point get position => $_getN(0);
  @$pb.TagNumber(1)
  set position(Point value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasPosition() => $_has(0);
  @$pb.TagNumber(1)
  void clearPosition() => $_clearField(1);
  @$pb.TagNumber(1)
  Point ensurePosition() => $_ensure(0);

  @$pb.TagNumber(2)
  Quaternion get orientation => $_getN(1);
  @$pb.TagNumber(2)
  set orientation(Quaternion value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasOrientation() => $_has(1);
  @$pb.TagNumber(2)
  void clearOrientation() => $_clearField(2);
  @$pb.TagNumber(2)
  Quaternion ensureOrientation() => $_ensure(1);

  @$pb.TagNumber(3)
  $core.double get yaw => $_getN(2);
  @$pb.TagNumber(3)
  set yaw($core.double value) => $_setDouble(2, value);
  @$pb.TagNumber(3)
  $core.bool hasYaw() => $_has(2);
  @$pb.TagNumber(3)
  void clearYaw() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.double get pitch => $_getN(3);
  @$pb.TagNumber(4)
  set pitch($core.double value) => $_setDouble(3, value);
  @$pb.TagNumber(4)
  $core.bool hasPitch() => $_has(3);
  @$pb.TagNumber(4)
  void clearPitch() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.double get roll => $_getN(4);
  @$pb.TagNumber(5)
  set roll($core.double value) => $_setDouble(4, value);
  @$pb.TagNumber(5)
  $core.bool hasRoll() => $_has(4);
  @$pb.TagNumber(5)
  void clearRoll() => $_clearField(5);
}

class Twist extends $pb.GeneratedMessage {
  factory Twist({
    Vector3? linear,
    Vector3? angular,
  }) {
    final result = create();
    if (linear != null) result.linear = linear;
    if (angular != null) result.angular = angular;
    return result;
  }

  Twist._();

  factory Twist.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Twist.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Twist',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Vector3>(1, _omitFieldNames ? '' : 'linear',
        subBuilder: Vector3.create)
    ..aOM<Vector3>(2, _omitFieldNames ? '' : 'angular',
        subBuilder: Vector3.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Twist clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Twist copyWith(void Function(Twist) updates) =>
      super.copyWith((message) => updates(message as Twist)) as Twist;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Twist create() => Twist._();
  @$core.override
  Twist createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Twist getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Twist>(create);
  static Twist? _defaultInstance;

  @$pb.TagNumber(1)
  Vector3 get linear => $_getN(0);
  @$pb.TagNumber(1)
  set linear(Vector3 value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasLinear() => $_has(0);
  @$pb.TagNumber(1)
  void clearLinear() => $_clearField(1);
  @$pb.TagNumber(1)
  Vector3 ensureLinear() => $_ensure(0);

  @$pb.TagNumber(2)
  Vector3 get angular => $_getN(1);
  @$pb.TagNumber(2)
  set angular(Vector3 value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasAngular() => $_has(1);
  @$pb.TagNumber(2)
  void clearAngular() => $_clearField(2);
  @$pb.TagNumber(2)
  Vector3 ensureAngular() => $_ensure(1);
}

class PoseStamped extends $pb.GeneratedMessage {
  factory PoseStamped({
    Header? header,
    Pose? pose,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (pose != null) result.pose = pose;
    return result;
  }

  PoseStamped._();

  factory PoseStamped.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory PoseStamped.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'PoseStamped',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Header>(1, _omitFieldNames ? '' : 'header', subBuilder: Header.create)
    ..aOM<Pose>(2, _omitFieldNames ? '' : 'pose', subBuilder: Pose.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PoseStamped clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PoseStamped copyWith(void Function(PoseStamped) updates) =>
      super.copyWith((message) => updates(message as PoseStamped))
          as PoseStamped;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static PoseStamped create() => PoseStamped._();
  @$core.override
  PoseStamped createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static PoseStamped getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<PoseStamped>(create);
  static PoseStamped? _defaultInstance;

  @$pb.TagNumber(1)
  Header get header => $_getN(0);
  @$pb.TagNumber(1)
  set header(Header value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasHeader() => $_has(0);
  @$pb.TagNumber(1)
  void clearHeader() => $_clearField(1);
  @$pb.TagNumber(1)
  Header ensureHeader() => $_ensure(0);

  @$pb.TagNumber(2)
  Pose get pose => $_getN(1);
  @$pb.TagNumber(2)
  set pose(Pose value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasPose() => $_has(1);
  @$pb.TagNumber(2)
  void clearPose() => $_clearField(2);
  @$pb.TagNumber(2)
  Pose ensurePose() => $_ensure(1);
}

class PoseWithCovariance extends $pb.GeneratedMessage {
  factory PoseWithCovariance({
    Pose? pose,
    $core.Iterable<$core.double>? covariance,
  }) {
    final result = create();
    if (pose != null) result.pose = pose;
    if (covariance != null) result.covariance.addAll(covariance);
    return result;
  }

  PoseWithCovariance._();

  factory PoseWithCovariance.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory PoseWithCovariance.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'PoseWithCovariance',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Pose>(1, _omitFieldNames ? '' : 'pose', subBuilder: Pose.create)
    ..p<$core.double>(
        2, _omitFieldNames ? '' : 'covariance', $pb.PbFieldType.KD)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PoseWithCovariance clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PoseWithCovariance copyWith(void Function(PoseWithCovariance) updates) =>
      super.copyWith((message) => updates(message as PoseWithCovariance))
          as PoseWithCovariance;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static PoseWithCovariance create() => PoseWithCovariance._();
  @$core.override
  PoseWithCovariance createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static PoseWithCovariance getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<PoseWithCovariance>(create);
  static PoseWithCovariance? _defaultInstance;

  @$pb.TagNumber(1)
  Pose get pose => $_getN(0);
  @$pb.TagNumber(1)
  set pose(Pose value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasPose() => $_has(0);
  @$pb.TagNumber(1)
  void clearPose() => $_clearField(1);
  @$pb.TagNumber(1)
  Pose ensurePose() => $_ensure(0);

  @$pb.TagNumber(2)
  $pb.PbList<$core.double> get covariance => $_getList(1);
}

class PoseWithCovarianceStamped extends $pb.GeneratedMessage {
  factory PoseWithCovarianceStamped({
    Header? header,
    PoseWithCovariance? pose,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (pose != null) result.pose = pose;
    return result;
  }

  PoseWithCovarianceStamped._();

  factory PoseWithCovarianceStamped.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory PoseWithCovarianceStamped.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'PoseWithCovarianceStamped',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Header>(1, _omitFieldNames ? '' : 'header', subBuilder: Header.create)
    ..aOM<PoseWithCovariance>(2, _omitFieldNames ? '' : 'pose',
        subBuilder: PoseWithCovariance.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PoseWithCovarianceStamped clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PoseWithCovarianceStamped copyWith(
          void Function(PoseWithCovarianceStamped) updates) =>
      super.copyWith((message) => updates(message as PoseWithCovarianceStamped))
          as PoseWithCovarianceStamped;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static PoseWithCovarianceStamped create() => PoseWithCovarianceStamped._();
  @$core.override
  PoseWithCovarianceStamped createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static PoseWithCovarianceStamped getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<PoseWithCovarianceStamped>(create);
  static PoseWithCovarianceStamped? _defaultInstance;

  @$pb.TagNumber(1)
  Header get header => $_getN(0);
  @$pb.TagNumber(1)
  set header(Header value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasHeader() => $_has(0);
  @$pb.TagNumber(1)
  void clearHeader() => $_clearField(1);
  @$pb.TagNumber(1)
  Header ensureHeader() => $_ensure(0);

  @$pb.TagNumber(2)
  PoseWithCovariance get pose => $_getN(1);
  @$pb.TagNumber(2)
  set pose(PoseWithCovariance value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasPose() => $_has(1);
  @$pb.TagNumber(2)
  void clearPose() => $_clearField(2);
  @$pb.TagNumber(2)
  PoseWithCovariance ensurePose() => $_ensure(1);
}

class Polygon extends $pb.GeneratedMessage {
  factory Polygon({
    $core.Iterable<Point>? points,
  }) {
    final result = create();
    if (points != null) result.points.addAll(points);
    return result;
  }

  Polygon._();

  factory Polygon.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Polygon.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Polygon',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..pPM<Point>(1, _omitFieldNames ? '' : 'points', subBuilder: Point.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Polygon clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Polygon copyWith(void Function(Polygon) updates) =>
      super.copyWith((message) => updates(message as Polygon)) as Polygon;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Polygon create() => Polygon._();
  @$core.override
  Polygon createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Polygon getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Polygon>(create);
  static Polygon? _defaultInstance;

  @$pb.TagNumber(1)
  $pb.PbList<Point> get points => $_getList(0);
}

class PolygonStamped extends $pb.GeneratedMessage {
  factory PolygonStamped({
    Header? header,
    Polygon? polygon,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (polygon != null) result.polygon = polygon;
    return result;
  }

  PolygonStamped._();

  factory PolygonStamped.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory PolygonStamped.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'PolygonStamped',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Header>(1, _omitFieldNames ? '' : 'header', subBuilder: Header.create)
    ..aOM<Polygon>(2, _omitFieldNames ? '' : 'polygon',
        subBuilder: Polygon.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PolygonStamped clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PolygonStamped copyWith(void Function(PolygonStamped) updates) =>
      super.copyWith((message) => updates(message as PolygonStamped))
          as PolygonStamped;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static PolygonStamped create() => PolygonStamped._();
  @$core.override
  PolygonStamped createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static PolygonStamped getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<PolygonStamped>(create);
  static PolygonStamped? _defaultInstance;

  @$pb.TagNumber(1)
  Header get header => $_getN(0);
  @$pb.TagNumber(1)
  set header(Header value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasHeader() => $_has(0);
  @$pb.TagNumber(1)
  void clearHeader() => $_clearField(1);
  @$pb.TagNumber(1)
  Header ensureHeader() => $_ensure(0);

  @$pb.TagNumber(2)
  Polygon get polygon => $_getN(1);
  @$pb.TagNumber(2)
  set polygon(Polygon value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasPolygon() => $_has(1);
  @$pb.TagNumber(2)
  void clearPolygon() => $_clearField(2);
  @$pb.TagNumber(2)
  Polygon ensurePolygon() => $_ensure(1);
}

class Transform extends $pb.GeneratedMessage {
  factory Transform({
    Vector3? translation,
    Quaternion? rotation,
  }) {
    final result = create();
    if (translation != null) result.translation = translation;
    if (rotation != null) result.rotation = rotation;
    return result;
  }

  Transform._();

  factory Transform.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory Transform.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Transform',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Vector3>(1, _omitFieldNames ? '' : 'translation',
        subBuilder: Vector3.create)
    ..aOM<Quaternion>(2, _omitFieldNames ? '' : 'rotation',
        subBuilder: Quaternion.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Transform clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Transform copyWith(void Function(Transform) updates) =>
      super.copyWith((message) => updates(message as Transform)) as Transform;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Transform create() => Transform._();
  @$core.override
  Transform createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static Transform getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Transform>(create);
  static Transform? _defaultInstance;

  @$pb.TagNumber(1)
  Vector3 get translation => $_getN(0);
  @$pb.TagNumber(1)
  set translation(Vector3 value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasTranslation() => $_has(0);
  @$pb.TagNumber(1)
  void clearTranslation() => $_clearField(1);
  @$pb.TagNumber(1)
  Vector3 ensureTranslation() => $_ensure(0);

  @$pb.TagNumber(2)
  Quaternion get rotation => $_getN(1);
  @$pb.TagNumber(2)
  set rotation(Quaternion value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasRotation() => $_has(1);
  @$pb.TagNumber(2)
  void clearRotation() => $_clearField(2);
  @$pb.TagNumber(2)
  Quaternion ensureRotation() => $_ensure(1);
}

class TransformStamped extends $pb.GeneratedMessage {
  factory TransformStamped({
    Header? header,
    $core.String? childFrameId,
    Transform? transform,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (childFrameId != null) result.childFrameId = childFrameId;
    if (transform != null) result.transform = transform;
    return result;
  }

  TransformStamped._();

  factory TransformStamped.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory TransformStamped.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TransformStamped',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<Header>(1, _omitFieldNames ? '' : 'header', subBuilder: Header.create)
    ..aOS(2, _omitFieldNames ? '' : 'childFrameId')
    ..aOM<Transform>(3, _omitFieldNames ? '' : 'transform',
        subBuilder: Transform.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TransformStamped clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TransformStamped copyWith(void Function(TransformStamped) updates) =>
      super.copyWith((message) => updates(message as TransformStamped))
          as TransformStamped;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static TransformStamped create() => TransformStamped._();
  @$core.override
  TransformStamped createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static TransformStamped getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TransformStamped>(create);
  static TransformStamped? _defaultInstance;

  @$pb.TagNumber(1)
  Header get header => $_getN(0);
  @$pb.TagNumber(1)
  set header(Header value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasHeader() => $_has(0);
  @$pb.TagNumber(1)
  void clearHeader() => $_clearField(1);
  @$pb.TagNumber(1)
  Header ensureHeader() => $_ensure(0);

  @$pb.TagNumber(2)
  $core.String get childFrameId => $_getSZ(1);
  @$pb.TagNumber(2)
  set childFrameId($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasChildFrameId() => $_has(1);
  @$pb.TagNumber(2)
  void clearChildFrameId() => $_clearField(2);

  @$pb.TagNumber(3)
  Transform get transform => $_getN(2);
  @$pb.TagNumber(3)
  set transform(Transform value) => $_setField(3, value);
  @$pb.TagNumber(3)
  $core.bool hasTransform() => $_has(2);
  @$pb.TagNumber(3)
  void clearTransform() => $_clearField(3);
  @$pb.TagNumber(3)
  Transform ensureTransform() => $_ensure(2);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
