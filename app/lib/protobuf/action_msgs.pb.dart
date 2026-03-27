// This is a generated file - do not edit.
//
// Generated from action_msgs.proto.

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

class GoalID extends $pb.GeneratedMessage {
  factory GoalID({
    $core.String? uuid,
  }) {
    final result = create();
    if (uuid != null) result.uuid = uuid;
    return result;
  }

  GoalID._();

  factory GoalID.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory GoalID.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'GoalID',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'uuid')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalID clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalID copyWith(void Function(GoalID) updates) =>
      super.copyWith((message) => updates(message as GoalID)) as GoalID;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static GoalID create() => GoalID._();
  @$core.override
  GoalID createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static GoalID getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<GoalID>(create);
  static GoalID? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get uuid => $_getSZ(0);
  @$pb.TagNumber(1)
  set uuid($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasUuid() => $_has(0);
  @$pb.TagNumber(1)
  void clearUuid() => $_clearField(1);
}

class GoalInfo extends $pb.GeneratedMessage {
  factory GoalInfo({
    GoalID? goalId,
    $0.Time? stamp,
  }) {
    final result = create();
    if (goalId != null) result.goalId = goalId;
    if (stamp != null) result.stamp = stamp;
    return result;
  }

  GoalInfo._();

  factory GoalInfo.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory GoalInfo.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'GoalInfo',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<GoalID>(1, _omitFieldNames ? '' : 'goalId', subBuilder: GoalID.create)
    ..aOM<$0.Time>(2, _omitFieldNames ? '' : 'stamp',
        subBuilder: $0.Time.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalInfo clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalInfo copyWith(void Function(GoalInfo) updates) =>
      super.copyWith((message) => updates(message as GoalInfo)) as GoalInfo;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static GoalInfo create() => GoalInfo._();
  @$core.override
  GoalInfo createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static GoalInfo getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<GoalInfo>(create);
  static GoalInfo? _defaultInstance;

  @$pb.TagNumber(1)
  GoalID get goalId => $_getN(0);
  @$pb.TagNumber(1)
  set goalId(GoalID value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasGoalId() => $_has(0);
  @$pb.TagNumber(1)
  void clearGoalId() => $_clearField(1);
  @$pb.TagNumber(1)
  GoalID ensureGoalId() => $_ensure(0);

  @$pb.TagNumber(2)
  $0.Time get stamp => $_getN(1);
  @$pb.TagNumber(2)
  set stamp($0.Time value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasStamp() => $_has(1);
  @$pb.TagNumber(2)
  void clearStamp() => $_clearField(2);
  @$pb.TagNumber(2)
  $0.Time ensureStamp() => $_ensure(1);
}

class GoalStatus extends $pb.GeneratedMessage {
  factory GoalStatus({
    GoalInfo? goalInfo,
    $core.int? status,
    $core.String? text,
  }) {
    final result = create();
    if (goalInfo != null) result.goalInfo = goalInfo;
    if (status != null) result.status = status;
    if (text != null) result.text = text;
    return result;
  }

  GoalStatus._();

  factory GoalStatus.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory GoalStatus.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'GoalStatus',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<GoalInfo>(1, _omitFieldNames ? '' : 'goalInfo',
        subBuilder: GoalInfo.create)
    ..aI(2, _omitFieldNames ? '' : 'status', fieldType: $pb.PbFieldType.OU3)
    ..aOS(3, _omitFieldNames ? '' : 'text')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalStatus clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalStatus copyWith(void Function(GoalStatus) updates) =>
      super.copyWith((message) => updates(message as GoalStatus)) as GoalStatus;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static GoalStatus create() => GoalStatus._();
  @$core.override
  GoalStatus createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static GoalStatus getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<GoalStatus>(create);
  static GoalStatus? _defaultInstance;

  @$pb.TagNumber(1)
  GoalInfo get goalInfo => $_getN(0);
  @$pb.TagNumber(1)
  set goalInfo(GoalInfo value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasGoalInfo() => $_has(0);
  @$pb.TagNumber(1)
  void clearGoalInfo() => $_clearField(1);
  @$pb.TagNumber(1)
  GoalInfo ensureGoalInfo() => $_ensure(0);

  @$pb.TagNumber(2)
  $core.int get status => $_getIZ(1);
  @$pb.TagNumber(2)
  set status($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasStatus() => $_has(1);
  @$pb.TagNumber(2)
  void clearStatus() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.String get text => $_getSZ(2);
  @$pb.TagNumber(3)
  set text($core.String value) => $_setString(2, value);
  @$pb.TagNumber(3)
  $core.bool hasText() => $_has(2);
  @$pb.TagNumber(3)
  void clearText() => $_clearField(3);
}

class GoalStatusArray extends $pb.GeneratedMessage {
  factory GoalStatusArray({
    $0.Header? header,
    $core.Iterable<GoalStatus>? statusList,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (statusList != null) result.statusList.addAll(statusList);
    return result;
  }

  GoalStatusArray._();

  factory GoalStatusArray.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory GoalStatusArray.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'GoalStatusArray',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..pPM<GoalStatus>(2, _omitFieldNames ? '' : 'statusList',
        subBuilder: GoalStatus.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalStatusArray clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  GoalStatusArray copyWith(void Function(GoalStatusArray) updates) =>
      super.copyWith((message) => updates(message as GoalStatusArray))
          as GoalStatusArray;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static GoalStatusArray create() => GoalStatusArray._();
  @$core.override
  GoalStatusArray createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static GoalStatusArray getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<GoalStatusArray>(create);
  static GoalStatusArray? _defaultInstance;

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
  $pb.PbList<GoalStatus> get statusList => $_getList(1);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
