// This is a generated file - do not edit.
//
// Generated from diagnostic_msgs.proto.

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

class KeyValue extends $pb.GeneratedMessage {
  factory KeyValue({
    $core.String? key,
    $core.String? value,
  }) {
    final result = create();
    if (key != null) result.key = key;
    if (value != null) result.value = value;
    return result;
  }

  KeyValue._();

  factory KeyValue.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory KeyValue.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'KeyValue',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'key')
    ..aOS(2, _omitFieldNames ? '' : 'value')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  KeyValue clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  KeyValue copyWith(void Function(KeyValue) updates) =>
      super.copyWith((message) => updates(message as KeyValue)) as KeyValue;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static KeyValue create() => KeyValue._();
  @$core.override
  KeyValue createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static KeyValue getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<KeyValue>(create);
  static KeyValue? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get key => $_getSZ(0);
  @$pb.TagNumber(1)
  set key($core.String value) => $_setString(0, value);
  @$pb.TagNumber(1)
  $core.bool hasKey() => $_has(0);
  @$pb.TagNumber(1)
  void clearKey() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get value => $_getSZ(1);
  @$pb.TagNumber(2)
  set value($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasValue() => $_has(1);
  @$pb.TagNumber(2)
  void clearValue() => $_clearField(2);
}

class DiagnosticStatus extends $pb.GeneratedMessage {
  factory DiagnosticStatus({
    $core.int? level,
    $core.String? name,
    $core.String? message,
    $core.String? hardwareId,
    $core.Iterable<KeyValue>? values,
  }) {
    final result = create();
    if (level != null) result.level = level;
    if (name != null) result.name = name;
    if (message != null) result.message = message;
    if (hardwareId != null) result.hardwareId = hardwareId;
    if (values != null) result.values.addAll(values);
    return result;
  }

  DiagnosticStatus._();

  factory DiagnosticStatus.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory DiagnosticStatus.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'DiagnosticStatus',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aI(1, _omitFieldNames ? '' : 'level')
    ..aOS(2, _omitFieldNames ? '' : 'name')
    ..aOS(3, _omitFieldNames ? '' : 'message')
    ..aOS(4, _omitFieldNames ? '' : 'hardwareId')
    ..pPM<KeyValue>(5, _omitFieldNames ? '' : 'values',
        subBuilder: KeyValue.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DiagnosticStatus clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DiagnosticStatus copyWith(void Function(DiagnosticStatus) updates) =>
      super.copyWith((message) => updates(message as DiagnosticStatus))
          as DiagnosticStatus;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static DiagnosticStatus create() => DiagnosticStatus._();
  @$core.override
  DiagnosticStatus createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static DiagnosticStatus getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<DiagnosticStatus>(create);
  static DiagnosticStatus? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get level => $_getIZ(0);
  @$pb.TagNumber(1)
  set level($core.int value) => $_setSignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasLevel() => $_has(0);
  @$pb.TagNumber(1)
  void clearLevel() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.String get name => $_getSZ(1);
  @$pb.TagNumber(2)
  set name($core.String value) => $_setString(1, value);
  @$pb.TagNumber(2)
  $core.bool hasName() => $_has(1);
  @$pb.TagNumber(2)
  void clearName() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.String get message => $_getSZ(2);
  @$pb.TagNumber(3)
  set message($core.String value) => $_setString(2, value);
  @$pb.TagNumber(3)
  $core.bool hasMessage() => $_has(2);
  @$pb.TagNumber(3)
  void clearMessage() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.String get hardwareId => $_getSZ(3);
  @$pb.TagNumber(4)
  set hardwareId($core.String value) => $_setString(3, value);
  @$pb.TagNumber(4)
  $core.bool hasHardwareId() => $_has(3);
  @$pb.TagNumber(4)
  void clearHardwareId() => $_clearField(4);

  @$pb.TagNumber(5)
  $pb.PbList<KeyValue> get values => $_getList(4);
}

class DiagnosticArray extends $pb.GeneratedMessage {
  factory DiagnosticArray({
    $0.Header? header,
    $core.Iterable<DiagnosticStatus>? status,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (status != null) result.status.addAll(status);
    return result;
  }

  DiagnosticArray._();

  factory DiagnosticArray.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory DiagnosticArray.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'DiagnosticArray',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..pPM<DiagnosticStatus>(2, _omitFieldNames ? '' : 'status',
        subBuilder: DiagnosticStatus.create)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DiagnosticArray clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DiagnosticArray copyWith(void Function(DiagnosticArray) updates) =>
      super.copyWith((message) => updates(message as DiagnosticArray))
          as DiagnosticArray;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static DiagnosticArray create() => DiagnosticArray._();
  @$core.override
  DiagnosticArray createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static DiagnosticArray getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<DiagnosticArray>(create);
  static DiagnosticArray? _defaultInstance;

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
  $pb.PbList<DiagnosticStatus> get status => $_getList(1);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
