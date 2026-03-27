// This is a generated file - do not edit.
//
// Generated from sensor_msg.proto.

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

class LaserScan extends $pb.GeneratedMessage {
  factory LaserScan({
    $0.Header? header,
    $core.double? angleMin,
    $core.double? angleMax,
    $core.double? angleIncrement,
    $core.double? timeIncrement,
    $core.double? scanTime,
    $core.double? rangeMin,
    $core.double? rangeMax,
    $core.Iterable<$core.double>? ranges,
    $core.Iterable<$core.double>? intensities,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (angleMin != null) result.angleMin = angleMin;
    if (angleMax != null) result.angleMax = angleMax;
    if (angleIncrement != null) result.angleIncrement = angleIncrement;
    if (timeIncrement != null) result.timeIncrement = timeIncrement;
    if (scanTime != null) result.scanTime = scanTime;
    if (rangeMin != null) result.rangeMin = rangeMin;
    if (rangeMax != null) result.rangeMax = rangeMax;
    if (ranges != null) result.ranges.addAll(ranges);
    if (intensities != null) result.intensities.addAll(intensities);
    return result;
  }

  LaserScan._();

  factory LaserScan.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory LaserScan.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'LaserScan',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..aD(2, _omitFieldNames ? '' : 'angleMin', fieldType: $pb.PbFieldType.OF)
    ..aD(3, _omitFieldNames ? '' : 'angleMax', fieldType: $pb.PbFieldType.OF)
    ..aD(4, _omitFieldNames ? '' : 'angleIncrement',
        fieldType: $pb.PbFieldType.OF)
    ..aD(5, _omitFieldNames ? '' : 'timeIncrement',
        fieldType: $pb.PbFieldType.OF)
    ..aD(6, _omitFieldNames ? '' : 'scanTime', fieldType: $pb.PbFieldType.OF)
    ..aD(7, _omitFieldNames ? '' : 'rangeMin', fieldType: $pb.PbFieldType.OF)
    ..aD(8, _omitFieldNames ? '' : 'rangeMax', fieldType: $pb.PbFieldType.OF)
    ..p<$core.double>(9, _omitFieldNames ? '' : 'ranges', $pb.PbFieldType.KF)
    ..p<$core.double>(
        10, _omitFieldNames ? '' : 'intensities', $pb.PbFieldType.KF)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  LaserScan clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  LaserScan copyWith(void Function(LaserScan) updates) =>
      super.copyWith((message) => updates(message as LaserScan)) as LaserScan;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static LaserScan create() => LaserScan._();
  @$core.override
  LaserScan createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static LaserScan getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<LaserScan>(create);
  static LaserScan? _defaultInstance;

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
  $core.double get angleMin => $_getN(1);
  @$pb.TagNumber(2)
  set angleMin($core.double value) => $_setFloat(1, value);
  @$pb.TagNumber(2)
  $core.bool hasAngleMin() => $_has(1);
  @$pb.TagNumber(2)
  void clearAngleMin() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.double get angleMax => $_getN(2);
  @$pb.TagNumber(3)
  set angleMax($core.double value) => $_setFloat(2, value);
  @$pb.TagNumber(3)
  $core.bool hasAngleMax() => $_has(2);
  @$pb.TagNumber(3)
  void clearAngleMax() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.double get angleIncrement => $_getN(3);
  @$pb.TagNumber(4)
  set angleIncrement($core.double value) => $_setFloat(3, value);
  @$pb.TagNumber(4)
  $core.bool hasAngleIncrement() => $_has(3);
  @$pb.TagNumber(4)
  void clearAngleIncrement() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.double get timeIncrement => $_getN(4);
  @$pb.TagNumber(5)
  set timeIncrement($core.double value) => $_setFloat(4, value);
  @$pb.TagNumber(5)
  $core.bool hasTimeIncrement() => $_has(4);
  @$pb.TagNumber(5)
  void clearTimeIncrement() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.double get scanTime => $_getN(5);
  @$pb.TagNumber(6)
  set scanTime($core.double value) => $_setFloat(5, value);
  @$pb.TagNumber(6)
  $core.bool hasScanTime() => $_has(5);
  @$pb.TagNumber(6)
  void clearScanTime() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.double get rangeMin => $_getN(6);
  @$pb.TagNumber(7)
  set rangeMin($core.double value) => $_setFloat(6, value);
  @$pb.TagNumber(7)
  $core.bool hasRangeMin() => $_has(6);
  @$pb.TagNumber(7)
  void clearRangeMin() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.double get rangeMax => $_getN(7);
  @$pb.TagNumber(8)
  set rangeMax($core.double value) => $_setFloat(7, value);
  @$pb.TagNumber(8)
  $core.bool hasRangeMax() => $_has(7);
  @$pb.TagNumber(8)
  void clearRangeMax() => $_clearField(8);

  @$pb.TagNumber(9)
  $pb.PbList<$core.double> get ranges => $_getList(8);

  @$pb.TagNumber(10)
  $pb.PbList<$core.double> get intensities => $_getList(9);
}

class BatteryState extends $pb.GeneratedMessage {
  factory BatteryState({
    $0.Header? header,
    $core.double? voltage,
    $core.double? temperature,
    $core.double? current,
    $core.double? charge,
    $core.double? capacity,
    $core.double? designCapacity,
    $core.int? percentage,
    $core.int? powerSupplyStatus,
    $core.int? powerSupplyHealth,
    $core.int? powerSupplyTechnology,
    $core.bool? present,
    $core.Iterable<$core.double>? cellVoltage,
    $core.String? location,
    $core.String? serialNumber,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (voltage != null) result.voltage = voltage;
    if (temperature != null) result.temperature = temperature;
    if (current != null) result.current = current;
    if (charge != null) result.charge = charge;
    if (capacity != null) result.capacity = capacity;
    if (designCapacity != null) result.designCapacity = designCapacity;
    if (percentage != null) result.percentage = percentage;
    if (powerSupplyStatus != null) result.powerSupplyStatus = powerSupplyStatus;
    if (powerSupplyHealth != null) result.powerSupplyHealth = powerSupplyHealth;
    if (powerSupplyTechnology != null)
      result.powerSupplyTechnology = powerSupplyTechnology;
    if (present != null) result.present = present;
    if (cellVoltage != null) result.cellVoltage.addAll(cellVoltage);
    if (location != null) result.location = location;
    if (serialNumber != null) result.serialNumber = serialNumber;
    return result;
  }

  BatteryState._();

  factory BatteryState.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory BatteryState.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'BatteryState',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..aD(2, _omitFieldNames ? '' : 'voltage', fieldType: $pb.PbFieldType.OF)
    ..aD(3, _omitFieldNames ? '' : 'temperature', fieldType: $pb.PbFieldType.OF)
    ..aD(4, _omitFieldNames ? '' : 'current', fieldType: $pb.PbFieldType.OF)
    ..aD(5, _omitFieldNames ? '' : 'charge', fieldType: $pb.PbFieldType.OF)
    ..aD(6, _omitFieldNames ? '' : 'capacity', fieldType: $pb.PbFieldType.OF)
    ..aD(7, _omitFieldNames ? '' : 'designCapacity',
        fieldType: $pb.PbFieldType.OF)
    ..aI(8, _omitFieldNames ? '' : 'percentage')
    ..aI(9, _omitFieldNames ? '' : 'powerSupplyStatus')
    ..aI(10, _omitFieldNames ? '' : 'powerSupplyHealth')
    ..aI(11, _omitFieldNames ? '' : 'powerSupplyTechnology')
    ..aOB(12, _omitFieldNames ? '' : 'present')
    ..p<$core.double>(
        13, _omitFieldNames ? '' : 'cellVoltage', $pb.PbFieldType.KF)
    ..aOS(14, _omitFieldNames ? '' : 'location')
    ..aOS(15, _omitFieldNames ? '' : 'serialNumber')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  BatteryState clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  BatteryState copyWith(void Function(BatteryState) updates) =>
      super.copyWith((message) => updates(message as BatteryState))
          as BatteryState;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static BatteryState create() => BatteryState._();
  @$core.override
  BatteryState createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static BatteryState getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<BatteryState>(create);
  static BatteryState? _defaultInstance;

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
  $core.double get voltage => $_getN(1);
  @$pb.TagNumber(2)
  set voltage($core.double value) => $_setFloat(1, value);
  @$pb.TagNumber(2)
  $core.bool hasVoltage() => $_has(1);
  @$pb.TagNumber(2)
  void clearVoltage() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.double get temperature => $_getN(2);
  @$pb.TagNumber(3)
  set temperature($core.double value) => $_setFloat(2, value);
  @$pb.TagNumber(3)
  $core.bool hasTemperature() => $_has(2);
  @$pb.TagNumber(3)
  void clearTemperature() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.double get current => $_getN(3);
  @$pb.TagNumber(4)
  set current($core.double value) => $_setFloat(3, value);
  @$pb.TagNumber(4)
  $core.bool hasCurrent() => $_has(3);
  @$pb.TagNumber(4)
  void clearCurrent() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.double get charge => $_getN(4);
  @$pb.TagNumber(5)
  set charge($core.double value) => $_setFloat(4, value);
  @$pb.TagNumber(5)
  $core.bool hasCharge() => $_has(4);
  @$pb.TagNumber(5)
  void clearCharge() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.double get capacity => $_getN(5);
  @$pb.TagNumber(6)
  set capacity($core.double value) => $_setFloat(5, value);
  @$pb.TagNumber(6)
  $core.bool hasCapacity() => $_has(5);
  @$pb.TagNumber(6)
  void clearCapacity() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.double get designCapacity => $_getN(6);
  @$pb.TagNumber(7)
  set designCapacity($core.double value) => $_setFloat(6, value);
  @$pb.TagNumber(7)
  $core.bool hasDesignCapacity() => $_has(6);
  @$pb.TagNumber(7)
  void clearDesignCapacity() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.int get percentage => $_getIZ(7);
  @$pb.TagNumber(8)
  set percentage($core.int value) => $_setSignedInt32(7, value);
  @$pb.TagNumber(8)
  $core.bool hasPercentage() => $_has(7);
  @$pb.TagNumber(8)
  void clearPercentage() => $_clearField(8);

  @$pb.TagNumber(9)
  $core.int get powerSupplyStatus => $_getIZ(8);
  @$pb.TagNumber(9)
  set powerSupplyStatus($core.int value) => $_setSignedInt32(8, value);
  @$pb.TagNumber(9)
  $core.bool hasPowerSupplyStatus() => $_has(8);
  @$pb.TagNumber(9)
  void clearPowerSupplyStatus() => $_clearField(9);

  @$pb.TagNumber(10)
  $core.int get powerSupplyHealth => $_getIZ(9);
  @$pb.TagNumber(10)
  set powerSupplyHealth($core.int value) => $_setSignedInt32(9, value);
  @$pb.TagNumber(10)
  $core.bool hasPowerSupplyHealth() => $_has(9);
  @$pb.TagNumber(10)
  void clearPowerSupplyHealth() => $_clearField(10);

  @$pb.TagNumber(11)
  $core.int get powerSupplyTechnology => $_getIZ(10);
  @$pb.TagNumber(11)
  set powerSupplyTechnology($core.int value) => $_setSignedInt32(10, value);
  @$pb.TagNumber(11)
  $core.bool hasPowerSupplyTechnology() => $_has(10);
  @$pb.TagNumber(11)
  void clearPowerSupplyTechnology() => $_clearField(11);

  @$pb.TagNumber(12)
  $core.bool get present => $_getBF(11);
  @$pb.TagNumber(12)
  set present($core.bool value) => $_setBool(11, value);
  @$pb.TagNumber(12)
  $core.bool hasPresent() => $_has(11);
  @$pb.TagNumber(12)
  void clearPresent() => $_clearField(12);

  @$pb.TagNumber(13)
  $pb.PbList<$core.double> get cellVoltage => $_getList(12);

  @$pb.TagNumber(14)
  $core.String get location => $_getSZ(13);
  @$pb.TagNumber(14)
  set location($core.String value) => $_setString(13, value);
  @$pb.TagNumber(14)
  $core.bool hasLocation() => $_has(13);
  @$pb.TagNumber(14)
  void clearLocation() => $_clearField(14);

  @$pb.TagNumber(15)
  $core.String get serialNumber => $_getSZ(14);
  @$pb.TagNumber(15)
  set serialNumber($core.String value) => $_setString(14, value);
  @$pb.TagNumber(15)
  $core.bool hasSerialNumber() => $_has(14);
  @$pb.TagNumber(15)
  void clearSerialNumber() => $_clearField(15);
}

class PointCloud2MapFrame extends $pb.GeneratedMessage {
  factory PointCloud2MapFrame({
    $0.Header? header,
    $core.Iterable<$core.double>? x,
    $core.Iterable<$core.double>? y,
    $core.Iterable<$core.double>? z,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (x != null) result.x.addAll(x);
    if (y != null) result.y.addAll(y);
    if (z != null) result.z.addAll(z);
    return result;
  }

  PointCloud2MapFrame._();

  factory PointCloud2MapFrame.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory PointCloud2MapFrame.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'PointCloud2MapFrame',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..p<$core.double>(2, _omitFieldNames ? '' : 'x', $pb.PbFieldType.KD)
    ..p<$core.double>(3, _omitFieldNames ? '' : 'y', $pb.PbFieldType.KD)
    ..p<$core.double>(4, _omitFieldNames ? '' : 'z', $pb.PbFieldType.KD)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PointCloud2MapFrame clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  PointCloud2MapFrame copyWith(void Function(PointCloud2MapFrame) updates) =>
      super.copyWith((message) => updates(message as PointCloud2MapFrame))
          as PointCloud2MapFrame;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static PointCloud2MapFrame create() => PointCloud2MapFrame._();
  @$core.override
  PointCloud2MapFrame createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static PointCloud2MapFrame getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<PointCloud2MapFrame>(create);
  static PointCloud2MapFrame? _defaultInstance;

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
  $pb.PbList<$core.double> get x => $_getList(1);

  @$pb.TagNumber(3)
  $pb.PbList<$core.double> get y => $_getList(2);

  @$pb.TagNumber(4)
  $pb.PbList<$core.double> get z => $_getList(3);
}

class LaserScanBaseFrame extends $pb.GeneratedMessage {
  factory LaserScanBaseFrame({
    $0.Header? header,
    $core.Iterable<$core.double>? x,
    $core.Iterable<$core.double>? y,
  }) {
    final result = create();
    if (header != null) result.header = header;
    if (x != null) result.x.addAll(x);
    if (y != null) result.y.addAll(y);
    return result;
  }

  LaserScanBaseFrame._();

  factory LaserScanBaseFrame.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(data, registry);
  factory LaserScanBaseFrame.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'LaserScanBaseFrame',
      package:
          const $pb.PackageName(_omitMessageNames ? '' : 'ros_gui_backend.pb'),
      createEmptyInstance: create)
    ..aOM<$0.Header>(1, _omitFieldNames ? '' : 'header',
        subBuilder: $0.Header.create)
    ..p<$core.double>(2, _omitFieldNames ? '' : 'x', $pb.PbFieldType.KF)
    ..p<$core.double>(3, _omitFieldNames ? '' : 'y', $pb.PbFieldType.KF)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  LaserScanBaseFrame clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  LaserScanBaseFrame copyWith(void Function(LaserScanBaseFrame) updates) =>
      super.copyWith((message) => updates(message as LaserScanBaseFrame))
          as LaserScanBaseFrame;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static LaserScanBaseFrame create() => LaserScanBaseFrame._();
  @$core.override
  LaserScanBaseFrame createEmptyInstance() => create();
  @$core.pragma('dart2js:noInline')
  static LaserScanBaseFrame getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<LaserScanBaseFrame>(create);
  static LaserScanBaseFrame? _defaultInstance;

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
  $pb.PbList<$core.double> get x => $_getList(1);

  @$pb.TagNumber(3)
  $pb.PbList<$core.double> get y => $_getList(2);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
