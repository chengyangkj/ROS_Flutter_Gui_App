// This is a generated file - do not edit.
//
// Generated from sensor_msg.proto.

// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_relative_imports
// ignore_for_file: unused_import

import 'dart:convert' as $convert;
import 'dart:core' as $core;
import 'dart:typed_data' as $typed_data;

@$core.Deprecated('Use laserScanDescriptor instead')
const LaserScan$json = {
  '1': 'LaserScan',
  '2': [
    {
      '1': 'header',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Header',
      '10': 'header'
    },
    {'1': 'angle_min', '3': 2, '4': 1, '5': 2, '10': 'angleMin'},
    {'1': 'angle_max', '3': 3, '4': 1, '5': 2, '10': 'angleMax'},
    {'1': 'angle_increment', '3': 4, '4': 1, '5': 2, '10': 'angleIncrement'},
    {'1': 'time_increment', '3': 5, '4': 1, '5': 2, '10': 'timeIncrement'},
    {'1': 'scan_time', '3': 6, '4': 1, '5': 2, '10': 'scanTime'},
    {'1': 'range_min', '3': 7, '4': 1, '5': 2, '10': 'rangeMin'},
    {'1': 'range_max', '3': 8, '4': 1, '5': 2, '10': 'rangeMax'},
    {'1': 'ranges', '3': 9, '4': 3, '5': 2, '10': 'ranges'},
    {'1': 'intensities', '3': 10, '4': 3, '5': 2, '10': 'intensities'},
  ],
};

/// Descriptor for `LaserScan`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List laserScanDescriptor = $convert.base64Decode(
    'CglMYXNlclNjYW4SMgoGaGVhZGVyGAEgASgLMhoucm9zX2d1aV9iYWNrZW5kLnBiLkhlYWRlcl'
    'IGaGVhZGVyEhsKCWFuZ2xlX21pbhgCIAEoAlIIYW5nbGVNaW4SGwoJYW5nbGVfbWF4GAMgASgC'
    'UghhbmdsZU1heBInCg9hbmdsZV9pbmNyZW1lbnQYBCABKAJSDmFuZ2xlSW5jcmVtZW50EiUKDn'
    'RpbWVfaW5jcmVtZW50GAUgASgCUg10aW1lSW5jcmVtZW50EhsKCXNjYW5fdGltZRgGIAEoAlII'
    'c2NhblRpbWUSGwoJcmFuZ2VfbWluGAcgASgCUghyYW5nZU1pbhIbCglyYW5nZV9tYXgYCCABKA'
    'JSCHJhbmdlTWF4EhYKBnJhbmdlcxgJIAMoAlIGcmFuZ2VzEiAKC2ludGVuc2l0aWVzGAogAygC'
    'UgtpbnRlbnNpdGllcw==');

@$core.Deprecated('Use batteryStateDescriptor instead')
const BatteryState$json = {
  '1': 'BatteryState',
  '2': [
    {
      '1': 'header',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Header',
      '10': 'header'
    },
    {'1': 'voltage', '3': 2, '4': 1, '5': 2, '10': 'voltage'},
    {'1': 'temperature', '3': 3, '4': 1, '5': 2, '10': 'temperature'},
    {'1': 'current', '3': 4, '4': 1, '5': 2, '10': 'current'},
    {'1': 'charge', '3': 5, '4': 1, '5': 2, '10': 'charge'},
    {'1': 'capacity', '3': 6, '4': 1, '5': 2, '10': 'capacity'},
    {'1': 'design_capacity', '3': 7, '4': 1, '5': 2, '10': 'designCapacity'},
    {'1': 'percentage', '3': 8, '4': 1, '5': 5, '10': 'percentage'},
    {
      '1': 'power_supply_status',
      '3': 9,
      '4': 1,
      '5': 5,
      '10': 'powerSupplyStatus'
    },
    {
      '1': 'power_supply_health',
      '3': 10,
      '4': 1,
      '5': 5,
      '10': 'powerSupplyHealth'
    },
    {
      '1': 'power_supply_technology',
      '3': 11,
      '4': 1,
      '5': 5,
      '10': 'powerSupplyTechnology'
    },
    {'1': 'present', '3': 12, '4': 1, '5': 8, '10': 'present'},
    {'1': 'cell_voltage', '3': 13, '4': 3, '5': 2, '10': 'cellVoltage'},
    {'1': 'location', '3': 14, '4': 1, '5': 9, '10': 'location'},
    {'1': 'serial_number', '3': 15, '4': 1, '5': 9, '10': 'serialNumber'},
  ],
};

/// Descriptor for `BatteryState`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List batteryStateDescriptor = $convert.base64Decode(
    'CgxCYXR0ZXJ5U3RhdGUSMgoGaGVhZGVyGAEgASgLMhoucm9zX2d1aV9iYWNrZW5kLnBiLkhlYW'
    'RlclIGaGVhZGVyEhgKB3ZvbHRhZ2UYAiABKAJSB3ZvbHRhZ2USIAoLdGVtcGVyYXR1cmUYAyAB'
    'KAJSC3RlbXBlcmF0dXJlEhgKB2N1cnJlbnQYBCABKAJSB2N1cnJlbnQSFgoGY2hhcmdlGAUgAS'
    'gCUgZjaGFyZ2USGgoIY2FwYWNpdHkYBiABKAJSCGNhcGFjaXR5EicKD2Rlc2lnbl9jYXBhY2l0'
    'eRgHIAEoAlIOZGVzaWduQ2FwYWNpdHkSHgoKcGVyY2VudGFnZRgIIAEoBVIKcGVyY2VudGFnZR'
    'IuChNwb3dlcl9zdXBwbHlfc3RhdHVzGAkgASgFUhFwb3dlclN1cHBseVN0YXR1cxIuChNwb3dl'
    'cl9zdXBwbHlfaGVhbHRoGAogASgFUhFwb3dlclN1cHBseUhlYWx0aBI2Chdwb3dlcl9zdXBwbH'
    'lfdGVjaG5vbG9neRgLIAEoBVIVcG93ZXJTdXBwbHlUZWNobm9sb2d5EhgKB3ByZXNlbnQYDCAB'
    'KAhSB3ByZXNlbnQSIQoMY2VsbF92b2x0YWdlGA0gAygCUgtjZWxsVm9sdGFnZRIaCghsb2NhdG'
    'lvbhgOIAEoCVIIbG9jYXRpb24SIwoNc2VyaWFsX251bWJlchgPIAEoCVIMc2VyaWFsTnVtYmVy');

@$core.Deprecated('Use pointCloud2MapFrameDescriptor instead')
const PointCloud2MapFrame$json = {
  '1': 'PointCloud2MapFrame',
  '2': [
    {
      '1': 'header',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Header',
      '10': 'header'
    },
    {'1': 'x', '3': 2, '4': 3, '5': 1, '10': 'x'},
    {'1': 'y', '3': 3, '4': 3, '5': 1, '10': 'y'},
    {'1': 'z', '3': 4, '4': 3, '5': 1, '10': 'z'},
  ],
};

/// Descriptor for `PointCloud2MapFrame`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List pointCloud2MapFrameDescriptor = $convert.base64Decode(
    'ChNQb2ludENsb3VkMk1hcEZyYW1lEjIKBmhlYWRlchgBIAEoCzIaLnJvc19ndWlfYmFja2VuZC'
    '5wYi5IZWFkZXJSBmhlYWRlchIMCgF4GAIgAygBUgF4EgwKAXkYAyADKAFSAXkSDAoBehgEIAMo'
    'AVIBeg==');

@$core.Deprecated('Use laserScanBaseLinkDescriptor instead')
const LaserScanBaseLink$json = {
  '1': 'LaserScanBaseLink',
  '2': [
    {
      '1': 'header',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Header',
      '10': 'header'
    },
    {'1': 'x', '3': 2, '4': 3, '5': 2, '10': 'x'},
    {'1': 'y', '3': 3, '4': 3, '5': 2, '10': 'y'},
  ],
};

/// Descriptor for `LaserScanBaseLink`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List laserScanBaseLinkDescriptor = $convert.base64Decode(
    'ChFMYXNlclNjYW5CYXNlTGluaxIyCgZoZWFkZXIYASABKAsyGi5yb3NfZ3VpX2JhY2tlbmQucG'
    'IuSGVhZGVyUgZoZWFkZXISDAoBeBgCIAMoAlIBeBIMCgF5GAMgAygCUgF5');
