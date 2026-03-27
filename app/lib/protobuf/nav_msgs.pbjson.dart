// This is a generated file - do not edit.
//
// Generated from nav_msgs.proto.

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

@$core.Deprecated('Use mapMetaDataDescriptor instead')
const MapMetaData$json = {
  '1': 'MapMetaData',
  '2': [
    {
      '1': 'map_load_time',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Time',
      '10': 'mapLoadTime'
    },
    {'1': 'resolution', '3': 2, '4': 1, '5': 2, '10': 'resolution'},
    {'1': 'width', '3': 3, '4': 1, '5': 13, '10': 'width'},
    {'1': 'height', '3': 4, '4': 1, '5': 13, '10': 'height'},
    {
      '1': 'origin',
      '3': 5,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Pose',
      '10': 'origin'
    },
  ],
};

/// Descriptor for `MapMetaData`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List mapMetaDataDescriptor = $convert.base64Decode(
    'CgtNYXBNZXRhRGF0YRI8Cg1tYXBfbG9hZF90aW1lGAEgASgLMhgucm9zX2d1aV9iYWNrZW5kLn'
    'BiLlRpbWVSC21hcExvYWRUaW1lEh4KCnJlc29sdXRpb24YAiABKAJSCnJlc29sdXRpb24SFAoF'
    'd2lkdGgYAyABKA1SBXdpZHRoEhYKBmhlaWdodBgEIAEoDVIGaGVpZ2h0EjAKBm9yaWdpbhgFIA'
    'EoCzIYLnJvc19ndWlfYmFja2VuZC5wYi5Qb3NlUgZvcmlnaW4=');

@$core.Deprecated('Use occupancyGridProtoDescriptor instead')
const OccupancyGridProto$json = {
  '1': 'OccupancyGridProto',
  '2': [
    {
      '1': 'header',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Header',
      '10': 'header'
    },
    {
      '1': 'info',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.MapMetaData',
      '10': 'info'
    },
    {'1': 'frame_id', '3': 3, '4': 1, '5': 9, '10': 'frameId'},
    {'1': 'data', '3': 4, '4': 3, '5': 5, '10': 'data'},
  ],
};

/// Descriptor for `OccupancyGridProto`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List occupancyGridProtoDescriptor = $convert.base64Decode(
    'ChJPY2N1cGFuY3lHcmlkUHJvdG8SMgoGaGVhZGVyGAEgASgLMhoucm9zX2d1aV9iYWNrZW5kLn'
    'BiLkhlYWRlclIGaGVhZGVyEjMKBGluZm8YAiABKAsyHy5yb3NfZ3VpX2JhY2tlbmQucGIuTWFw'
    'TWV0YURhdGFSBGluZm8SGQoIZnJhbWVfaWQYAyABKAlSB2ZyYW1lSWQSEgoEZGF0YRgEIAMoBV'
    'IEZGF0YQ==');

@$core.Deprecated('Use pathDescriptor instead')
const Path$json = {
  '1': 'Path',
  '2': [
    {
      '1': 'header',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Header',
      '10': 'header'
    },
    {
      '1': 'poses',
      '3': 2,
      '4': 3,
      '5': 11,
      '6': '.ros_gui_backend.pb.PoseStamped',
      '10': 'poses'
    },
  ],
};

/// Descriptor for `Path`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List pathDescriptor = $convert.base64Decode(
    'CgRQYXRoEjIKBmhlYWRlchgBIAEoCzIaLnJvc19ndWlfYmFja2VuZC5wYi5IZWFkZXJSBmhlYW'
    'RlchI1CgVwb3NlcxgCIAMoCzIfLnJvc19ndWlfYmFja2VuZC5wYi5Qb3NlU3RhbXBlZFIFcG9z'
    'ZXM=');

@$core.Deprecated('Use odometryDescriptor instead')
const Odometry$json = {
  '1': 'Odometry',
  '2': [
    {
      '1': 'header',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Header',
      '10': 'header'
    },
    {'1': 'child_frame_id', '3': 2, '4': 1, '5': 9, '10': 'childFrameId'},
    {
      '1': 'pose',
      '3': 3,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.PoseWithCovariance',
      '10': 'pose'
    },
    {
      '1': 'twist',
      '3': 4,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Twist',
      '10': 'twist'
    },
  ],
};

/// Descriptor for `Odometry`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List odometryDescriptor = $convert.base64Decode(
    'CghPZG9tZXRyeRIyCgZoZWFkZXIYASABKAsyGi5yb3NfZ3VpX2JhY2tlbmQucGIuSGVhZGVyUg'
    'ZoZWFkZXISJAoOY2hpbGRfZnJhbWVfaWQYAiABKAlSDGNoaWxkRnJhbWVJZBI6CgRwb3NlGAMg'
    'ASgLMiYucm9zX2d1aV9iYWNrZW5kLnBiLlBvc2VXaXRoQ292YXJpYW5jZVIEcG9zZRIvCgV0d2'
    'lzdBgEIAEoCzIZLnJvc19ndWlfYmFja2VuZC5wYi5Ud2lzdFIFdHdpc3Q=');
