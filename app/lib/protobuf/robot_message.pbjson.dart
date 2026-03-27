// This is a generated file - do not edit.
//
// Generated from robot_message.proto.

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

@$core.Deprecated('Use imageFrameDescriptor instead')
const ImageFrame$json = {
  '1': 'ImageFrame',
  '2': [
    {'1': 'encoding', '3': 1, '4': 1, '5': 9, '10': 'encoding'},
    {'1': 'width', '3': 2, '4': 1, '5': 13, '10': 'width'},
    {'1': 'height', '3': 3, '4': 1, '5': 13, '10': 'height'},
    {'1': 'step', '3': 4, '4': 1, '5': 13, '10': 'step'},
    {'1': 'data', '3': 5, '4': 1, '5': 12, '10': 'data'},
    {'1': 'stamp_sec', '3': 6, '4': 1, '5': 3, '10': 'stampSec'},
    {'1': 'stamp_nanosec', '3': 7, '4': 1, '5': 13, '10': 'stampNanosec'},
    {'1': 'frame_id', '3': 8, '4': 1, '5': 9, '10': 'frameId'},
    {'1': 'topic', '3': 9, '4': 1, '5': 9, '10': 'topic'},
  ],
};

/// Descriptor for `ImageFrame`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List imageFrameDescriptor = $convert.base64Decode(
    'CgpJbWFnZUZyYW1lEhoKCGVuY29kaW5nGAEgASgJUghlbmNvZGluZxIUCgV3aWR0aBgCIAEoDV'
    'IFd2lkdGgSFgoGaGVpZ2h0GAMgASgNUgZoZWlnaHQSEgoEc3RlcBgEIAEoDVIEc3RlcBISCgRk'
    'YXRhGAUgASgMUgRkYXRhEhsKCXN0YW1wX3NlYxgGIAEoA1IIc3RhbXBTZWMSIwoNc3RhbXBfbm'
    'Fub3NlYxgHIAEoDVIMc3RhbXBOYW5vc2VjEhkKCGZyYW1lX2lkGAggASgJUgdmcmFtZUlkEhQK'
    'BXRvcGljGAkgASgJUgV0b3BpYw==');

@$core.Deprecated('Use heartbeatDescriptor instead')
const Heartbeat$json = {
  '1': 'Heartbeat',
  '2': [
    {'1': 'server_time_ms', '3': 1, '4': 1, '5': 3, '10': 'serverTimeMs'},
    {'1': 'seq', '3': 2, '4': 1, '5': 13, '10': 'seq'},
  ],
};

/// Descriptor for `Heartbeat`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List heartbeatDescriptor = $convert.base64Decode(
    'CglIZWFydGJlYXQSJAoOc2VydmVyX3RpbWVfbXMYASABKANSDHNlcnZlclRpbWVNcxIQCgNzZX'
    'EYAiABKA1SA3NlcQ==');

@$core.Deprecated('Use controlSubscribeImageDescriptor instead')
const ControlSubscribeImage$json = {
  '1': 'ControlSubscribeImage',
  '2': [
    {'1': 'topic', '3': 1, '4': 1, '5': 9, '10': 'topic'},
    {'1': 'subscribe', '3': 2, '4': 1, '5': 8, '10': 'subscribe'},
  ],
};

/// Descriptor for `ControlSubscribeImage`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List controlSubscribeImageDescriptor = $convert.base64Decode(
    'ChVDb250cm9sU3Vic2NyaWJlSW1hZ2USFAoFdG9waWMYASABKAlSBXRvcGljEhwKCXN1YnNjcm'
    'liZRgCIAEoCFIJc3Vic2NyaWJl');

@$core.Deprecated('Use cancelNavDescriptor instead')
const CancelNav$json = {
  '1': 'CancelNav',
};

/// Descriptor for `CancelNav`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List cancelNavDescriptor =
    $convert.base64Decode('CglDYW5jZWxOYXY=');

@$core.Deprecated('Use transformLookupRequestDescriptor instead')
const TransformLookupRequest$json = {
  '1': 'TransformLookupRequest',
  '2': [
    {'1': 'target_frame', '3': 1, '4': 1, '5': 9, '10': 'targetFrame'},
    {'1': 'source_frame', '3': 2, '4': 1, '5': 9, '10': 'sourceFrame'},
    {'1': 'seq', '3': 3, '4': 1, '5': 13, '10': 'seq'},
  ],
};

/// Descriptor for `TransformLookupRequest`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List transformLookupRequestDescriptor = $convert.base64Decode(
    'ChZUcmFuc2Zvcm1Mb29rdXBSZXF1ZXN0EiEKDHRhcmdldF9mcmFtZRgBIAEoCVILdGFyZ2V0Rn'
    'JhbWUSIQoMc291cmNlX2ZyYW1lGAIgASgJUgtzb3VyY2VGcmFtZRIQCgNzZXEYAyABKA1SA3Nl'
    'cQ==');

@$core.Deprecated('Use transformLookupResponseDescriptor instead')
const TransformLookupResponse$json = {
  '1': 'TransformLookupResponse',
  '2': [
    {'1': 'seq', '3': 1, '4': 1, '5': 13, '10': 'seq'},
    {'1': 'ok', '3': 2, '4': 1, '5': 8, '10': 'ok'},
    {'1': 'error', '3': 3, '4': 1, '5': 9, '10': 'error'},
    {
      '1': 'transform',
      '3': 4,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.TransformStamped',
      '10': 'transform'
    },
  ],
};

/// Descriptor for `TransformLookupResponse`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List transformLookupResponseDescriptor = $convert.base64Decode(
    'ChdUcmFuc2Zvcm1Mb29rdXBSZXNwb25zZRIQCgNzZXEYASABKA1SA3NlcRIOCgJvaxgCIAEoCF'
    'ICb2sSFAoFZXJyb3IYAyABKAlSBWVycm9yEkIKCXRyYW5zZm9ybRgEIAEoCzIkLnJvc19ndWlf'
    'YmFja2VuZC5wYi5UcmFuc2Zvcm1TdGFtcGVkUgl0cmFuc2Zvcm0=');

@$core.Deprecated('Use robotMessageDescriptor instead')
const RobotMessage$json = {
  '1': 'RobotMessage',
  '2': [
    {
      '1': 'image',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.ImageFrame',
      '9': 0,
      '10': 'image'
    },
    {
      '1': 'heartbeat',
      '3': 3,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Heartbeat',
      '9': 0,
      '10': 'heartbeat'
    },
    {
      '1': 'laser_scan',
      '3': 4,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.LaserScanBaseLink',
      '9': 0,
      '10': 'laserScan'
    },
    {
      '1': 'robot_pose_map',
      '3': 5,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.PoseStamped',
      '9': 0,
      '10': 'robotPoseMap'
    },
    {
      '1': 'path_local',
      '3': 6,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Path',
      '9': 0,
      '10': 'pathLocal'
    },
    {
      '1': 'path_global',
      '3': 7,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Path',
      '9': 0,
      '10': 'pathGlobal'
    },
    {
      '1': 'path_trace',
      '3': 8,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Path',
      '9': 0,
      '10': 'pathTrace'
    },
    {
      '1': 'odometry',
      '3': 9,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Odometry',
      '9': 0,
      '10': 'odometry'
    },
    {
      '1': 'battery',
      '3': 10,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.BatteryState',
      '9': 0,
      '10': 'battery'
    },
    {
      '1': 'footprint',
      '3': 11,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.PolygonStamped',
      '9': 0,
      '10': 'footprint'
    },
    {
      '1': 'local_costmap',
      '3': 12,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.OccupancyGridProto',
      '9': 0,
      '10': 'localCostmap'
    },
    {
      '1': 'global_costmap',
      '3': 13,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.OccupancyGridProto',
      '9': 0,
      '10': 'globalCostmap'
    },
    {
      '1': 'pointcloud_map',
      '3': 14,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.PointCloud2MapFrame',
      '9': 0,
      '10': 'pointcloudMap'
    },
    {
      '1': 'diagnostic',
      '3': 15,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.DiagnosticArray',
      '9': 0,
      '10': 'diagnostic'
    },
    {
      '1': 'nav_status',
      '3': 17,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.GoalStatusArray',
      '9': 0,
      '10': 'navStatus'
    },
    {
      '1': 'transform_lookup_response',
      '3': 18,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.TransformLookupResponse',
      '9': 0,
      '10': 'transformLookupResponse'
    },
  ],
  '8': [
    {'1': 'payload'},
  ],
  '9': [
    {'1': 2, '2': 3},
  ],
};

/// Descriptor for `RobotMessage`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List robotMessageDescriptor = $convert.base64Decode(
    'CgxSb2JvdE1lc3NhZ2USNgoFaW1hZ2UYASABKAsyHi5yb3NfZ3VpX2JhY2tlbmQucGIuSW1hZ2'
    'VGcmFtZUgAUgVpbWFnZRI9CgloZWFydGJlYXQYAyABKAsyHS5yb3NfZ3VpX2JhY2tlbmQucGIu'
    'SGVhcnRiZWF0SABSCWhlYXJ0YmVhdBJGCgpsYXNlcl9zY2FuGAQgASgLMiUucm9zX2d1aV9iYW'
    'NrZW5kLnBiLkxhc2VyU2NhbkJhc2VMaW5rSABSCWxhc2VyU2NhbhJHCg5yb2JvdF9wb3NlX21h'
    'cBgFIAEoCzIfLnJvc19ndWlfYmFja2VuZC5wYi5Qb3NlU3RhbXBlZEgAUgxyb2JvdFBvc2VNYX'
    'ASOQoKcGF0aF9sb2NhbBgGIAEoCzIYLnJvc19ndWlfYmFja2VuZC5wYi5QYXRoSABSCXBhdGhM'
    'b2NhbBI7CgtwYXRoX2dsb2JhbBgHIAEoCzIYLnJvc19ndWlfYmFja2VuZC5wYi5QYXRoSABSCn'
    'BhdGhHbG9iYWwSOQoKcGF0aF90cmFjZRgIIAEoCzIYLnJvc19ndWlfYmFja2VuZC5wYi5QYXRo'
    'SABSCXBhdGhUcmFjZRI6CghvZG9tZXRyeRgJIAEoCzIcLnJvc19ndWlfYmFja2VuZC5wYi5PZG'
    '9tZXRyeUgAUghvZG9tZXRyeRI8CgdiYXR0ZXJ5GAogASgLMiAucm9zX2d1aV9iYWNrZW5kLnBi'
    'LkJhdHRlcnlTdGF0ZUgAUgdiYXR0ZXJ5EkIKCWZvb3RwcmludBgLIAEoCzIiLnJvc19ndWlfYm'
    'Fja2VuZC5wYi5Qb2x5Z29uU3RhbXBlZEgAUglmb290cHJpbnQSTQoNbG9jYWxfY29zdG1hcBgM'
    'IAEoCzImLnJvc19ndWlfYmFja2VuZC5wYi5PY2N1cGFuY3lHcmlkUHJvdG9IAFIMbG9jYWxDb3'
    'N0bWFwEk8KDmdsb2JhbF9jb3N0bWFwGA0gASgLMiYucm9zX2d1aV9iYWNrZW5kLnBiLk9jY3Vw'
    'YW5jeUdyaWRQcm90b0gAUg1nbG9iYWxDb3N0bWFwElAKDnBvaW50Y2xvdWRfbWFwGA4gASgLMi'
    'cucm9zX2d1aV9iYWNrZW5kLnBiLlBvaW50Q2xvdWQyTWFwRnJhbWVIAFINcG9pbnRjbG91ZE1h'
    'cBJFCgpkaWFnbm9zdGljGA8gASgLMiMucm9zX2d1aV9iYWNrZW5kLnBiLkRpYWdub3N0aWNBcn'
    'JheUgAUgpkaWFnbm9zdGljEkQKCm5hdl9zdGF0dXMYESABKAsyIy5yb3NfZ3VpX2JhY2tlbmQu'
    'cGIuR29hbFN0YXR1c0FycmF5SABSCW5hdlN0YXR1cxJpChl0cmFuc2Zvcm1fbG9va3VwX3Jlc3'
    'BvbnNlGBIgASgLMisucm9zX2d1aV9iYWNrZW5kLnBiLlRyYW5zZm9ybUxvb2t1cFJlc3BvbnNl'
    'SABSF3RyYW5zZm9ybUxvb2t1cFJlc3BvbnNlQgkKB3BheWxvYWRKBAgCEAM=');

@$core.Deprecated('Use clientRobotMessageDescriptor instead')
const ClientRobotMessage$json = {
  '1': 'ClientRobotMessage',
  '2': [
    {
      '1': 'control_subscribe_image',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.ControlSubscribeImage',
      '9': 0,
      '10': 'controlSubscribeImage'
    },
    {
      '1': 'cmd_vel',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Twist',
      '9': 0,
      '10': 'cmdVel'
    },
    {
      '1': 'nav_goal',
      '3': 3,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.PoseStamped',
      '9': 0,
      '10': 'navGoal'
    },
    {
      '1': 'cancel_nav',
      '3': 4,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.CancelNav',
      '9': 0,
      '10': 'cancelNav'
    },
    {
      '1': 'reloc',
      '3': 5,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.PoseWithCovarianceStamped',
      '9': 0,
      '10': 'reloc'
    },
    {
      '1': 'transform_lookup_request',
      '3': 7,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.TransformLookupRequest',
      '9': 0,
      '10': 'transformLookupRequest'
    },
  ],
  '8': [
    {'1': 'payload'},
  ],
};

/// Descriptor for `ClientRobotMessage`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List clientRobotMessageDescriptor = $convert.base64Decode(
    'ChJDbGllbnRSb2JvdE1lc3NhZ2USYwoXY29udHJvbF9zdWJzY3JpYmVfaW1hZ2UYASABKAsyKS'
    '5yb3NfZ3VpX2JhY2tlbmQucGIuQ29udHJvbFN1YnNjcmliZUltYWdlSABSFWNvbnRyb2xTdWJz'
    'Y3JpYmVJbWFnZRI0CgdjbWRfdmVsGAIgASgLMhkucm9zX2d1aV9iYWNrZW5kLnBiLlR3aXN0SA'
    'BSBmNtZFZlbBI8CghuYXZfZ29hbBgDIAEoCzIfLnJvc19ndWlfYmFja2VuZC5wYi5Qb3NlU3Rh'
    'bXBlZEgAUgduYXZHb2FsEj4KCmNhbmNlbF9uYXYYBCABKAsyHS5yb3NfZ3VpX2JhY2tlbmQucG'
    'IuQ2FuY2VsTmF2SABSCWNhbmNlbE5hdhJFCgVyZWxvYxgFIAEoCzItLnJvc19ndWlfYmFja2Vu'
    'ZC5wYi5Qb3NlV2l0aENvdmFyaWFuY2VTdGFtcGVkSABSBXJlbG9jEmYKGHRyYW5zZm9ybV9sb2'
    '9rdXBfcmVxdWVzdBgHIAEoCzIqLnJvc19ndWlfYmFja2VuZC5wYi5UcmFuc2Zvcm1Mb29rdXBS'
    'ZXF1ZXN0SABSFnRyYW5zZm9ybUxvb2t1cFJlcXVlc3RCCQoHcGF5bG9hZA==');
