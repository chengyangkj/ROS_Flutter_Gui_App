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
      '6': '.ros_gui_backend.pb.LaserScanBaseFrame',
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
    'SGVhcnRiZWF0SABSCWhlYXJ0YmVhdBJHCgpsYXNlcl9zY2FuGAQgASgLMiYucm9zX2d1aV9iYW'
    'NrZW5kLnBiLkxhc2VyU2NhbkJhc2VGcmFtZUgAUglsYXNlclNjYW4SRwoOcm9ib3RfcG9zZV9t'
    'YXAYBSABKAsyHy5yb3NfZ3VpX2JhY2tlbmQucGIuUG9zZVN0YW1wZWRIAFIMcm9ib3RQb3NlTW'
    'FwEjkKCnBhdGhfbG9jYWwYBiABKAsyGC5yb3NfZ3VpX2JhY2tlbmQucGIuUGF0aEgAUglwYXRo'
    'TG9jYWwSOwoLcGF0aF9nbG9iYWwYByABKAsyGC5yb3NfZ3VpX2JhY2tlbmQucGIuUGF0aEgAUg'
    'pwYXRoR2xvYmFsEjkKCnBhdGhfdHJhY2UYCCABKAsyGC5yb3NfZ3VpX2JhY2tlbmQucGIuUGF0'
    'aEgAUglwYXRoVHJhY2USOgoIb2RvbWV0cnkYCSABKAsyHC5yb3NfZ3VpX2JhY2tlbmQucGIuT2'
    'RvbWV0cnlIAFIIb2RvbWV0cnkSPAoHYmF0dGVyeRgKIAEoCzIgLnJvc19ndWlfYmFja2VuZC5w'
    'Yi5CYXR0ZXJ5U3RhdGVIAFIHYmF0dGVyeRJCCglmb290cHJpbnQYCyABKAsyIi5yb3NfZ3VpX2'
    'JhY2tlbmQucGIuUG9seWdvblN0YW1wZWRIAFIJZm9vdHByaW50Ek0KDWxvY2FsX2Nvc3RtYXAY'
    'DCABKAsyJi5yb3NfZ3VpX2JhY2tlbmQucGIuT2NjdXBhbmN5R3JpZFByb3RvSABSDGxvY2FsQ2'
    '9zdG1hcBJPCg5nbG9iYWxfY29zdG1hcBgNIAEoCzImLnJvc19ndWlfYmFja2VuZC5wYi5PY2N1'
    'cGFuY3lHcmlkUHJvdG9IAFINZ2xvYmFsQ29zdG1hcBJQCg5wb2ludGNsb3VkX21hcBgOIAEoCz'
    'InLnJvc19ndWlfYmFja2VuZC5wYi5Qb2ludENsb3VkMk1hcEZyYW1lSABSDXBvaW50Y2xvdWRN'
    'YXASRQoKZGlhZ25vc3RpYxgPIAEoCzIjLnJvc19ndWlfYmFja2VuZC5wYi5EaWFnbm9zdGljQX'
    'JyYXlIAFIKZGlhZ25vc3RpYxJECgpuYXZfc3RhdHVzGBEgASgLMiMucm9zX2d1aV9iYWNrZW5k'
    'LnBiLkdvYWxTdGF0dXNBcnJheUgAUgluYXZTdGF0dXMSaQoZdHJhbnNmb3JtX2xvb2t1cF9yZX'
    'Nwb25zZRgSIAEoCzIrLnJvc19ndWlfYmFja2VuZC5wYi5UcmFuc2Zvcm1Mb29rdXBSZXNwb25z'
    'ZUgAUhd0cmFuc2Zvcm1Mb29rdXBSZXNwb25zZUIJCgdwYXlsb2FkSgQIAhAD');

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
