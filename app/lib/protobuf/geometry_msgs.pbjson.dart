// This is a generated file - do not edit.
//
// Generated from geometry_msgs.proto.

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

@$core.Deprecated('Use timeDescriptor instead')
const Time$json = {
  '1': 'Time',
  '2': [
    {'1': 'sec', '3': 1, '4': 1, '5': 5, '10': 'sec'},
    {'1': 'nanosec', '3': 2, '4': 1, '5': 13, '10': 'nanosec'},
  ],
};

/// Descriptor for `Time`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List timeDescriptor = $convert.base64Decode(
    'CgRUaW1lEhAKA3NlYxgBIAEoBVIDc2VjEhgKB25hbm9zZWMYAiABKA1SB25hbm9zZWM=');

@$core.Deprecated('Use headerDescriptor instead')
const Header$json = {
  '1': 'Header',
  '2': [
    {
      '1': 'stamp',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Time',
      '10': 'stamp'
    },
    {'1': 'frame_id', '3': 2, '4': 1, '5': 9, '10': 'frameId'},
  ],
};

/// Descriptor for `Header`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List headerDescriptor = $convert.base64Decode(
    'CgZIZWFkZXISLgoFc3RhbXAYASABKAsyGC5yb3NfZ3VpX2JhY2tlbmQucGIuVGltZVIFc3RhbX'
    'ASGQoIZnJhbWVfaWQYAiABKAlSB2ZyYW1lSWQ=');

@$core.Deprecated('Use pointDescriptor instead')
const Point$json = {
  '1': 'Point',
  '2': [
    {'1': 'x', '3': 1, '4': 1, '5': 1, '10': 'x'},
    {'1': 'y', '3': 2, '4': 1, '5': 1, '10': 'y'},
    {'1': 'z', '3': 3, '4': 1, '5': 1, '10': 'z'},
  ],
};

/// Descriptor for `Point`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List pointDescriptor = $convert.base64Decode(
    'CgVQb2ludBIMCgF4GAEgASgBUgF4EgwKAXkYAiABKAFSAXkSDAoBehgDIAEoAVIBeg==');

@$core.Deprecated('Use quaternionDescriptor instead')
const Quaternion$json = {
  '1': 'Quaternion',
  '2': [
    {'1': 'x', '3': 1, '4': 1, '5': 1, '10': 'x'},
    {'1': 'y', '3': 2, '4': 1, '5': 1, '10': 'y'},
    {'1': 'z', '3': 3, '4': 1, '5': 1, '10': 'z'},
    {'1': 'w', '3': 4, '4': 1, '5': 1, '10': 'w'},
  ],
};

/// Descriptor for `Quaternion`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List quaternionDescriptor = $convert.base64Decode(
    'CgpRdWF0ZXJuaW9uEgwKAXgYASABKAFSAXgSDAoBeRgCIAEoAVIBeRIMCgF6GAMgASgBUgF6Eg'
    'wKAXcYBCABKAFSAXc=');

@$core.Deprecated('Use vector3Descriptor instead')
const Vector3$json = {
  '1': 'Vector3',
  '2': [
    {'1': 'x', '3': 1, '4': 1, '5': 1, '10': 'x'},
    {'1': 'y', '3': 2, '4': 1, '5': 1, '10': 'y'},
    {'1': 'z', '3': 3, '4': 1, '5': 1, '10': 'z'},
  ],
};

/// Descriptor for `Vector3`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List vector3Descriptor = $convert.base64Decode(
    'CgdWZWN0b3IzEgwKAXgYASABKAFSAXgSDAoBeRgCIAEoAVIBeRIMCgF6GAMgASgBUgF6');

@$core.Deprecated('Use poseDescriptor instead')
const Pose$json = {
  '1': 'Pose',
  '2': [
    {
      '1': 'position',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Point',
      '10': 'position'
    },
    {
      '1': 'orientation',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Quaternion',
      '10': 'orientation'
    },
    {'1': 'yaw', '3': 3, '4': 1, '5': 1, '9': 0, '10': 'yaw', '17': true},
    {'1': 'pitch', '3': 4, '4': 1, '5': 1, '9': 1, '10': 'pitch', '17': true},
    {'1': 'roll', '3': 5, '4': 1, '5': 1, '9': 2, '10': 'roll', '17': true},
  ],
  '8': [
    {'1': '_yaw'},
    {'1': '_pitch'},
    {'1': '_roll'},
  ],
};

/// Descriptor for `Pose`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List poseDescriptor = $convert.base64Decode(
    'CgRQb3NlEjUKCHBvc2l0aW9uGAEgASgLMhkucm9zX2d1aV9iYWNrZW5kLnBiLlBvaW50Ughwb3'
    'NpdGlvbhJACgtvcmllbnRhdGlvbhgCIAEoCzIeLnJvc19ndWlfYmFja2VuZC5wYi5RdWF0ZXJu'
    'aW9uUgtvcmllbnRhdGlvbhIVCgN5YXcYAyABKAFIAFIDeWF3iAEBEhkKBXBpdGNoGAQgASgBSA'
    'FSBXBpdGNoiAEBEhcKBHJvbGwYBSABKAFIAlIEcm9sbIgBAUIGCgRfeWF3QggKBl9waXRjaEIH'
    'CgVfcm9sbA==');

@$core.Deprecated('Use twistDescriptor instead')
const Twist$json = {
  '1': 'Twist',
  '2': [
    {
      '1': 'linear',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Vector3',
      '10': 'linear'
    },
    {
      '1': 'angular',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Vector3',
      '10': 'angular'
    },
  ],
};

/// Descriptor for `Twist`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List twistDescriptor = $convert.base64Decode(
    'CgVUd2lzdBIzCgZsaW5lYXIYASABKAsyGy5yb3NfZ3VpX2JhY2tlbmQucGIuVmVjdG9yM1IGbG'
    'luZWFyEjUKB2FuZ3VsYXIYAiABKAsyGy5yb3NfZ3VpX2JhY2tlbmQucGIuVmVjdG9yM1IHYW5n'
    'dWxhcg==');

@$core.Deprecated('Use poseStampedDescriptor instead')
const PoseStamped$json = {
  '1': 'PoseStamped',
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
      '1': 'pose',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Pose',
      '10': 'pose'
    },
  ],
};

/// Descriptor for `PoseStamped`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List poseStampedDescriptor = $convert.base64Decode(
    'CgtQb3NlU3RhbXBlZBIyCgZoZWFkZXIYASABKAsyGi5yb3NfZ3VpX2JhY2tlbmQucGIuSGVhZG'
    'VyUgZoZWFkZXISLAoEcG9zZRgCIAEoCzIYLnJvc19ndWlfYmFja2VuZC5wYi5Qb3NlUgRwb3Nl');

@$core.Deprecated('Use poseWithCovarianceDescriptor instead')
const PoseWithCovariance$json = {
  '1': 'PoseWithCovariance',
  '2': [
    {
      '1': 'pose',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Pose',
      '10': 'pose'
    },
    {'1': 'covariance', '3': 2, '4': 3, '5': 1, '10': 'covariance'},
  ],
};

/// Descriptor for `PoseWithCovariance`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List poseWithCovarianceDescriptor = $convert.base64Decode(
    'ChJQb3NlV2l0aENvdmFyaWFuY2USLAoEcG9zZRgBIAEoCzIYLnJvc19ndWlfYmFja2VuZC5wYi'
    '5Qb3NlUgRwb3NlEh4KCmNvdmFyaWFuY2UYAiADKAFSCmNvdmFyaWFuY2U=');

@$core.Deprecated('Use poseWithCovarianceStampedDescriptor instead')
const PoseWithCovarianceStamped$json = {
  '1': 'PoseWithCovarianceStamped',
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
      '1': 'pose',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.PoseWithCovariance',
      '10': 'pose'
    },
  ],
};

/// Descriptor for `PoseWithCovarianceStamped`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List poseWithCovarianceStampedDescriptor = $convert.base64Decode(
    'ChlQb3NlV2l0aENvdmFyaWFuY2VTdGFtcGVkEjIKBmhlYWRlchgBIAEoCzIaLnJvc19ndWlfYm'
    'Fja2VuZC5wYi5IZWFkZXJSBmhlYWRlchI6CgRwb3NlGAIgASgLMiYucm9zX2d1aV9iYWNrZW5k'
    'LnBiLlBvc2VXaXRoQ292YXJpYW5jZVIEcG9zZQ==');

@$core.Deprecated('Use polygonDescriptor instead')
const Polygon$json = {
  '1': 'Polygon',
  '2': [
    {
      '1': 'points',
      '3': 1,
      '4': 3,
      '5': 11,
      '6': '.ros_gui_backend.pb.Point',
      '10': 'points'
    },
  ],
};

/// Descriptor for `Polygon`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List polygonDescriptor = $convert.base64Decode(
    'CgdQb2x5Z29uEjEKBnBvaW50cxgBIAMoCzIZLnJvc19ndWlfYmFja2VuZC5wYi5Qb2ludFIGcG'
    '9pbnRz');

@$core.Deprecated('Use polygonStampedDescriptor instead')
const PolygonStamped$json = {
  '1': 'PolygonStamped',
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
      '1': 'polygon',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Polygon',
      '10': 'polygon'
    },
  ],
};

/// Descriptor for `PolygonStamped`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List polygonStampedDescriptor = $convert.base64Decode(
    'Cg5Qb2x5Z29uU3RhbXBlZBIyCgZoZWFkZXIYASABKAsyGi5yb3NfZ3VpX2JhY2tlbmQucGIuSG'
    'VhZGVyUgZoZWFkZXISNQoHcG9seWdvbhgCIAEoCzIbLnJvc19ndWlfYmFja2VuZC5wYi5Qb2x5'
    'Z29uUgdwb2x5Z29u');

@$core.Deprecated('Use transformDescriptor instead')
const Transform$json = {
  '1': 'Transform',
  '2': [
    {
      '1': 'translation',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Vector3',
      '10': 'translation'
    },
    {
      '1': 'rotation',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Quaternion',
      '10': 'rotation'
    },
  ],
};

/// Descriptor for `Transform`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List transformDescriptor = $convert.base64Decode(
    'CglUcmFuc2Zvcm0SPQoLdHJhbnNsYXRpb24YASABKAsyGy5yb3NfZ3VpX2JhY2tlbmQucGIuVm'
    'VjdG9yM1ILdHJhbnNsYXRpb24SOgoIcm90YXRpb24YAiABKAsyHi5yb3NfZ3VpX2JhY2tlbmQu'
    'cGIuUXVhdGVybmlvblIIcm90YXRpb24=');

@$core.Deprecated('Use transformStampedDescriptor instead')
const TransformStamped$json = {
  '1': 'TransformStamped',
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
      '1': 'transform',
      '3': 3,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Transform',
      '10': 'transform'
    },
  ],
};

/// Descriptor for `TransformStamped`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List transformStampedDescriptor = $convert.base64Decode(
    'ChBUcmFuc2Zvcm1TdGFtcGVkEjIKBmhlYWRlchgBIAEoCzIaLnJvc19ndWlfYmFja2VuZC5wYi'
    '5IZWFkZXJSBmhlYWRlchIkCg5jaGlsZF9mcmFtZV9pZBgCIAEoCVIMY2hpbGRGcmFtZUlkEjsK'
    'CXRyYW5zZm9ybRgDIAEoCzIdLnJvc19ndWlfYmFja2VuZC5wYi5UcmFuc2Zvcm1SCXRyYW5zZm'
    '9ybQ==');
