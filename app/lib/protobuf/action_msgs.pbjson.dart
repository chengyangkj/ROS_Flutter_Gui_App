// This is a generated file - do not edit.
//
// Generated from action_msgs.proto.

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

@$core.Deprecated('Use goalIDDescriptor instead')
const GoalID$json = {
  '1': 'GoalID',
  '2': [
    {'1': 'uuid', '3': 1, '4': 1, '5': 9, '10': 'uuid'},
  ],
};

/// Descriptor for `GoalID`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List goalIDDescriptor =
    $convert.base64Decode('CgZHb2FsSUQSEgoEdXVpZBgBIAEoCVIEdXVpZA==');

@$core.Deprecated('Use goalInfoDescriptor instead')
const GoalInfo$json = {
  '1': 'GoalInfo',
  '2': [
    {
      '1': 'goal_id',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.GoalID',
      '10': 'goalId'
    },
    {
      '1': 'stamp',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.Time',
      '10': 'stamp'
    },
  ],
};

/// Descriptor for `GoalInfo`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List goalInfoDescriptor = $convert.base64Decode(
    'CghHb2FsSW5mbxIzCgdnb2FsX2lkGAEgASgLMhoucm9zX2d1aV9iYWNrZW5kLnBiLkdvYWxJRF'
    'IGZ29hbElkEi4KBXN0YW1wGAIgASgLMhgucm9zX2d1aV9iYWNrZW5kLnBiLlRpbWVSBXN0YW1w');

@$core.Deprecated('Use goalStatusDescriptor instead')
const GoalStatus$json = {
  '1': 'GoalStatus',
  '2': [
    {
      '1': 'goal_info',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.GoalInfo',
      '10': 'goalInfo'
    },
    {'1': 'status', '3': 2, '4': 1, '5': 13, '10': 'status'},
    {'1': 'text', '3': 3, '4': 1, '5': 9, '10': 'text'},
  ],
};

/// Descriptor for `GoalStatus`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List goalStatusDescriptor = $convert.base64Decode(
    'CgpHb2FsU3RhdHVzEjkKCWdvYWxfaW5mbxgBIAEoCzIcLnJvc19ndWlfYmFja2VuZC5wYi5Hb2'
    'FsSW5mb1IIZ29hbEluZm8SFgoGc3RhdHVzGAIgASgNUgZzdGF0dXMSEgoEdGV4dBgDIAEoCVIE'
    'dGV4dA==');

@$core.Deprecated('Use goalStatusArrayDescriptor instead')
const GoalStatusArray$json = {
  '1': 'GoalStatusArray',
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
      '1': 'status_list',
      '3': 2,
      '4': 3,
      '5': 11,
      '6': '.ros_gui_backend.pb.GoalStatus',
      '10': 'statusList'
    },
  ],
};

/// Descriptor for `GoalStatusArray`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List goalStatusArrayDescriptor = $convert.base64Decode(
    'Cg9Hb2FsU3RhdHVzQXJyYXkSMgoGaGVhZGVyGAEgASgLMhoucm9zX2d1aV9iYWNrZW5kLnBiLk'
    'hlYWRlclIGaGVhZGVyEj8KC3N0YXR1c19saXN0GAIgAygLMh4ucm9zX2d1aV9iYWNrZW5kLnBi'
    'LkdvYWxTdGF0dXNSCnN0YXR1c0xpc3Q=');
