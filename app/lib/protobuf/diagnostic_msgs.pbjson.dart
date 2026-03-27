// This is a generated file - do not edit.
//
// Generated from diagnostic_msgs.proto.

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

@$core.Deprecated('Use keyValueDescriptor instead')
const KeyValue$json = {
  '1': 'KeyValue',
  '2': [
    {'1': 'key', '3': 1, '4': 1, '5': 9, '10': 'key'},
    {'1': 'value', '3': 2, '4': 1, '5': 9, '10': 'value'},
  ],
};

/// Descriptor for `KeyValue`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List keyValueDescriptor = $convert.base64Decode(
    'CghLZXlWYWx1ZRIQCgNrZXkYASABKAlSA2tleRIUCgV2YWx1ZRgCIAEoCVIFdmFsdWU=');

@$core.Deprecated('Use diagnosticStatusDescriptor instead')
const DiagnosticStatus$json = {
  '1': 'DiagnosticStatus',
  '2': [
    {'1': 'level', '3': 1, '4': 1, '5': 5, '10': 'level'},
    {'1': 'name', '3': 2, '4': 1, '5': 9, '10': 'name'},
    {'1': 'message', '3': 3, '4': 1, '5': 9, '10': 'message'},
    {'1': 'hardware_id', '3': 4, '4': 1, '5': 9, '10': 'hardwareId'},
    {
      '1': 'values',
      '3': 5,
      '4': 3,
      '5': 11,
      '6': '.ros_gui_backend.pb.KeyValue',
      '10': 'values'
    },
  ],
};

/// Descriptor for `DiagnosticStatus`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List diagnosticStatusDescriptor = $convert.base64Decode(
    'ChBEaWFnbm9zdGljU3RhdHVzEhQKBWxldmVsGAEgASgFUgVsZXZlbBISCgRuYW1lGAIgASgJUg'
    'RuYW1lEhgKB21lc3NhZ2UYAyABKAlSB21lc3NhZ2USHwoLaGFyZHdhcmVfaWQYBCABKAlSCmhh'
    'cmR3YXJlSWQSNAoGdmFsdWVzGAUgAygLMhwucm9zX2d1aV9iYWNrZW5kLnBiLktleVZhbHVlUg'
    'Z2YWx1ZXM=');

@$core.Deprecated('Use diagnosticArrayDescriptor instead')
const DiagnosticArray$json = {
  '1': 'DiagnosticArray',
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
      '1': 'status',
      '3': 2,
      '4': 3,
      '5': 11,
      '6': '.ros_gui_backend.pb.DiagnosticStatus',
      '10': 'status'
    },
  ],
};

/// Descriptor for `DiagnosticArray`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List diagnosticArrayDescriptor = $convert.base64Decode(
    'Cg9EaWFnbm9zdGljQXJyYXkSMgoGaGVhZGVyGAEgASgLMhoucm9zX2d1aV9iYWNrZW5kLnBiLk'
    'hlYWRlclIGaGVhZGVyEjwKBnN0YXR1cxgCIAMoCzIkLnJvc19ndWlfYmFja2VuZC5wYi5EaWFn'
    'bm9zdGljU3RhdHVzUgZzdGF0dXM=');
