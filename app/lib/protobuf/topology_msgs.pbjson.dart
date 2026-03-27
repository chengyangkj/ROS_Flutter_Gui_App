// This is a generated file - do not edit.
//
// Generated from topology_msgs.proto.

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

@$core.Deprecated('Use topologyRouteInfoPbDescriptor instead')
const TopologyRouteInfoPb$json = {
  '1': 'TopologyRouteInfoPb',
  '2': [
    {'1': 'controller', '3': 1, '4': 1, '5': 9, '10': 'controller'},
    {'1': 'goal_checker', '3': 2, '4': 1, '5': 9, '10': 'goalChecker'},
    {'1': 'speed_limit', '3': 3, '4': 1, '5': 2, '10': 'speedLimit'},
  ],
};

/// Descriptor for `TopologyRouteInfoPb`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List topologyRouteInfoPbDescriptor = $convert.base64Decode(
    'ChNUb3BvbG9neVJvdXRlSW5mb1BiEh4KCmNvbnRyb2xsZXIYASABKAlSCmNvbnRyb2xsZXISIQ'
    'oMZ29hbF9jaGVja2VyGAIgASgJUgtnb2FsQ2hlY2tlchIfCgtzcGVlZF9saW1pdBgDIAEoAlIK'
    'c3BlZWRMaW1pdA==');

@$core.Deprecated('Use topologyRoutePbDescriptor instead')
const TopologyRoutePb$json = {
  '1': 'TopologyRoutePb',
  '2': [
    {'1': 'from_point', '3': 1, '4': 1, '5': 9, '10': 'fromPoint'},
    {'1': 'to_point', '3': 2, '4': 1, '5': 9, '10': 'toPoint'},
    {
      '1': 'route_info',
      '3': 3,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.TopologyRouteInfoPb',
      '10': 'routeInfo'
    },
  ],
};

/// Descriptor for `TopologyRoutePb`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List topologyRoutePbDescriptor = $convert.base64Decode(
    'Cg9Ub3BvbG9neVJvdXRlUGISHQoKZnJvbV9wb2ludBgBIAEoCVIJZnJvbVBvaW50EhkKCHRvX3'
    'BvaW50GAIgASgJUgd0b1BvaW50EkYKCnJvdXRlX2luZm8YAyABKAsyJy5yb3NfZ3VpX2JhY2tl'
    'bmQucGIuVG9wb2xvZ3lSb3V0ZUluZm9QYlIJcm91dGVJbmZv');

@$core.Deprecated('Use topologyPointPbDescriptor instead')
const TopologyPointPb$json = {
  '1': 'TopologyPointPb',
  '2': [
    {'1': 'x', '3': 1, '4': 1, '5': 1, '10': 'x'},
    {'1': 'y', '3': 2, '4': 1, '5': 1, '10': 'y'},
    {'1': 'theta', '3': 3, '4': 1, '5': 1, '10': 'theta'},
    {'1': 'name', '3': 4, '4': 1, '5': 9, '10': 'name'},
    {'1': 'type', '3': 5, '4': 1, '5': 9, '10': 'type'},
  ],
};

/// Descriptor for `TopologyPointPb`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List topologyPointPbDescriptor = $convert.base64Decode(
    'Cg9Ub3BvbG9neVBvaW50UGISDAoBeBgBIAEoAVIBeBIMCgF5GAIgASgBUgF5EhQKBXRoZXRhGA'
    'MgASgBUgV0aGV0YRISCgRuYW1lGAQgASgJUgRuYW1lEhIKBHR5cGUYBSABKAlSBHR5cGU=');

@$core.Deprecated('Use topologyMapPropertyPbDescriptor instead')
const TopologyMapPropertyPb$json = {
  '1': 'TopologyMapPropertyPb',
  '2': [
    {
      '1': 'support_controllers',
      '3': 1,
      '4': 3,
      '5': 9,
      '10': 'supportControllers'
    },
    {
      '1': 'support_goal_checkers',
      '3': 2,
      '4': 3,
      '5': 9,
      '10': 'supportGoalCheckers'
    },
  ],
};

/// Descriptor for `TopologyMapPropertyPb`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List topologyMapPropertyPbDescriptor = $convert.base64Decode(
    'ChVUb3BvbG9neU1hcFByb3BlcnR5UGISLwoTc3VwcG9ydF9jb250cm9sbGVycxgBIAMoCVISc3'
    'VwcG9ydENvbnRyb2xsZXJzEjIKFXN1cHBvcnRfZ29hbF9jaGVja2VycxgCIAMoCVITc3VwcG9y'
    'dEdvYWxDaGVja2Vycw==');

@$core.Deprecated('Use topologyMapPbDescriptor instead')
const TopologyMapPb$json = {
  '1': 'TopologyMapPb',
  '2': [
    {'1': 'map_name', '3': 1, '4': 1, '5': 9, '10': 'mapName'},
    {
      '1': 'map_property',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.ros_gui_backend.pb.TopologyMapPropertyPb',
      '10': 'mapProperty'
    },
    {
      '1': 'points',
      '3': 3,
      '4': 3,
      '5': 11,
      '6': '.ros_gui_backend.pb.TopologyPointPb',
      '10': 'points'
    },
    {
      '1': 'routes',
      '3': 4,
      '4': 3,
      '5': 11,
      '6': '.ros_gui_backend.pb.TopologyRoutePb',
      '10': 'routes'
    },
  ],
};

/// Descriptor for `TopologyMapPb`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List topologyMapPbDescriptor = $convert.base64Decode(
    'Cg1Ub3BvbG9neU1hcFBiEhkKCG1hcF9uYW1lGAEgASgJUgdtYXBOYW1lEkwKDG1hcF9wcm9wZX'
    'J0eRgCIAEoCzIpLnJvc19ndWlfYmFja2VuZC5wYi5Ub3BvbG9neU1hcFByb3BlcnR5UGJSC21h'
    'cFByb3BlcnR5EjsKBnBvaW50cxgDIAMoCzIjLnJvc19ndWlfYmFja2VuZC5wYi5Ub3BvbG9neV'
    'BvaW50UGJSBnBvaW50cxI7CgZyb3V0ZXMYBCADKAsyIy5yb3NfZ3VpX2JhY2tlbmQucGIuVG9w'
    'b2xvZ3lSb3V0ZVBiUgZyb3V0ZXM=');
