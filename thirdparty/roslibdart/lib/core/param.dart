// Copyright (c) 2019 Conrad Heidebrecht.

import 'dart:async';
import 'ros.dart';
import 'service.dart';

/// A wrapper for a ROS parameter.
class Param {
  Param({required this.ros, required this.name});

  /// The ROS connection.
  Ros ros;

  /// Name of the parameter.
  String name;

  /// Get the parameter from the ROS node using the /rosapi/get_param service.
  Future get() {
    final client = Service(
      ros: ros,
      name: '/rosapi/get_param',
      type: 'rosapi/GetParam',
    );
    return client.call({'name': name});
  }

  /// Set the [value] of the parameter.
  Future set(dynamic value) {
    final client = Service(
      ros: ros,
      name: '/rosapi/set_param',
      type: 'rosapi/SetParam',
    );
    return client.call({
      'name': name,
      'value': value,
    });
  }

  /// Delete the parameter.
  Future delete() {
    final client = Service(
      ros: ros,
      name: '/rosapi/delete_param',
      type: 'rosapi/DeleteParam',
    );
    return client.call({'name': name});
  }
}
