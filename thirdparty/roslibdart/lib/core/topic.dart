// Copyright (c) 2019 Conrad Heidebrecht.

import 'dart:async';
import 'ros.dart';
import 'request.dart';

// Receiver function to handle requests when the service is advertising.
typedef SubscribeHandler = Future<void> Function(Map<String, dynamic> args);

/// Wrapper to interact with ROS topics.
class Topic {
  Topic({
    required this.ros,
    required this.name,
    required this.type,
    this.compression = 'none',
    this.throttleRate = 0,
    this.latch = false,
    this.queueSize = 100,
    this.queueLength = 0,
    this.reconnectOnClose = true,
  })  : assert(['png', 'cbor-raw','cbor', 'none'].contains(compression)),
        assert(throttleRate >= 0);

  /// The ROS connection.
  Ros ros;

  /// Stream subscribers to the topic can listen to.
  Stream<Map<String, dynamic>>? subscription;

  /// Name of the topic.
  String name;

  /// Message type the topic uses.
  String type;

  /// Subscription ID provided by [ros].
  String? subscribeId;

  /// Advertiser ID provided by [ros].
  String? advertiseId;

  /// Checks whether or not the topic is currently advertising.
  bool get isAdvertised => advertiseId != null;

  /// Publisher ID provided by [ros].
  late String publishId;

  /// The type of compression to use, like 'png' or 'cbor'.
  ///
  /// Defaults to 'none'.
  String compression;

  /// The rate (in ms between messages) at which to throttle the topic.
  ///
  /// Defaults to 0.
  int throttleRate;

  /// Latch the topic when publishing.
  ///
  /// Defaults to false.
  bool latch;

  /// The queue created at the bridge side for republishing topics.
  ///
  /// Defaults to 100.
  int queueSize;

  /// The queue length at the bridge side used when subscribing.
  ///
  /// Defaults to 0 (no queueing).
  int queueLength;

  /// Flag to enable resubscription and readvertisement on a ROS connection close event.
  ///
  /// Defaults to true.
  bool reconnectOnClose;
  /*
  /// Subscribe to the topic if not already subscribed.
  Future<void> subscribe() async {
    if (subscribeId == null) {
      // Create the listenable broadcast subscription stream.
      subscription = ros.stream.where((message) => message['topic'] == name);
      subscribeId = ros.requestSubscriber(name);
      // Send the subscribe request to ROS.
      await safeSend(Request(
        op: 'subscribe',
        id: subscribeId,
        type: type,
        topic: name,
        compression: compression,
        throttleRate: throttleRate,
        queueLength: queueLength,
      ));
    }
  }
  */

  Future<void> subscribe(SubscribeHandler subscribeHandler) async {
    if (subscribeId == null) {
      // Create the listenable broadcast subscription stream.
      subscription = ros.stream;
      subscribeId = ros.requestSubscriber(name);
      await safeSend(Request(
        op: 'subscribe',
        id: subscribeId,
        type: type,
        topic: name,
        compression: compression,
        throttleRate: throttleRate,
        queueLength: queueLength,
      ));
      subscription!.listen((Map<String, dynamic> message) async {
        if (message['topic'] != name) {
          return;
        }
        await subscribeHandler(message['msg']);
      });
    }
  }

  /// Unsubscribe from the topic.
  Future<void> unsubscribe() async {
    if (subscribeId != null) {
      // Send the request and reset the subscription variables.
      await safeSend(Request(
        op: 'unsubscribe',
        id: subscribeId,
        topic: name,
      ));
      // await ros.requestUnsubscribe(id);
      subscription = null;
      subscribeId = null;
    }
  }

  /// Publish a [message] to the topic.
  Future<void> publish(dynamic message) async {
    // Advertise the topic and then send the publish request.
    await advertise();
    publishId = ros.requestPublisher(name);
    await safeSend(Request(
      op: 'publish',
      topic: name,
      id: publishId,
      msg: message,
      latch: latch,
    ));
  }

  /// Advertise the topic.
  Future<void> advertise() async {
    if (!isAdvertised) {
      // Send the advertisement request.
      advertiseId = ros.requestAdvertiser(name);
      await safeSend(Request(
        op: 'advertise',
        id: advertiseId,
        type: type,
        topic: name,
        latch: latch,
        queueSize: queueSize,
      ));
      // If the ROS connection closes show that we're not advertising anymore.
      watchForClose();
    }
  }

  /// Wait for the connection to close and then reset advertising variables.
  Future<void> watchForClose() async {
    if (!reconnectOnClose) {
      await ros.statusStream.firstWhere((s) => s == Status.closed);
      advertiseId = null;
    }
  }

  /// Stop advertising the topic.
  Future<void> unadvertise() async {
    if (isAdvertised) {
      // Send the unadvertise request and reset variables.
      await safeSend(Request(
        op: 'unadvertise',
        id: advertiseId,
        topic: name,
      ));
      advertiseId = null;
    }
  }

  /// Safely send a [message] to ROS.
  Future<void> safeSend(Request message) async {
    // Send the message but if we're not connected and the [reconnectOnClose] flag
    // is set, wait for ROS to reconnect and then resend the [message].
    ros.send(message);
    if (reconnectOnClose && ros.status != Status.connected) {
      await ros.statusStream.firstWhere((s) => s == Status.connected);
      ros.send(message);
    }
  }
}
