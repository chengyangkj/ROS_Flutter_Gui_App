// Copyright (c) 2019 Conrad Heidebrecht.

import 'dart:async';
import 'ros.dart';
import 'request.dart';

// Receiver function to handle requests when the service is advertising.
typedef ServiceHandler = Future<Map<String, dynamic>>? Function(
    Map<String, dynamic> args);

/// Wrapper to interact with ROS services.
class Service {
  Service({
    required this.ros,
    required this.name,
    required this.type,
  });

  /// The ROS connection.
  Ros ros;

  /// Name of the service.
  String name;

  /// Type of the service.
  String type;

  /// Advertiser that is listened to for service requests when advertising.
  Stream<Map<String, dynamic>>? _advertiser;

  /// Checks whether or not the service is currently advertising.
  bool get isAdvertised => _advertiser != null;

  StreamSubscription? listener;

  /// Call the service with a request ([req]).
  Future call(dynamic req) {
    // The service can't be called if it's currently advertising.
    if (isAdvertised) return Future.value(false);
    // Set up the response receiver by filtering data from the ROS node by the ID generated.
    final callId = ros.requestServiceCaller(name);
    final receiver = ros.stream.where((message) => message['id'] == callId).map(
        (Map<String, dynamic> message) => message['result'] == null
            ? Future.error(message['values']!)
            : Future.value(message['values']));
    // Wait for the receiver to receive a single response and then return.
    final completer = Completer();

    listener = receiver.listen((d) {
      completer.complete(d);
      listener!.cancel();
    });
    // Actually send the request.
    ros.send(Request(
      op: 'call_service',
      id: callId,
      service: name,
      type: type,
      args: req,
    ));
    return completer.future;
  }

  // Advertise the service and provide a [handler] to deal with requests.
  Future<void> advertise(ServiceHandler handler) async {
    if (isAdvertised) return;
    // Send the advertise request.
    ros.send(Request(
      op: 'advertise_service',
      type: type,
      service: name,
    ));
    // Listen for requests, forward them to the handler and then
    // send the response back to the ROS node.
    _advertiser = ros.stream;
    _advertiser!.listen((Map<String, dynamic> message) async {
      if (message['service'] != name) {
        return;
      }
      Map<String, dynamic>? resp = await handler(message['args']);
      ros.send(Request(
        op: 'service_response',
        id: message['id'],
        service: name,
        values: resp ?? {},
        result: resp != null,
      ));
    });

    /*
    _advertiser = ros.stream
        .where((message) => message['service'] == name)
        .asyncMap((req) => handler(req['args'])!.then((resp) {
              ros.send(Request(
                op: 'service_response',
                id: req['id'],
                service: name,
                values: resp ?? {},
                result: resp != null,
              ));
            }));
            */
  }

  // Stop advertising the service.
  void unadvertise() {
    if (!isAdvertised) return;
    ros.send(Request(
      op: 'unadvertise_service',
      service: name,
    ));
    _advertiser = null;
  }
}
