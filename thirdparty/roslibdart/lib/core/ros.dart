// Copyright (c) 2019 Conrad Heidebrecht.

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:cbor/cbor.dart';
import 'service.dart';


// ignore: uri_does_not_exist
// ignore: unused_import
import 'ros_stub.dart'
    // ignore: uri_does_not_exist
    if (dart.library.html) 'ros_html.dart'
    // ignore: uri_does_not_exist
    if (dart.library.io) 'ros_io.dart';

import 'request.dart';

/// Helper function to convert CBOR data to Map
Map<String, dynamic> _cborToMap(dynamic cborValue) {
  if (cborValue is Map) {
    Map<String, dynamic> result = {};
    cborValue.forEach((key, value) {
      String stringKey = key.toString();
      if (value is Map) {
        result[stringKey] = _cborToMap(value);
      } else if (value is List) {
        result[stringKey] = _cborToList(value);
      } else {
        result[stringKey] = value.toString();
      }
    });
    return result;
  }
  return {};
}

/// Helper function to convert CBOR list to List
List<dynamic> _cborToList(dynamic cborValue) {
  if (cborValue is List) {
    return cborValue.map((item) {
      if (item is Map) {
        return _cborToMap(item);
      } else if (item is List) {
        return _cborToList(item);
      } else {
        return item.toString();
      }
    }).toList();
  }
  return [];
}

/// Status enums.
enum Status { none, connecting, connected, closed, errored }
enum TopicStatus {
  subscribed,
  unsubscribed,
  publisher,
  advertised,
  unadvertised,
}

/// The class through which all data to and from a ROS node goes through.
/// Manages status and key information about the connection and node.
class Ros {
  /// Initializes the [_statusController] as a broadcast.
  /// The [url] of the ROS node can be optionally specified at this point.
  Ros({this.url}) {
    _statusController = StreamController<Status>.broadcast();
  }

  /// The url of ROS node running the rosbridge server.
  dynamic url;

  /// Total subscribers to ever connect.
  int subscribers = 0;

  /// Total number of advertisers to ever connect.
  int advertisers = 0;

  /// Total number of publishers to ever connect.
  int publishers = 0;

  /// Total number of callers to ever call a service.
  int serviceCallers = 0;

  /// The sum to generate IDs with.
  int get ids => subscribers + advertisers + publishers + serviceCallers;

  /// The websocket connection to communicate with the ROS node.
  late WebSocketChannel _channel;

  /// Subscription to the websocket stream.
  late StreamSubscription _channelListener;

  /// JSON broadcast websocket stream.
  late Stream<Map<String, dynamic>> stream;

  /// The controller to update subscribers on the state of the connection.
  late StreamController<Status> _statusController;

  /// Subscribable stream to listen for connection status changes.
  Stream<Status> get statusStream => _statusController.stream;

  /// Status variable that can be used when not interested in getting live updates.
  Status status = Status.none;

  /// Connect to the ROS node, the [url] can override what was provided in the constructor.
  Future<String> connect({dynamic url}) async {
    this.url = url ?? this.url;
    url ??= this.url;
    try {
      // Initialize the connection to the ROS node with a Websocket channel.
      _channel = await initializeWebSocketChannel(url);

      stream =
          _channel.stream.asBroadcastStream().map((raw) {
            //judge is binary or json data
             if(raw is Uint8List){
                 //cbor uncompressed
                Map<String, dynamic> jsonData = _cborToMap(cbor.decode(raw));
               return jsonData;
             }else{
              return json.decode(raw);
             }
          });
      // Update the connection status.
      status = Status.connected;
      _statusController.add(status);
      // Listen for messages on the connection to update the status.
      _channelListener = stream.listen((data) {
        //这里永远不会进来，因为连接后没订阅话题不会有任何数据发送过来
        // print('INCOMING: $data');
        if (status != Status.connected) {
          status = Status.connected;
          _statusController.add(status);
        }
      },onError: (error) {
        print('Stream error in ros.connect: $error');
        status = Status.errored;
        _statusController.add(status);
      }, onDone: () {
        print('Stream done in ros.connect');
        status = Status.closed;
        _statusController.add(status);
      }, cancelOnError: false);

      return "";

    } on WebSocketChannelException catch (e) {
      status = Status.errored;
      _statusController.add(status);
      return "$e";
    } on SocketException catch (e) {
      status = Status.errored;
      _statusController.add(status);
      return "$e";
    } catch (e) {
      status = Status.errored;
      _statusController.add(status);
      return "$e";
    }
  }

  /// Close the connection to the ROS node, an exit [code] and [reason] can
  /// be optionally specified.
  Future<void> close([int? code, String? reason]) async {
    /// Close listener and websocket.
    await _channelListener.cancel();
    await _channel.sink.close(code, reason);

    /// Update the connection status.
    _statusController.add(Status.closed);
    status = Status.closed;
  }

  /// Send a [message] to the ROS node
  bool send(dynamic message) {
    // If we're not connected give up.
    if (status != Status.connected) return false;
    // Format the message into JSON and then stringify.
    final toSend = (message is Request)
        ? json.encode(message.toJson())
        : (message is Map || message is List)
            ? json.encode(message)
            : message;
    //print('OUTGOING: $toSend');
    // Actually send it to the node.
    _channel.sink.add(toSend);
    return true;
  }

  void authenticate({
    required String mac,
    required String client,
    required String dest,
    required String rand,
    required DateTime t,
    required String level,
    required DateTime end,
  }) async {
    send({
      'mac': mac,
      'client': client,
      'dest': dest,
      'rand': rand,
      't': t.millisecondsSinceEpoch,
      'level': level,
      'end': end.millisecondsSinceEpoch,
    });
  }

  /// Sends a set_level request to the server.
  /// [level] can be one of {none, error, warning, info}, and
  /// [id] is the optional operation ID to change status level on
  void setStatusLevel({String ?level, int ?id}) {
    send({
      'op': 'set_level',
      'level': level,
      'id': id,
    });
  }

  /// Request a subscription ID.
  String requestSubscriber(String name) {
    subscribers++;
    return 'subscribe:' + name + ':' + ids.toString();
  }

  /// Request an advertiser ID.
  String requestAdvertiser(String name) {
    advertisers++;
    return 'advertise:' + name + ':' + ids.toString();
  }

  /// Request a publisher ID.
  String requestPublisher(String name) {
    publishers++;
    return 'publish:' + name + ':' + ids.toString();
  }

  /// Request a service caller ID.
  String requestServiceCaller(String name) {
    serviceCallers++;
    return 'call_service:' + name + ':' + ids.toString();
  }
  @override
  bool operator ==(other) {
    return other.hashCode == hashCode;
  }

  Future<Map<String, dynamic>> getTopicsAndRawTypes() async {
    var service = Service(ros: this, name: 'rosapi/topics_and_raw_types', type: 'rosapi/TopicsAndRawTypes');
    Map<String, dynamic> json={};
    Map<String, dynamic> result =await service.call(json);
    return result;
    
  }

  @override
  int get hashCode =>
      url.hashCode +
      subscribers.hashCode +
      advertisers.hashCode +
      publishers.hashCode +
      _channel.hashCode +
      _channelListener.hashCode +
      stream.hashCode +
      _statusController.hashCode +
      status.hashCode;
}
