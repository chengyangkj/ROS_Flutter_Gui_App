import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:dartssh2/dartssh2.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ws_channel.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class _ListIntSink implements StreamSink<List<int>> {
  _ListIntSink(this._sink);
  final WebSocketSink _sink;

  @override
  void add(List<int> data) {
    _sink.add(data is Uint8List ? data : Uint8List.fromList(data));
  }

  @override
  void addError(Object error, [StackTrace? stackTrace]) {
    _sink.addError(error, stackTrace);
  }

  @override
  Future<void> addStream(Stream<List<int>> stream) async {
    await for (final chunk in stream) {
      add(chunk);
    }
  }

  @override
  Future<void> close() => _sink.close();

  @override
  Future<void> get done => _sink.done;
}

class _SshOverWsSocket implements SSHSocket {
  _SshOverWsSocket(this._channel) {
    _sub = _channel.stream.listen(
      _onWire,
      onError: _in.addError,
      onDone: _in.close,
      cancelOnError: false,
    );
  }

  final WebSocketChannel _channel;
  final StreamController<Uint8List> _in = StreamController<Uint8List>.broadcast();
  late final StreamSubscription<dynamic> _sub;

  void _onWire(dynamic data) {
    if (data is Uint8List) {
      _in.add(data);
    } else if (data is List<int>) {
      _in.add(Uint8List.fromList(data));
    } else if (data is String) {
      _in.add(Uint8List.fromList(utf8.encode(data)));
    }
  }

  @override
  Stream<Uint8List> get stream => _in.stream;

  @override
  StreamSink<List<int>> get sink => _ListIntSink(_channel.sink);

  @override
  Future<void> get done => _channel.sink.done;

  @override
  Future<void> close() async {
    await _sub.cancel();
    try {
      await _channel.sink.close();
    } catch (_) {}
    await _in.close();
  }

  @override
  void destroy() {
    _sub.cancel();
    try {
      _channel.sink.close();
    } catch (_) {}
    _in.close();
  }
}

Future<Object?> sshConnect({
  required String username,
  required String password,
}) async {
  final backendHost = globalSetting.robotIp.trim();
  final backendPort = int.tryParse(globalSetting.httpServerPort.trim()) ?? 8080;
  final uri = WsChannel.sshTunnelWebSocketUri(backendHost, backendPort);
  final channel = WebSocketChannel.connect(uri);
  final socket = _SshOverWsSocket(channel);
  final client = SSHClient(
    socket,
    username: username,
    onPasswordRequest: () => password,
    disableHostkeyVerification: true,
  );
  await client.authenticated;
  return client;
}

Future<Object?> sshStartShell(Object? client) {
  final c = client as SSHClient;
  return c.shell(pty: const SSHPtyConfig(type: 'xterm'));
}

Future<String> sshRunRemoteCommand(Object? client, String command) async {
  final c = client as SSHClient;
  final cmd = command.trim();
  if (cmd.isEmpty) return '';
  final r = await c.runWithResult(cmd);
  final out = utf8.decode(r.stdout);
  final err = utf8.decode(r.stderr);
  return out + (err.isEmpty ? '' : err);
}

void sshClientClose(Object? client) {
  if (client is SSHClient) {
    client.close();
  }
}

void sshSessionWriteBytes(Object? session, Uint8List data) {
  (session as SSHSession).write(data);
}

StreamSubscription<Uint8List> sshSessionListenStdout(
  Object? session,
  void Function(Uint8List data) onData,
) {
  return (session as SSHSession).stdout.listen(onData);
}

StreamSubscription<Uint8List> sshSessionListenStderr(
  Object? session,
  void Function(Uint8List data) onData,
) {
  return (session as SSHSession).stderr.listen(onData);
}

void sshSessionClose(Object? session) {
  if (session is SSHSession) {
    session.close();
  }
}
