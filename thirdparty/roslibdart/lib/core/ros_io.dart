import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/io.dart';
import 'dart:io';

Future<WebSocketChannel> initializeWebSocketChannel(String url) async {
  try {
    WebSocket ws = await WebSocket.connect(url);
    return IOWebSocketChannel(ws);
  } on SocketException catch (e) {
    print('SocketException in initializeWebSocketChannel: $e');
    rethrow; // 重新抛出异常，让上层处理
  } catch (e) {
    print('Unknown error in initializeWebSocketChannel: $e');
    rethrow; // 重新抛出异常，让上层处理
  }
}
