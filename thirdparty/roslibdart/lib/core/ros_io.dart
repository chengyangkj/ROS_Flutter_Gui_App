import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/io.dart';
import 'dart:io';

WebSocketChannel initializeWebSocketChannel(String url) {
  try {
    return IOWebSocketChannel.connect(url);
  } on SocketException catch (e) {
    print('SocketException in initializeWebSocketChannel: $e');
    rethrow; // 重新抛出异常，让上层处理
  } catch (e) {
    print('Unknown error in initializeWebSocketChannel: $e');
    rethrow; // 重新抛出异常，让上层处理
  }
}
