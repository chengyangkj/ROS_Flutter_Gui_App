import 'package:web_socket_channel/web_socket_channel.dart';

//Implemented in `ros_html.dart` and `ros_io.dart`
Future<WebSocketChannel> initializeWebSocketChannel(String url) async {
  throw UnsupportedError('Cannot create a web socket channel without dart:html or dart:io.');
}
