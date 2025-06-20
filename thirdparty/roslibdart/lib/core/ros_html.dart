import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/html.dart';

Future<WebSocketChannel> initializeWebSocketChannel(String url) async {
  return HtmlWebSocketChannel.connect(url);
}
