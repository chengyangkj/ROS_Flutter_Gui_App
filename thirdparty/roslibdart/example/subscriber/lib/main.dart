import 'package:flutter/material.dart';
import 'package:roslibdart/roslibdart.dart';
import 'dart:async';
import 'dart:convert';

void main() {
  runApp(const ExampleApp());
}

class ExampleApp extends StatelessWidget {
  const ExampleApp({Key? key}) : super(key: key);
  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      title: 'Roslibdart subscriber Example',
      home: HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({Key? key}) : super(key: key);
  @override
  _HomePageState createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  late Ros ros;
  late Topic chatter;


  @override
  void initState() {
    ros = Ros(url: 'ws://192.168.111.29:9090');
    chatter = Topic(
        ros: ros,
        name: '/topic',
        type: "std_msgs/String",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);
  
    super.initState();
    ros.connect();

    Timer(const Duration(seconds: 3), () async {
      await chatter.subscribe(subscribeHandler);
      // await chatter.subscribe();

      var topicsAndRawTypes = await ros.getTopicsAndRawTypes();
      print("topicsAndRawTypes:${topicsAndRawTypes}");
      // for(var topic in topicsAndRawTypes['topics']){
      //   print("topic:${topic['name']} type:${topic['type']}");
      // }
    });
  }

  void destroyConnection() async {
    await chatter.unsubscribe();
    await ros.close();
    setState(() {});
  }

  String msgReceived = '';
  Future<void> subscribeHandler(Map<String, dynamic> msg) async {
    msgReceived = json.encode(msg);
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Roslibdart Subscriber Example'),
      ),
      body: Center(
        // Center is a layout widget. It takes a single child and positions it
        // in the middle of the parent.
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text(msgReceived + ' received'),
          ],
        ),
      ),
    );
  }
}
