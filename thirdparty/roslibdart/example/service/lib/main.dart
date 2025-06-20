import 'dart:async';

import 'package:flutter/material.dart';
import 'package:roslibdart/roslibdart.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Roslibdart Service Example',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const MyHomePage(title: 'Roslibdart Service Example'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({Key? key, required this.title}) : super(key: key);

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  late Ros ros;
  late Service service;
  int msgToPublished = 0;
  @override
  void initState() {
    ros = Ros(url: 'ws://127.0.0.1:9090');
    service = Service(
        name: 'add_two_ints', ros: ros, type: "tutorial_interfaces/AddTwoInts");

    super.initState();
    ros.connect();
    Timer(const Duration(seconds: 3), () async {
      await service.advertise(serviceHandler);
    });
  }

  void destroyConnection() async {
    await ros.close();
    setState(() {});
  }

  Future<Map<String, dynamic>>? serviceHandler(
      Map<String, dynamic> args) async {
    Map<String, dynamic> response = {};
    response['sum'] = args['a'] + args['b'];
    msgToPublished = response['sum'];
    setState(() {});
    return response;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        // Here we take the value from the MyHomePage object that was created by
        // the App.build method, and use it to set our appbar title.
        title: Text(widget.title),
      ),
      body: Center(
        // Center is a layout widget. It takes a single child and positions it
        // in the middle of the parent.
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text('Service returns answer ' + msgToPublished.toString()),
          ],
        ),
      ),
    );
  }
}
