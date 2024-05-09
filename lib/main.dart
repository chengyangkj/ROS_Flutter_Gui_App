import 'dart:async';
import 'package:toast/toast.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/page/MapPage.dart';
import 'package:ros_flutter_gui_app/page/RobotConnectPage.dart';
import 'package:ros_flutter_gui_app/channel/ros_channel.dart';
// import 'package:ros_flutter_gui_app/hardware/gamepad.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
        providers: [
          ChangeNotifierProvider<RosChannel>(create: (_) => RosChannel()),
          // ChangeNotifierProvider<JoyStickController>(
          //     create: (_) => JoyStickController()),
        ],
        child: MaterialApp(
          title: 'Ros Flutter GUI App',
          initialRoute: "/connect",
          routes: {
            "/connect": ((context) => RobotConnectionPage()),
            "/map": ((context) => MapPage()),
            // "/gamepad":((context) => GamepadPage()),
          },
          theme: ThemeData(
            primarySwatch: Colors.blue,
          ),
          home: RobotConnectionPage(),
        ));
  }
}
