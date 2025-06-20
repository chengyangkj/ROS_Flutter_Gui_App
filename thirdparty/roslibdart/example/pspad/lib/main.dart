import 'package:flutter/material.dart';
import 'pad.dart';
import 'package:roslibdart/roslibdart.dart';
import 'dart:async';
import 'dart:convert';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    String title = 'PS Pad';
    return MaterialApp(
      title: title,
      home: RoslibPSPad(title: title),
    );
  }
}

class RoslibPSPad extends StatefulWidget {
  const RoslibPSPad({Key? key, required this.title}) : super(key: key);

  final String title;

  @override
  State<RoslibPSPad> createState() => _RoslibPSPadState();
}

class _RoslibPSPadState extends State<RoslibPSPad> {
  String host = 'ws://127.0.0.1:9090';
  late Ros ros;
  late Topic cmdVelTopic;
  double turnAngularVelocity = 1.5;
  double forwardVelocity = 1.5;

  @override
  void initState() {
    ros = Ros(url: host);
    cmdVelTopic = Topic(
        ros: ros,
        name: '/turtle1/cmd_vel',
        type: "geometry_msgs/msg/Twist",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);

    ros.connect();
    super.initState();
    Timer.periodic(const Duration(milliseconds: 200), directionFired);
    Timer.periodic(const Duration(milliseconds: 200), keyFired);
  }

  int leftButtonState = 1;
  int rightButtonState = 1;
  int forwardButtonState = 1;
  int backwardButtonState = 1;
  int triangleButtonState = 1;
  int squareButtonState = 1;
  int circleButtonState = 1;
  int crossButtonState = 1;

  var lastDirectionTwist = {};
  var lastKeyTwist = {};
  void directionFired(Timer timer) {
    var linear = {'x': 0.0, 'y': 0.0, 'z': 0.0};
    var angular = {'x': 0.0, 'y': 0.0, 'z': 0.0};
    if (leftButtonState == -1) {
      linear = {'x': 0.0, 'y': 0.0, 'z': 0.0};
      angular = {'x': 0.0, 'y': 0.0, 'z': turnAngularVelocity};
    } else if (rightButtonState == -1) {
      linear = {'x': 0.0, 'y': 0.0, 'z': 0.0};
      angular = {'x': 0.0, 'y': 0.0, 'z': -turnAngularVelocity};
    } else if (forwardButtonState == -1) {
      linear = {'x': forwardVelocity, 'y': 0.0, 'z': 0.0};
      angular = {'x': 0.0, 'y': 0.0, 'z': 0};
    } else if (backwardButtonState == -1) {
      linear = {'x': -forwardVelocity, 'y': 0.0, 'z': 0.0};
      angular = {'x': 0.0, 'y': 0.0, 'z': 0};
    }
    var twist = {'linear': linear, 'angular': angular};
    if (lastDirectionTwist == twist &&
        twist['linear'] == {'x': 0.0, 'y': 0.0, 'z': 0.0} &&
        twist['angular'] == {'x': 0.0, 'y': 0.0, 'z': 0.0}) {
      return;
    }
    print('Direction to published ' + jsonEncode(twist));
    cmdVelTopic.publish(twist);
    lastDirectionTwist = twist;
  }

  void keyFired(Timer timer) {
    var keys = {
      'triangle': triangleButtonState,
      'square': squareButtonState,
      'circle': circleButtonState,
      'cross': crossButtonState
    };

    if (lastKeyTwist == keys &&
        keys['triangle'] == 1 &&
        keys['square'] == 1 &&
        keys['circle'] == 1 &&
        keys['cross'] == 1) {
      return;
    }
    print('Key to published ' + jsonEncode(keys));
    lastKeyTwist = keys;
  }

  void leftCallback(int event) {
    leftButtonState = event;
  }

  void rightCallback(int event) {
    rightButtonState = event;
  }

  void forwardCallback(int event) {
    forwardButtonState = event;
  }

  void backwardCallback(int event) {
    backwardButtonState = event;
  }

  void squareCallback(int event) {
    squareButtonState = event;
  }

  void circleCallback(int event) {
    circleButtonState = event;
  }

  void triangleCallback(int event) {
    triangleButtonState = event;
  }

  void crossCallback(int event) {
    crossButtonState = event;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Center(
          // Center is a layout widget. It takes a single child and positions it
          // in the middle of the parent.
          child: Container(
        margin: const EdgeInsets.all(50),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            DirectionPad(
              diameter: 200,
              leftCallback: leftCallback,
              rightCallback: rightCallback,
              forwardCallback: forwardCallback,
              backwardCallback: backwardCallback,
            ),
            KeyPad(
              diameter: 200,
              squareCallback: squareCallback,
              circleCallback: circleCallback,
              triangleCallback: triangleCallback,
              crossCallback: crossCallback,
            ),
          ],
        ),
      )),
    );
  }
}
