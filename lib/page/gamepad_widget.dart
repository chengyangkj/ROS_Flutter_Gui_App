import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:gamepads/gamepads.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

class GamepadWidget extends StatefulWidget {
  GamepadWidget();

  @override
  _GamepadWidgetState createState() => _GamepadWidgetState();
}

class _GamepadWidgetState extends State<GamepadWidget> {
  JoystickController leftJoystickController = JoystickController();
  JoystickController rightJoystickController = JoystickController();

  StreamSubscription<GamepadEvent>? _subscription;
  ValueNotifier<String> eventString = ValueNotifier("");

  double joystickWidgetSize = 120;

  Map<KeyName, JoyStickEvent> eventMap = {};

  @override
  void initState() {
    super.initState();

    _subscription = Gamepads.events.listen((event) {
      String key = event.key;
      double value = event.value;
      eventString.value = event.toString();
      if (event.type == KeyType.button) {
        if (globalSetting.buttonMapping.containsKey(key)) {
          var mappingEvent = globalSetting.buttonMapping[key]!;

          bool trigger = (value != 0);
          if (trigger) {
            mappingEvent.value =
                globalSetting.buttonMapping[key]!.reverse ? 0 : 1;
          } else {
            mappingEvent.value =
                globalSetting.buttonMapping[key]!.reverse ? 1 : 0;
          }

          eventMap[mappingEvent.keyName] = mappingEvent;
        } else {
          print(
              'current button not mapping! event key: ${event.key},value:${event.value}type:${event.type}');
        }
      } else {
        if (key == "AXIS_X") {
          JoyStickEvent event = JoyStickEvent(KeyName.leftAxisX);
          event.value = value;
          eventMap[KeyName.leftAxisX] = event;
        } else if (key == "AXIS_Y") {
          JoyStickEvent event = JoyStickEvent(KeyName.leftAxisY);
          event.value = value;
          eventMap[KeyName.leftAxisY] = event;
        } else if (key == "AXIS_Z") {
          JoyStickEvent event = JoyStickEvent(KeyName.rightAxisX);
          event.value = value;
          eventMap[KeyName.rightAxisX] = event;
        } else if (key == "AXIS_RZ") {
          JoyStickEvent event = JoyStickEvent(KeyName.rightAxisY);
          event.value = value;
          eventMap[KeyName.rightAxisY] = event;
        }
      }

      leftJoystickController.start(Offset.zero);
      rightJoystickController.start(Offset.zero);

      if (eventMap.containsKey(KeyName.leftAxisY)) {
        leftJoystickController.update(Offset(
            0, -eventMap[KeyName.leftAxisY]!.value * joystickWidgetSize));
      }

      if (eventMap.containsKey(KeyName.rightAxisX) &&
          eventMap.containsKey(KeyName.rightAxisY)) {
        rightJoystickController.update(Offset(
            eventMap[KeyName.rightAxisX]!.value * joystickWidgetSize,
            -eventMap[KeyName.rightAxisY]!.value * joystickWidgetSize));
      }
    });
  }

  @override
  void dispose() {
    super.dispose();
    // leftJoystickController.end();
    // rightJoystickController.end();
    _subscription?.cancel();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;
    return Visibility(
        visible:
            Provider.of<GlobalState>(context, listen: false).isManualCtrl.value,
        child: Stack(
          children: [
            Positioned(
              left: 30,
              bottom: 10,
              child: Opacity(
                opacity: 1,
                child: Container(
                  width: joystickWidgetSize,
                  height: joystickWidgetSize,
                  child: Joystick(
                    controller: leftJoystickController,
                    mode: JoystickMode.all,
                    includeInitialAnimation: false,
                    listener: (details) {
                      double max_vx =
                          double.parse(globalSetting.getConfig('MaxVx'));
                      double vx = max_vx * details.y * -1;
                      Provider.of<RosChannel>(context, listen: false).setVx(vx);

                      double max_vy =
                          double.parse(globalSetting.getConfig('MaxVy'));
                      double vy = max_vy * details.x * -1;
                      Provider.of<RosChannel>(context, listen: false).setVy(vy);
                    },
                  ),
                ),
              ),
            ),

            //右摇杆单制角速度
            Positioned(
              right: 30,
              bottom: 10,
              child: Container(
                  width: joystickWidgetSize,
                  height: joystickWidgetSize,
                  child: Joystick(
                    controller: rightJoystickController,
                    mode: JoystickMode.horizontal,
                    includeInitialAnimation: false,
                    listener: (details) {
                      double max_vw =
                          double.parse(globalSetting.getConfig('MaxVw'));

                      double vw = max_vw * details.x * -1;
                      Provider.of<RosChannel>(context, listen: false).setVw(vw);

                      if (details.x.abs() <= 0.2) {
                        Provider.of<RosChannel>(context, listen: false)
                            .setVw(0);
                      }
                    },
                  )),
            ),
            Positioned(
              bottom: 10,
              right: 10,
              child: Container(
                child: Text("${eventString.value}"),
              ),
            )
          ],
        ));
  }
}
