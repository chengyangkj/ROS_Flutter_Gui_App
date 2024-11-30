import 'dart:async';
import 'package:flutter/services.dart';
import 'package:gamepads/gamepads.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:toast/toast.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/page/map_page.dart';
import 'package:ros_flutter_gui_app/page/robot_connect_page.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/page/setting_page.dart';
import 'package:wakelock_plus/wakelock_plus.dart';
import 'provider/them_provider.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await _setInitialOrientation();
  runApp(MultiProvider(providers: [
    ChangeNotifierProvider<RosChannel>(create: (_) => RosChannel()),
    ChangeNotifierProvider<ThemeProvider>(create: (_) => ThemeProvider()),
    ChangeNotifierProvider<GlobalState>(create: (_) => GlobalState())
  ], child: MyApp()));
}

Future<void> _setInitialOrientation() async {
  final prefs = await SharedPreferences.getInstance();
  final orientationValue = prefs.getString('screenOrientation') ?? 'landscape';
  if (orientationValue == 'portrait') {
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
  } else {
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }
}

class MyApp extends StatefulWidget {
  @override
  _MyAppState createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  ThemeMode _themeMode = ThemeMode.system;

  @override
  void initState() {
    super.initState();
    // 设置横屏模式
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    // 关闭系统状态栏的显示
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.manual, overlays: []);
    WakelockPlus.toggle(enable: true);
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Ros Flutter GUI App',
      initialRoute: "/connect",
      routes: {
        "/connect": ((context) => RobotConnectionPage()),
        "/map": ((context) => MapPage()),
        "/setting": ((context) => SettingsPage()),
        // "/gamepad":((context) => GamepadPage()),
      },
      themeMode: Provider.of<ThemeProvider>(context, listen: true).themeMode,
      theme: ThemeData(
        brightness: Brightness.light,
        colorScheme: ColorScheme.fromSwatch().copyWith(
          primary: Colors.blue,
          secondary: Colors.blue[50],
          background: Color.fromRGBO(240, 240, 240, 1),
          surface: Color.fromARGB(153, 224, 224, 224),
        ),
        inputDecorationTheme: const InputDecorationTheme(
          enabledBorder: UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.grey),
          ),
          focusedBorder: UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.blue, width: 2.0),
          ),
        ),
        iconTheme: IconThemeData(
          color: Colors.black, // 设置全局图标颜色为绿色
        ),
        cardColor: Color.fromRGBO(230, 230, 230, 1),
        scaffoldBackgroundColor: Colors.white,
        appBarTheme: AppBarTheme(elevation: 0),
        chipTheme: ThemeData.light().chipTheme.copyWith(
              backgroundColor: Colors.white,
              elevation: 10.0,
              shape: StadiumBorder(
                side: BorderSide(
                  color: Colors.grey[300]!, // 设置边框颜色
                  width: 1.0, // 设置边框宽度
                ),
              ),
            ),
      ),
      darkTheme: ThemeData(
        brightness: Brightness.dark,
        colorScheme:
            ColorScheme.fromSwatch(brightness: Brightness.dark).copyWith(
          primary: Colors.blue,
          secondary: Colors.blueGrey,
          surface: Color.fromRGBO(60, 60, 60, 1),
        ),
        inputDecorationTheme: const InputDecorationTheme(
          enabledBorder: UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.grey),
          ),
          focusedBorder: UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.blue, width: 2.0),
          ),
        ),
        cardColor: Color.fromRGBO(230, 230, 230, 1),
        scaffoldBackgroundColor: Color.fromRGBO(40, 40, 40, 1),
        appBarTheme: AppBarTheme(elevation: 0),
        iconTheme: IconThemeData(
          color: Colors.white, // 设置全局图标颜色为绿色
        ),
        chipTheme: ThemeData.dark().chipTheme.copyWith(
              backgroundColor: Color.fromRGBO(60, 60, 60, 1),
              elevation: 10.0,
              shape: StadiumBorder(
                side: BorderSide(
                  color: Colors.white, // 设置边框颜色
                  width: 1.0, // 设置边框宽度
                ),
              ),
            ),
      ),
      home: RobotConnectionPage(),
    );
  }
}
