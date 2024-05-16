import 'package:easy_loading_button/easy_loading_button.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:toast/toast.dart';

class RobotConnectionPage extends StatefulWidget {
  @override
  _RobotConnectionPageState createState() => _RobotConnectionPageState();
}

class _RobotConnectionPageState extends State<RobotConnectionPage> {
  TextEditingController _ipController =
      TextEditingController(text: '127.0.0.1');
  TextEditingController _portController = TextEditingController(text: '9090');

  @override
  Widget build(BuildContext context) {
    ToastContext().init(context);
    return Scaffold(
      body: FutureBuilder<bool>(
        future: initGlobalSetting(),
        builder: (BuildContext context, AsyncSnapshot<bool> snapshot) {
          if (snapshot.connectionState == ConnectionState.waiting) {
            return CircularProgressIndicator();
          } else if (snapshot.hasError) {
            return Text('发生错误：${snapshot.error}');
          } else {
            _ipController.text = globalSetting.robotIp;
            _portController.text = globalSetting.robotPort;
            return Padding(
              padding: EdgeInsets.all(16.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  TextField(
                    controller: _ipController,
                    decoration: InputDecoration(labelText: 'Robot IP'),
                  ),
                  SizedBox(height: 16.0),
                  TextField(
                    controller: _portController,
                    decoration: InputDecoration(labelText: 'Robot Port'),
                    keyboardType: TextInputType.number,
                  ),
                  SizedBox(height: 16.0),
                  EasyButton(
                      type: EasyButtonType.text,
                      idleStateWidget: const Text(
                        'connect',
                        style: TextStyle(
                          color: Colors.blue,
                        ),
                      ),
                      loadingStateWidget: const CircularProgressIndicator(
                        strokeWidth: 3.0,
                        valueColor: AlwaysStoppedAnimation<Color>(
                          Colors.green,
                        ),
                      ),
                      useWidthAnimation: true,
                      useEqualLoadingStateWidgetDimension: true,
                      onPressed: () async {
                        // 连接机器人的逻辑
                        String ip = _ipController.text;
                        int port = int.tryParse(_portController.text) ?? 9090;
                        // 在这里执行连接机器人的操作，可以使用ip和port变量
                        print('Connecting to robot at $ip:$port');
                        var provider =
                            Provider.of<RosChannel>(context, listen: false);
                        globalSetting.setRobotIp(ip);
                        globalSetting.setRobotPort(_portController.text);
                        provider.connect("ws://$ip:$port").then((success) {
                          if (success) {
                            // 连接成功后，可以跳转到下一个页面
                            Navigator.pushNamed(context, "/map");
                          } else {
                            print('连接失败');
                            Toast.show("connect to ros failed!",
                                duration: Toast.lengthShort,
                                gravity: Toast.bottom);
                          }
                        });
                      }),
                ],
              ),
            );
          }
        },
      ),
    );
  }
}
