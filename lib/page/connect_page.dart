import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/them_provider.dart';
import 'package:fluttertoast/fluttertoast.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';

class ConnectPage extends StatefulWidget {
  @override
  _ConnectPageState createState() => _ConnectPageState();
}

class _ConnectPageState extends State<ConnectPage> {
  final TextEditingController _ipController =
      TextEditingController(text: '127.0.0.1');
  final TextEditingController _portController =
      TextEditingController(text: '9090');
  bool _isConnecting = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: FutureBuilder<bool>(
        future: initGlobalSetting(),
        builder: (BuildContext context, AsyncSnapshot<bool> snapshot) {
          if (snapshot.connectionState == ConnectionState.waiting) {
            return const Center(child: CircularProgressIndicator());
          } else if (snapshot.hasError) {
            return Center(child: Text('发生错误：${snapshot.error}'));
          }

          _ipController.text = globalSetting.robotIp;
          _portController.text = globalSetting.robotPort;

          return Container(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [
                  Theme.of(context).colorScheme.primary.withOpacity(0.05),
                  Theme.of(context).colorScheme.primary.withOpacity(0.1),
                ],
              ),
            ),
            child: SafeArea(
              child: Stack(
                children: [
                  // 居中小窗口
                  Center(
                    child: Container(
                      constraints: const BoxConstraints(
                        maxWidth: 400,
                        maxHeight: 500,
                      ),
                      margin: const EdgeInsets.all(24.0),
                      decoration: BoxDecoration(
                        color: Theme.of(context).colorScheme.surface,
                        borderRadius: BorderRadius.circular(20),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black.withOpacity(0.1),
                            blurRadius: 20,
                            offset: const Offset(0, 10),
                          ),
                        ],
                        border: Border.all(
                          color: Theme.of(context).colorScheme.primary.withOpacity(0.2),
                          width: 1.5,
                        ),
                      ),
                      child: Padding(
                        padding: const EdgeInsets.all(24.0),
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            // 顶部图标和标题
                            Icon(
                              Icons.smart_toy_outlined,
                              size: 50,
                              color: Theme.of(context).colorScheme.primary,
                            ),
                            const SizedBox(height: 16),
                            Text(
                              AppLocalizations.of(context)!.connect_robot,
                              style: Theme.of(context).textTheme.headlineSmall?.copyWith(
                                color: Theme.of(context).colorScheme.primary,
                                fontWeight: FontWeight.w600,
                              ),
                              textAlign: TextAlign.center,
                            ),
                            const SizedBox(height: 32),

                            // IP地址和端口输入框
                            Row(
                              children: [
                                Expanded(
                                  flex: 3,
                                  child: TextField(
                                    controller: _ipController,
                                    decoration: InputDecoration(
                                      labelText: AppLocalizations.of(context)!.ip_address,
                                      labelStyle: TextStyle(
                                        color: Theme.of(context).colorScheme.primary,
                                      ),
                                      prefixIcon: Icon(
                                        Icons.computer,
                                        color: Theme.of(context).colorScheme.primary,
                                      ),
                                      border: OutlineInputBorder(
                                        borderRadius: BorderRadius.circular(12),
                                        borderSide: BorderSide(
                                          color: Theme.of(context).colorScheme.primary,
                                        ),
                                      ),
                                      enabledBorder: OutlineInputBorder(
                                        borderRadius: BorderRadius.circular(12),
                                        borderSide: BorderSide(
                                          color: Theme.of(context).colorScheme.primary.withOpacity(0.5),
                                        ),
                                      ),
                                      focusedBorder: OutlineInputBorder(
                                        borderRadius: BorderRadius.circular(12),
                                        borderSide: BorderSide(
                                          color: Theme.of(context).colorScheme.primary,
                                          width: 2,
                                        ),
                                      ),
                                    ),
                                  ),
                                ),
                                const SizedBox(width: 16),
                                Expanded(
                                  flex: 1,
                                  child: TextField(
                                    controller: _portController,
                                    decoration: InputDecoration(
                                      labelText: AppLocalizations.of(context)!.port,
                                      labelStyle: TextStyle(
                                        color: Theme.of(context).colorScheme.primary,
                                      ),
                                      prefixIcon: Icon(
                                        Icons.settings_ethernet,
                                        color: Theme.of(context).colorScheme.primary,
                                      ),
                                      border: OutlineInputBorder(
                                        borderRadius: BorderRadius.circular(12),
                                        borderSide: BorderSide(
                                          color: Theme.of(context).colorScheme.primary,
                                        ),
                                      ),
                                      enabledBorder: OutlineInputBorder(
                                        borderRadius: BorderRadius.circular(12),
                                        borderSide: BorderSide(
                                          color: Theme.of(context).colorScheme.primary.withOpacity(0.5),
                                        ),
                                      ),
                                      focusedBorder: OutlineInputBorder(
                                        borderRadius: BorderRadius.circular(12),
                                        borderSide: BorderSide(
                                          color: Theme.of(context).colorScheme.primary,
                                          width: 2,
                                        ),
                                      ),
                                    ),
                                    keyboardType: TextInputType.number,
                                  ),
                                ),
                              ],
                            ),
                            const SizedBox(height: 24),

                            // 连接按钮
                            SizedBox(
                              width: double.infinity,
                              child: ElevatedButton(
                                onPressed: _isConnecting ? null : _handleConnect,
                                style: ElevatedButton.styleFrom(
                                  padding: const EdgeInsets.symmetric(vertical: 16),
                                  shape: RoundedRectangleBorder(
                                    borderRadius: BorderRadius.circular(12),
                                  ),
                                  backgroundColor: Theme.of(context).colorScheme.primary,
                                  foregroundColor: Theme.of(context).colorScheme.onPrimary,
                                  elevation: 2,
                                ),
                                child: _isConnecting
                                    ? SizedBox(
                                        height: 20,
                                        width: 20,
                                        child: CircularProgressIndicator(
                                          strokeWidth: 2,
                                          color: Theme.of(context).colorScheme.onPrimary,
                                        ),
                                      )
                                    : Text(
                                        AppLocalizations.of(context)!.connect_robot,
                                        style: const TextStyle(
                                          fontSize: 16,
                                          fontWeight: FontWeight.w600,
                                        ),
                                      ),
                              ),
                            ),
                            const SizedBox(height: 24),

                            // 主题切换
                            Container(
                              decoration: BoxDecoration(
                                color: Theme.of(context).colorScheme.surface.withOpacity(0.1),
                                borderRadius: BorderRadius.circular(12),
                                border: Border.all(
                                  color: Theme.of(context).colorScheme.primary.withOpacity(0.2),
                                ),
                              ),
                              child: Padding(
                                padding: const EdgeInsets.all(8.0),
                                child: Row(
                                  mainAxisAlignment: MainAxisAlignment.center,
                                  children: [
                                    Icon(
                                      Icons.palette_outlined,
                                      size: 18,
                                      color: Theme.of(context).colorScheme.primary,
                                    ),
                                    const SizedBox(width: 8),
                                    Expanded(
                                      child: SegmentedButton<ThemeMode>(
                                        segments: [
                                          ButtonSegment(
                                            value: ThemeMode.system,
                                            icon: Icon(Icons.brightness_auto, size: 18),
                                            label: Text(
                                              AppLocalizations.of(context)!.auto,
                                              style: const TextStyle(fontSize: 12),
                                            ),
                                          ),
                                          ButtonSegment(
                                            value: ThemeMode.light,
                                            icon: Icon(Icons.light_mode, size: 18),
                                            label: Text(
                                              AppLocalizations.of(context)!.light,
                                              style: const TextStyle(fontSize: 12),
                                            ),
                                          ),
                                          ButtonSegment(
                                            value: ThemeMode.dark,
                                            icon: Icon(Icons.dark_mode, size: 18),
                                            label: Text(
                                              AppLocalizations.of(context)!.dark,
                                              style: const TextStyle(fontSize: 12),
                                            ),
                                          ),
                                        ],
                                        selected: {
                                          Provider.of<ThemeProvider>(context).themeMode
                                        },
                                        onSelectionChanged: (Set<ThemeMode> selection) {
                                          Provider.of<ThemeProvider>(context, listen: false)
                                              .updateThemeMode(selection.first.index);
                                        },
                                        style: ButtonStyle(
                                          side: MaterialStateProperty.all(BorderSide(
                                            color: Theme.of(context).colorScheme.primary.withOpacity(0.5),
                                          )),
                                        ),
                                      ),
                                    ),
                                  ],
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ),

                  // 右上角设置按钮
                  Positioned(
                    top: 8,
                    right: 8,
                    child: IconButton(
                      onPressed: () => Navigator.pushNamed(context, "/setting"),
                      icon: Icon(
                        Icons.settings,
                        color: Theme.of(context).colorScheme.primary,
                        size: 24,
                      ),
                      style: IconButton.styleFrom(
                        backgroundColor: Theme.of(context).colorScheme.surface.withOpacity(0.9),
                        padding: const EdgeInsets.all(8),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                          side: BorderSide(
                            color: Theme.of(context).colorScheme.primary.withOpacity(0.2),
                            width: 1.5,
                          ),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          );
        },
      ),
    );
  }

  Future<void> _handleConnect() async {
    setState(() => _isConnecting = true);
    try {
      final String ip = _ipController.text;
      final int port = int.tryParse(_portController.text) ?? 9090;

      final provider = Provider.of<RosChannel>(context, listen: false);
      globalSetting.setRobotIp(ip);
      globalSetting.setRobotPort(_portController.text);

      final error = await provider.connect("ws://$ip:$port");
      if (error.isEmpty) {
        Navigator.pushNamed(context, "/map");
      } else {
        Fluttertoast.showToast(
          msg: "connect to ROS failed: $error",
          toastLength: Toast.LENGTH_LONG,
          gravity: ToastGravity.BOTTOM,
        );
      }
    } finally {
      setState(() => _isConnecting = false);
    }
  }
}
