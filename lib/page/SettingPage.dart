import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';

class SettingsPage extends StatefulWidget {
  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  final Map<String, TextEditingController> _controllers = {};

  @override
  void initState() {
    super.initState();
    _loadSettings();
  }

  void _loadSettings() async {
    final prefs = await SharedPreferences.getInstance();
    final keys = prefs.getKeys();
    final Map<String, TextEditingController> newControllers = {};
    for (String key in keys) {
      final dynamic value = prefs.get(key); // 获取任何类型的值
      String stringValue;
      if (value is String) {
        stringValue = value;
      } else if (value is int) {
        stringValue = value.toString();
      } else if (value is double) {
        stringValue = value.toString();
      } else if (value is bool) {
        stringValue = value ? 'true' : 'false';
      } else {
        stringValue = 'Unsupported type';
      }
      newControllers[key] = TextEditingController(text: stringValue);
    }
    setState(() {
      _controllers.addAll(newControllers);
    });
  }

  void _saveSettings(String key, String value) async {
    final prefs = await SharedPreferences.getInstance();
    // 这里仅作为示例，实际使用时应确保保存的类型与读取的类型一致
    if (key == "init"){
      Setting globalSetting = Setting();
      await prefs.setString(key, value);
      await initGlobalSetting();
      _loadSettings();
    }
    else{
      await prefs.setString(key, value);
    }
  }

  @override
  void dispose() {
    _controllers.forEach((key, controller) {
      controller.dispose();
    });
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Settings"),
      ),
      body: ListView(
        children: _controllers.entries.map((entry) {
          return ListTile(
            title: TextField(
              controller: entry.value,
              decoration: InputDecoration(labelText: entry.key),
              onChanged: (value) => _saveSettings(entry.key, value),
            ),
          );
        }).toList(),
      ),
    );
  }
}
