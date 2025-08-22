import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

class ThemeProvider extends ChangeNotifier {
  ThemeMode _themeMode = ThemeMode.system;

  ThemeProvider() {
    _loadThemeMode();
  }

  ThemeMode get themeMode => _themeMode;

  void _loadThemeMode() async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    int themeIndex = prefs.getInt('themeMode') ?? 2; // 默认使用暗黑模式 (ThemeMode.dark = 2)
    _themeMode = ThemeMode.values[themeIndex];
    notifyListeners();
  }

  void updateThemeMode(int index) async {
    SharedPreferences prefs = await SharedPreferences.getInstance();
    await prefs.setInt('themeMode', index);
    _themeMode = ThemeMode.values[index];
    notifyListeners();
  }
}
