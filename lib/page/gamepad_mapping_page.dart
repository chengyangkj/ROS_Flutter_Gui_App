import 'package:flutter/material.dart';
import 'package:gamepads/gamepads.dart';
import 'dart:async';
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import '../global/setting.dart';

class GamepadMappingPage extends StatefulWidget {
  const GamepadMappingPage({super.key});

  @override
  State<GamepadMappingPage> createState() => _GamepadMappingPageState();
}

class _GamepadMappingPageState extends State<GamepadMappingPage> {
  StreamSubscription? _subscription;
  String _currentMapping = '';
  KeyName? _waitingForMapping;
  final Map<KeyName, String> _mappedKeys = {};
  KeyName? _editingKey; // 当前正在编辑的按键

  // 添加保存原始映射的变量
  Map<String, JoyStickEvent> _originalAxisMapping = {};
  Map<String, JoyStickEvent> _originalButtonMapping = {};

  @override
  void initState() {
    super.initState();
    _loadMappings();
    _startListening();
  }

  void _loadMappings() async {
    final prefs = await SharedPreferences.getInstance();
    final mappingStr = prefs.getString('gamepadMapping');
    if (mappingStr != null) {
      try {
        final mapping = jsonDecode(mappingStr);
        setState(() {
          // 加载 axisMapping
          if (mapping['axisMapping'] != null) {
            (mapping['axisMapping'] as Map<String, dynamic>)
                .forEach((key, value) {
              _mappedKeys[_parseKeyName(value['keyName'])] = key;
            });
          }

          // 加载 buttonMapping
          if (mapping['buttonMapping'] != null) {
            (mapping['buttonMapping'] as Map<String, dynamic>)
                .forEach((key, value) {
              _mappedKeys[_parseKeyName(value['keyName'])] = key;
            });
          }
        });
      } catch (e) {
        print('Error loading gamepad mapping: $e');
        // 加载失败时使用默认映射
        _loadDefaultMappings();
      }
    } else {
      // 如果没有保存的映射，使用默认映射
      _loadDefaultMappings();
    }
  }

  void _loadDefaultMappings() {
    setState(() {
      _mappedKeys.clear();
      // 加载默认的轴映射
      axisMapping.forEach((key, value) {
        _mappedKeys[value.keyName] = key;
      });
      // 加载默认的按钮映射
      buttonMapping.forEach((key, value) {
        _mappedKeys[value.keyName] = key;
      });
    });
  }

  KeyName _parseKeyName(String keyNameStr) {
    // 移除 'KeyName.' 前缀
    final enumStr = keyNameStr.replaceAll('KeyName.', '');
    return KeyName.values.firstWhere(
      (e) => e.toString() == 'KeyName.$enumStr',
      orElse: () => KeyName.None,
    );
  }

  void _startListening() {
    _subscription?.cancel(); // 取消之前的订阅
    _subscription = Gamepads.events.listen((event) {
      if (_editingKey != null) {
        setState(() {
          _currentMapping = '${event.key} (${event.value})';

          // 直接更新映射
          if (event.type == KeyType.analog) {
            axisMapping[event.key] = JoyStickEvent(_editingKey!);
          } else if (event.type == KeyType.button) {
            buttonMapping[event.key] = JoyStickEvent(
              _editingKey!,
              maxValue: 1,
              minValue: 0,
              reverse: event.value < 0,
            );
          }
        });
      }
    });
  }

  Widget _buildMappingTable() {
    return SingleChildScrollView(
      scrollDirection: Axis.horizontal,
      child: DataTable(
        columns: const [
          DataColumn(label: Text('按键')),
          DataColumn(label: Text('映射键')),
          DataColumn(label: Text('最大值')),
          DataColumn(label: Text('最小值')),
          DataColumn(label: Text('反转')),
          DataColumn(label: Text('操作')),
        ],
        rows: [
          ...axisMapping.entries
              .map((entry) => _buildAxisDataRow(entry.key, entry.value)),
          ...buttonMapping.entries
              .map((entry) => _buildButtonDataRow(entry.key, entry.value)),
        ],
      ),
    );
  }

  DataRow _buildAxisDataRow(String key, JoyStickEvent event) {
    bool isEditing = _editingKey == event.keyName;
    TextEditingController maxController =
        TextEditingController(text: event.maxValue.toString());
    TextEditingController minController =
        TextEditingController(text: event.minValue.toString());

    return DataRow(
      cells: [
        DataCell(Text(_translateKey(key))),
        DataCell(
          isEditing
              ? Text('等待按键... $_currentMapping')
              : Text(event.keyName.toString()),
        ),
        DataCell(
          isEditing
              ? SizedBox(
                  width: 80,
                  child: TextField(
                    controller: maxController,
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(
                      isDense: true,
                      contentPadding: EdgeInsets.all(8),
                    ),
                  ),
                )
              : Text(event.maxValue.toString()),
        ),
        DataCell(
          isEditing
              ? SizedBox(
                  width: 80,
                  child: TextField(
                    controller: minController,
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(
                      isDense: true,
                      contentPadding: EdgeInsets.all(8),
                    ),
                  ),
                )
              : Text(event.minValue.toString()),
        ),
        DataCell(
          isEditing
              ? Checkbox(
                  value: event.reverse,
                  onChanged: (value) {
                    setState(() {
                      event.reverse = value ?? false;
                    });
                  },
                )
              : Text(event.reverse ? '是' : '否'),
        ),
        DataCell(
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              ElevatedButton(
                onPressed: () {
                  if (isEditing) {
                    // 保存修改
                    setState(() {
                      event.maxValue =
                          int.tryParse(maxController.text) ?? event.maxValue;
                      event.minValue =
                          int.tryParse(minController.text) ?? event.minValue;
                      _editingKey = null;
                      _subscription?.cancel();
                    });
                  } else {
                    // 开始编辑，保存原始值
                    _originalAxisMapping = Map.from(axisMapping);
                    _originalButtonMapping = Map.from(buttonMapping);
                    setState(() {
                      _editingKey = event.keyName;
                      _startListening();
                    });
                  }
                },
                child: Text(isEditing ? '保存' : '重新映射'),
              ),
              if (isEditing) ...[
                const SizedBox(width: 8),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      // 恢复原始值
                      axisMapping.clear();
                      buttonMapping.clear();
                      axisMapping.addAll(_originalAxisMapping);
                      buttonMapping.addAll(_originalButtonMapping);
                      _editingKey = null;
                      _subscription?.cancel();
                    });
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.grey,
                  ),
                  child: const Text('取消'),
                ),
              ],
            ],
          ),
        ),
      ],
    );
  }

  DataRow _buildButtonDataRow(String key, JoyStickEvent event) {
    bool isEditing = _editingKey == event.keyName;

    return DataRow(
      cells: [
        DataCell(Text(_translateKey(key))),
        DataCell(
          isEditing
              ? Text('等待按键... $_currentMapping')
              : Text(event.keyName.toString()),
        ),
        DataCell(Text('1')), // 按钮固定最大值
        DataCell(Text('0')), // 按钮固定最小值
        DataCell(
          isEditing
              ? Checkbox(
                  value: event.reverse,
                  onChanged: (value) {
                    setState(() {
                      event.reverse = value ?? false;
                    });
                  },
                )
              : Text(event.reverse ? '是' : '否'),
        ),
        DataCell(
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              ElevatedButton(
                onPressed: () {
                  if (isEditing) {
                    // 保存修改
                    setState(() {
                      _editingKey = null;
                      _subscription?.cancel();
                    });
                  } else {
                    // 开始编辑，保存原始值
                    _originalAxisMapping = Map.from(axisMapping);
                    _originalButtonMapping = Map.from(buttonMapping);
                    setState(() {
                      _editingKey = event.keyName;
                      _startListening();
                    });
                  }
                },
                child: Text(isEditing ? '保存' : '重新映射'),
              ),
              if (isEditing) ...[
                const SizedBox(width: 8),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      // 恢复原始值
                      axisMapping.clear();
                      buttonMapping.clear();
                      axisMapping.addAll(_originalAxisMapping);
                      buttonMapping.addAll(_originalButtonMapping);
                      _editingKey = null;
                      _subscription?.cancel();
                    });
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.grey,
                  ),
                  child: const Text('取消'),
                ),
              ],
            ],
          ),
        ),
      ],
    );
  }

  String _translateKey(String key) {
    const translations = {
      "AXIS_X": "左摇杆 X",
      "AXIS_Y": "左摇杆 Y",
      "AXIS_Z": "右摇杆 X",
      "AXIS_RZ": "右摇杆 Y",
      "triggerRight": "右扳机",
      "triggerLeft": "左扳机",
      "buttonLeftRight": "方向键 左/右",
      "buttonUpDown": "方向键 上/下",
      "KEYCODE_BUTTON_A": "按钮 A",
      "KEYCODE_BUTTON_B": "按钮 B",
      "KEYCODE_BUTTON_X": "按钮 X",
      "KEYCODE_BUTTON_Y": "按钮 Y",
      "KEYCODE_BUTTON_L1": "按钮 L1",
      "KEYCODE_BUTTON_R1": "按钮 R1",
    };
    return translations[key] ?? key;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('手柄按键映射'),
        actions: [
          IconButton(
            icon: const Icon(Icons.save),
            onPressed: _saveMapping,
          ),
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: () {
              setState(() {
                _mappedKeys.clear();
                axisMapping.clear();
                buttonMapping.clear();
              });
            },
          ),
        ],
      ),
      body: SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            children: [
              _buildMappingTable(),
              const SizedBox(height: 16),
            ],
          ),
        ),
      ),
    );
  }

  @override
  void dispose() {
    _subscription?.cancel();
    super.dispose();
  }

  Future<void> _saveMapping() async {
    final prefs = await SharedPreferences.getInstance();
    final mapping = {
      'axisMapping': axisMapping.map((key, value) => MapEntry(key, {
            'keyName': value.keyName.toString(),
            'maxValue': value.maxValue,
            'minValue': value.minValue,
            'reverse': value.reverse,
          })),
      'buttonMapping': buttonMapping.map((key, value) => MapEntry(key, {
            'keyName': value.keyName.toString(),
            'maxValue': value.maxValue,
            'minValue': value.minValue,
            'reverse': value.reverse,
          })),
    };

    await prefs.setString('gamepadMapping', jsonEncode(mapping));
    if (context.mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('映射已保存')),
      );
    }
  }
}
