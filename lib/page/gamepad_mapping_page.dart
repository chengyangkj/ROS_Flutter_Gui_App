import 'package:flutter/material.dart';
import 'package:gamepads/gamepads.dart';
import 'dart:async';
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import '../global/setting.dart';
import 'package:fluttertoast/fluttertoast.dart';

class GamepadMappingPage extends StatefulWidget {
  const GamepadMappingPage({super.key});

  @override
  State<GamepadMappingPage> createState() => _GamepadMappingPageState();
}

class MappingResult {
  String key = "";
  double value = 0;
  GamepadEvent? event;
  int count = 0;

  MappingResult(
      {required this.key,
      required this.value,
      this.event,
      required this.count});
}

class _GamepadMappingPageState extends State<GamepadMappingPage> {
  StreamSubscription? _subscription;

  GamepadEvent _recvEvent = GamepadEvent(
      gamepadId: "0",
      timestamp: 0,
      type: KeyType.analog,
      key: "None",
      value: 0);

  final Map<KeyName, String> _mappedKeys = {};
  String _editingKey = ""; // 当前正在编辑的按键
  Map<String, MappingResult> _mappingResults = {};

  // 添加保存原始映射的变量
  Map<String, JoyStickEvent> _originalAxisMapping = {};
  Map<String, JoyStickEvent> _originalButtonMapping = {};

  OverlayEntry? _overlayEntry;

  String _msg = ""; // 添加用于显示消息的变量

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
    _subscription = Gamepads.events.listen((event) async {
      _recvEvent = event;

      var curAbsValue = _recvEvent.value.abs();

      // 获取检测到的次数最多的值
      double maxValue = 0;
      int maxCount = 0;
      MappingResult? maxResult;
      _mappingResults.forEach((key, value) {
        print("$key: ${value.count}");
        if (value.count > maxCount) {
          maxCount = value.count;
          maxResult = value;
        }
        if (value.value >= maxValue) {
          maxValue = value.value;
        }
      });

      int detect_count = 5;

      // 如果检测次数大于次数，则认为已经完成映射
      if (maxCount >= detect_count) {
        _editingKey = "";
        // _mappedKeys[maxResult!.event!.key] = maxResult!.key;
        print(maxResult);
        _mappingResults = {};
        _editingKey = "";
        // 显示成功的弹窗
        if (context.mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(content: Text('映射成功,当前键位映射到 ${maxResult!.key}')),
          );
        }
        return;
      }

      // 如果当前正在映射，则记录检测到的值
      if (curAbsValue > 0 && _editingKey.isNotEmpty) {
        print("${_recvEvent.key}: ${curAbsValue}");
        if (_mappingResults.containsKey(_recvEvent.key)) {
          if (curAbsValue >= _mappingResults[_recvEvent.key]!.value &&
              curAbsValue >= maxValue) {
            _mappingResults[_recvEvent.key]!.count++;
            _mappingResults[_recvEvent.key]!.value = curAbsValue;
          }
        } else {
          _mappingResults[_recvEvent.key] = MappingResult(
              key: _recvEvent.key,
              value: curAbsValue,
              event: _recvEvent,
              count: 1);
        }

        _msg =
            '请多次推动摇杆或按键至该位置，当前检测到的值: ${_recvEvent.key}: ${_recvEvent.value.toStringAsFixed(2)},最优值：${maxResult?.key}: ${maxResult?.value.toStringAsFixed(2)} 还需触发检测 ${detect_count - maxCount} 次';
      }
      setState(() {});
    });
  }

  Widget _buildMappingTable() {
    return SingleChildScrollView(
      scrollDirection: Axis.horizontal,
      child: DataTable(
        columns: const [
          DataColumn(label: Text('按键')),
          DataColumn(label: Text('操作')),
        ],
        rows: [
          _buildDataRow('左摇杆上'),
          _buildDataRow('左摇杆下'),
          _buildDataRow('左摇杆左'),
          _buildDataRow('左摇杆右'),
          _buildDataRow('右摇杆上'),
          _buildDataRow('右摇杆下'),
          _buildDataRow('右摇杆最左侧'),
          _buildDataRow('右摇杆最右侧'),
          ...buttonMapping.keys.map((key) => _buildDataRow(_translateKey(key))),
        ],
      ),
    );
  }

  void _showPersistentBottomMessage(BuildContext context, String message) {
    _removePersistentBottomMessage(); // 先移除之前的 OverlayEntry
    _overlayEntry = OverlayEntry(
      builder: (context) => Positioned(
        bottom: 50.0,
        left: 0,
        right: 0,
        child: Material(
          color: Colors.transparent,
          child: Container(
            padding:
                const EdgeInsets.symmetric(horizontal: 24.0, vertical: 12.0),
            margin: const EdgeInsets.symmetric(horizontal: 24.0),
            decoration: BoxDecoration(
              color: Colors.black54,
              borderRadius: BorderRadius.circular(8.0),
            ),
            child: Text(
              message,
              style: const TextStyle(color: Colors.white),
              textAlign: TextAlign.center,
            ),
          ),
        ),
      ),
    );

    Overlay.of(context)?.insert(_overlayEntry!);
  }

  void _removePersistentBottomMessage() {
    _overlayEntry?.remove();
    _overlayEntry = null;
  }

  DataRow _buildDataRow(String key) {
    bool isEditing = _editingKey == key;

    return DataRow(
      cells: [
        DataCell(Text(key)),
        DataCell(
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              if (isEditing) ...[
                const SizedBox(width: 8),
                ElevatedButton(
                  onPressed: () {
                    _editingKey = "";
                    setState(() {});
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.grey,
                  ),
                  child: const Text('取消'),
                ),
              ] else ...[
                ElevatedButton(
                  onPressed: () {
                    // 开始重新映射
                    setState(() {
                      _editingKey = key;
                      _mappingResults = {};
                      _startListening();
                    });
                  },
                  child: const Text('重新映射'),
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
      bottomNavigationBar: _editingKey.isNotEmpty
          ? Container(
              color: Colors.black54,
              padding: const EdgeInsets.all(8.0),
              child: Text(
                _msg,
                style: const TextStyle(color: Colors.white),
                textAlign: TextAlign.center,
              ),
            )
          : null,
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
