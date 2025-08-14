import 'package:flutter/material.dart';
import 'package:gamepads/gamepads.dart';
import 'dart:async';
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import '../global/setting.dart';
import 'package:fluttertoast/fluttertoast.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';

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
      globalSetting.axisMapping.forEach((key, value) {
        _mappedKeys[value.keyName] = key;
      });
      // 加载默认的按钮映射
      globalSetting.buttonMapping.forEach((key, value) {
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
        if (value.count > maxCount) {
          maxCount = value.count;
          maxResult = value;
        }
        if (value.value >= maxValue) {
          maxValue = value.value;
        }
      });

      int detect_count = 10;

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

        // 保存映射
        if (_editingKey == "左摇杆上") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.leftAxisY);
          //上为正值
          if (maxResult!.value < 0) {
            j_event.reverse = true;
          }
          j_event.maxValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        } else if (_editingKey == "左摇杆下") {
          globalSetting.axisMapping[maxResult!.key]!.minValue =
              maxResult!.value;
        } else if (_editingKey == "左摇杆左") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.leftAxisX);
          //右为正值
          if (maxResult!.value > 0) {
            j_event.reverse = true;
          }
          j_event.minValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        } else if (_editingKey == "左摇杆右") {
          globalSetting.axisMapping[maxResult!.key]!.maxValue =
              maxResult!.value;
        } else if (_editingKey == "右摇杆上") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.rightAxisY);
          //上为正值
          if (maxResult!.value < 0) {
            j_event.reverse = true;
          }
          j_event.maxValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        } else if (_editingKey == "右摇杆下") {
          globalSetting.axisMapping[maxResult!.key]!.minValue =
              maxResult!.value;
        } else if (_editingKey == "右摇杆左") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.rightAxisX);
          //右为正值
          if (maxResult!.value > 0) {
            j_event.reverse = true;
          }
          j_event.minValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        } else if (_editingKey == "右摇杆右") {
          globalSetting.axisMapping[maxResult!.key]!.maxValue =
              maxResult!.value;
        } else if (_editingKey == "按钮 A") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.buttonA);
          //右为正值
          if (maxResult!.value < 0) {
            j_event.reverse = true;
          }
          j_event.maxValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        } else if (_editingKey == "按钮 B") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.buttonB);
          //右为正值
          if (maxResult!.value < 0) {
            j_event.reverse = true;
          }
          j_event.maxValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        } else if (_editingKey == "按钮 X") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.buttonX);
          //右为正值
          if (maxResult!.value < 0) {
            j_event.reverse = true;
          }
          j_event.maxValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        } else if (_editingKey == "按钮 Y") {
          JoyStickEvent j_event = JoyStickEvent(KeyName.buttonY);
          //右为正值
          if (maxResult!.value < 0) {
            j_event.reverse = true;
          }
          j_event.maxValue = maxResult!.value;

          globalSetting.axisMapping[maxResult!.key] = j_event;
        }
        globalSetting.saveGamepadMapping();

        return;
      }

      // 如果当前正在映射，则记录检测到的值
      if (curAbsValue > 0 && _editingKey.isNotEmpty) {
        print("${_recvEvent.key}: ${curAbsValue}");
        if (_mappingResults.containsKey(_recvEvent.key)) {
          if (curAbsValue >= _mappingResults[_recvEvent.key]!.value.abs() &&
              curAbsValue >= maxValue) {
            _mappingResults[_recvEvent.key]!.count++;
            _mappingResults[_recvEvent.key]!.value = event.value;
          }
        } else {
          _mappingResults[_recvEvent.key] = MappingResult(
              key: _recvEvent.key,
              value: event.value,
              event: _recvEvent,
              count: 1);
        }
      }
      _msg =
          '请多次推动摇杆或按键至该位置，当前检测到的值: ${_recvEvent.key}: ${_recvEvent.value.toStringAsFixed(2)},最优值：${maxResult?.key}: ${maxResult?.value.toStringAsFixed(2)} 还需触发检测 ${detect_count - maxCount} 次';
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
          _buildDataRow('右摇杆左'),
          _buildDataRow('右摇杆右'),
          ...globalSetting.buttonMapping.keys
              .map((key) => _buildDataRow(_translateKey(key))),
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
                    _mappingResults = {};

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
                      _msg = AppLocalizations.of(context)!.start_mapping_message;
                      _startListening();
                    });
                  },
                  child: Text(AppLocalizations.of(context)!.remap),
                ),
              ],
            ],
          ),
        ),
      ],
    );
  }

  String _translateKey(String key) {
    final translations = {
      "AXIS_X": AppLocalizations.of(context)!.left_stick_x,
      "AXIS_Y": AppLocalizations.of(context)!.left_stick_y,
      "AXIS_Z": AppLocalizations.of(context)!.right_stick_x,
      "AXIS_RZ": AppLocalizations.of(context)!.right_stick_y,
      // "triggerRight": "右扳机",
      // "triggerLeft": "左扳机",
      // "buttonLeftRight": "方向键 左/右",
      // "buttonUpDown": "方向键 上/下",
      "KEYCODE_BUTTON_A": AppLocalizations.of(context)!.button_a,
      "KEYCODE_BUTTON_B": AppLocalizations.of(context)!.button_b,
      "KEYCODE_BUTTON_X": AppLocalizations.of(context)!.button_x,
      "KEYCODE_BUTTON_Y": AppLocalizations.of(context)!.button_y,
      // "KEYCODE_BUTTON_L1": "按钮 L1",
      // "KEYCODE_BUTTON_R1": "按钮 R1",
    };
    return translations[key] ?? key;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(AppLocalizations.of(context)!.gamepad_mapping),
        actions: [
          IconButton(
            icon: const Icon(Icons.reset_tv),
            onPressed: () async {
              await globalSetting.resetGamepadMapping();
              ScaffoldMessenger.of(context).showSnackBar(
                SnackBar(content: Text(AppLocalizations.of(context)!.mapping_reset)),
              );
              setState(() {});
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
}
