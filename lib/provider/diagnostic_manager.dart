import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_array.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_status.dart';

class DiagnosticState {
  int level;
  String message;
  Map<String, String> keyValues;
  DateTime lastUpdateTime;

  DiagnosticState({
    required this.level,
    required this.message,
    Map<String, String>? keyValues,
    DateTime? lastUpdateTime,
  }) : keyValues = keyValues ?? {},
       lastUpdateTime = lastUpdateTime ?? DateTime.now();

  factory DiagnosticState.fromDiagnosticStatus(DiagnosticStatus status) {
    Map<String, String> kvMap = {};
    for (var kv in status.values) {
      kvMap[kv.key] = kv.value;
    }
    
    return DiagnosticState(
      level: status.level,
      message: status.message,
      keyValues: kvMap,
    );
  }

  DiagnosticState copyWith({
    int? level,
    String? message,
    Map<String, String>? keyValues,
    DateTime? lastUpdateTime,
  }) {
    return DiagnosticState(
      level: level ?? this.level,
      message: message ?? this.message,
      keyValues: keyValues ?? Map.from(this.keyValues),
      lastUpdateTime: lastUpdateTime ?? this.lastUpdateTime,
    );
  }

  String get levelDisplayName {
    switch (level) {
      case DiagnosticStatus.OK:
        return '正常';
      case DiagnosticStatus.WARN:
        return '警告';
      case DiagnosticStatus.ERROR:
        return '错误';
      case DiagnosticStatus.STALE:
        return '失活';
      default:
        return '未知';
    }
  }

  Color get levelColor {
    switch (level) {
      case DiagnosticStatus.OK:
        return const Color(0xFF4CAF50);
      case DiagnosticStatus.WARN:
        return const Color(0xFFFF9800);
      case DiagnosticStatus.ERROR:
        return const Color(0xFFF44336);
      case DiagnosticStatus.STALE:
        return const Color(0xFF9E9E9E);
      default:
        return const Color(0xFF9E9E9E);
    }
  }

  IconData get levelIcon {
    switch (level) {
      case DiagnosticStatus.OK:
        return Icons.check_circle;
      case DiagnosticStatus.WARN:
        return Icons.warning;
      case DiagnosticStatus.ERROR:
        return Icons.error;
      case DiagnosticStatus.STALE:
        return Icons.schedule;
      default:
        return Icons.help;
    }
  }
}

class DiagnosticManager extends ChangeNotifier {
  // Map<hardware_id, Map<component_name, DiagnosticState>>
  final Map<String, Map<String, DiagnosticState>> _diagnosticStates = {};
  
  // 过期检测定时器
  Timer? _staleCheckTimer;
  
  // 过期时间阈值（5秒）
  static const Duration _staleThreshold = Duration(seconds: 5);
  
  // 新错误/警告回调函数
  Function(List<Map<String, dynamic>>)? _onNewErrorsWarnings;
  
  // 构造函数
  DiagnosticManager() {
    _startStaleCheckTimer();
  }

  // 析构函数
  @override
  void dispose() {
    _stopStaleCheckTimer();
    super.dispose();
  }

  // 启动过期检测定时器
  void _startStaleCheckTimer() {
    _staleCheckTimer?.cancel();
    _staleCheckTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      _checkForStaleStates();
    });
  }

  // 停止过期检测定时器
  void _stopStaleCheckTimer() {
    _staleCheckTimer?.cancel();
    _staleCheckTimer = null;
  }

  // 检查过期状态
  void _checkForStaleStates() {
    bool hasChanges = false;
    List<Map<String, dynamic>> newStaleStates = []; // 存储新变为失活的状态
    final now = DateTime.now();

    var stateHardwareId=[];

    for (var hardwareEntry in _diagnosticStates.entries) {
      for (var componentEntry in hardwareEntry.value.entries) {
        final state = componentEntry.value;
        final timeSinceUpdate = now.difference(state.lastUpdateTime);

        // 如果超过5秒未更新且当前不是过期状态，则设置为过期
        if (timeSinceUpdate > _staleThreshold && state.level != DiagnosticStatus.STALE) {
          final newStaleState = state.copyWith(
            level: DiagnosticStatus.STALE,
            message: '(已过期)',
            lastUpdateTime: state.lastUpdateTime, // 保持原始更新时间
          );
          
          _diagnosticStates[hardwareEntry.key]![componentEntry.key] = newStaleState;
          hasChanges = true;
          
          if( hardwareEntry.key!="Node Start History" &&!stateHardwareId.contains(hardwareEntry.key)){
            stateHardwareId.add(hardwareEntry.key);
          }
         
        }
      }
    }

    if (hasChanges) {
      notifyListeners();
      
      // 如果有新变为失活的状态，触发回调
      if (stateHardwareId.isNotEmpty) {
        for (var hardwareId in stateHardwareId) {
          newStaleStates.add({
            'hardwareId': hardwareId,
            'componentName': '',
            'state': DiagnosticState(level: DiagnosticStatus.STALE, message: '超过5s未更新数据'),
          });
        }
        _onNewErrorsWarnings?.call(newStaleStates);
      }
    }
  }

  // 设置新错误/警告回调函数
  void setOnNewErrorsWarnings(Function(List<Map<String, dynamic>>) callback) {
    _onNewErrorsWarnings = callback;
  }

  // 获取所有硬件ID
  List<String> get hardwareIds => _diagnosticStates.keys.toList();
  
  // 获取指定硬件的所有组件
  List<String> getComponentsForHardware(String hardwareId) {
    return _diagnosticStates[hardwareId]?.keys.toList() ?? [];
  }
  
  // 获取指定硬件和组件的状态
  DiagnosticState? getState(String hardwareId, String componentName) {
    return _diagnosticStates[hardwareId]?[componentName];
  }
  
  // 获取指定硬件的所有状态
  Map<String, DiagnosticState> getStatesForHardware(String hardwareId) {
    return Map.from(_diagnosticStates[hardwareId] ?? {});
  }
  
  // 获取指定硬件的最高状态级别
  int getMaxLevelForHardware(String hardwareId) {
    final states = _diagnosticStates[hardwareId];
    if (states == null || states.isEmpty) return DiagnosticStatus.OK;
    
    int maxLevel = DiagnosticStatus.OK;
    for (var state in states.values) {
      if (state.level > maxLevel) {
        maxLevel = state.level;
      }
    }
    return maxLevel;
  }
  
  // 获取所有状态的统计信息
  Map<int, int> getStatusCounts() {
    Map<int, int> counts = {
      DiagnosticStatus.OK: 0,
      DiagnosticStatus.WARN: 0,
      DiagnosticStatus.ERROR: 0,
      DiagnosticStatus.STALE: 0,
    };
    
    for (var hardwareStates in _diagnosticStates.values) {
      for (var state in hardwareStates.values) {
        counts[state.level] = (counts[state.level] ?? 0) + 1;
      }
    }
    
    return counts;
  }
  
  // 更新诊断状态
  void updateDiagnosticStates(DiagnosticArray diagnosticArray) {
    List<Map<String, dynamic>> newErrorsWarnings = []; // 存储新出现的错误和警告
    
    for (var status in diagnosticArray.status) {

      //进程启动时会发布这个诊断信息
      if(status.message== "Node starting up"){
        status.hardwareId="Node Start History";
      }
      final hardwareId = status.hardwareId.isEmpty ? '未知硬件' : status.hardwareId;
      final componentName = status.name;
      
      // 检查是否为新的错误或警告
      final existingState = _diagnosticStates[hardwareId]?[componentName];
      bool isNewErrorOrWarning = false;
      
      if (status.level == DiagnosticStatus.ERROR || status.level == DiagnosticStatus.WARN) {
        // 如果没有之前的状态，或者之前的状态不是错误/警告，则认为是新出现的
        if (existingState == null || 
            (existingState.level != DiagnosticStatus.ERROR && existingState.level != DiagnosticStatus.WARN)) {
          isNewErrorOrWarning = true;
        }
      }
      
      // 创建新的状态，使用当前时间作为更新时间
      final newState = DiagnosticState.fromDiagnosticStatus(status);
      
      _diagnosticStates[hardwareId] ??= {};
      // 更新状态
      _diagnosticStates[hardwareId]![componentName] = newState;
      
      // 如果是新出现的错误或警告，添加到列表中
      if (isNewErrorOrWarning) {
        newErrorsWarnings.add({
          'hardwareId': hardwareId,
          'componentName': componentName,
          'state': newState,
        });
      }
    }

    // 通知监听者，并传递新出现的错误和警告信息
    notifyListeners();
    
    // 如果有新出现的错误或警告，触发回调
    if (newErrorsWarnings.isNotEmpty) {
      _onNewErrorsWarnings?.call(newErrorsWarnings);
    }
  }
  
  // 清除所有诊断状态
  void clearAllStates() {
    _diagnosticStates.clear();
    notifyListeners();
  }
  
  // 清除指定硬件的状态
  void clearHardwareStates(String hardwareId) {
    _diagnosticStates.remove(hardwareId);
    notifyListeners();
  }
  
  // 清除指定组件的状态
  void clearComponentState(String hardwareId, String componentName) {
    _diagnosticStates[hardwareId]?.remove(componentName);
    if (_diagnosticStates[hardwareId]?.isEmpty == true) {
      _diagnosticStates.remove(hardwareId);
    }
    notifyListeners();
  }
  
  
  // 获取所有诊断状态的扁平列表（用于搜索和筛选）
  List<MapEntry<String, MapEntry<String, DiagnosticState>>> getAllStates() {
    List<MapEntry<String, MapEntry<String, DiagnosticState>>> result = [];
    
    for (var hardwareEntry in _diagnosticStates.entries) {
      for (var componentEntry in hardwareEntry.value.entries) {
        result.add(MapEntry(hardwareEntry.key, componentEntry));
      }
    }
    
    return result;
  }
  
  // 搜索诊断状态
  List<MapEntry<String, MapEntry<String, DiagnosticState>>> searchStates(String query) {
    if (query.isEmpty) return getAllStates();
    
    final lowerQuery = query.toLowerCase();
    return getAllStates().where((entry) {
      final hardwareId = entry.key.toLowerCase();
      final componentName = entry.value.key.toLowerCase();
      final state = entry.value.value;
      
      return hardwareId.contains(lowerQuery) ||
             componentName.contains(lowerQuery) ||
             state.message.toLowerCase().contains(lowerQuery) ||
             state.keyValues.values.any((value) => value.toLowerCase().contains(lowerQuery));
    }).toList();
  }
  
  // 按状态级别筛选
  List<MapEntry<String, MapEntry<String, DiagnosticState>>> filterByLevel(int level) {
    return getAllStates().where((entry) {
      return entry.value.value.level == level;
    }).toList();
  }
}
