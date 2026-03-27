import 'package:ros_flutter_gui_app/basic/key_value.dart';

class DiagnosticStatus {
  static const int OK = 0;
  static const int WARN = 1;
  static const int ERROR = 2;
  static const int STALE = 3;

  int level;
  String name;
  String message;
  String hardwareId;
  List<KeyValue> values;

  DiagnosticStatus({
    required this.level,
    required this.name,
    required this.message,
    required this.hardwareId,
    this.values = const [],
  });

  factory DiagnosticStatus.fromJson(Map<String, dynamic> json) {
    return DiagnosticStatus(
      level: json['level'] ?? 0,
      name: json['name'] ?? '',
      message: json['message'] ?? '',
      hardwareId: json['hardware_id'] ?? '',
      values: (json['values'] as List<dynamic>?)
          ?.map((item) => KeyValue.fromJson(item))
          .toList() ?? [],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'level': level,
      'name': name,
      'message': message,
      'hardware_id': hardwareId,
      'values': values.map((item) => item.toJson()).toList(),
    };
  }

  String get levelString {
    switch (level) {
      case OK:
        return 'OK';
      case WARN:
        return 'WARN';
      case ERROR:
        return 'ERROR';
      case STALE:
        return 'STALE';
      default:
        return 'UNKNOWN';
    }
  }

  String get levelDisplayName {
    switch (level) {
      case OK:
        return '正常';
      case WARN:
        return '警告';
      case ERROR:
        return '错误';
      case STALE:
        return '过期';
      default:
        return '未知';
    }
  }
}