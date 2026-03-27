import 'package:ros_flutter_gui_app/basic/gui_layer_type.dart';

class UserLayerSpec {
  final String id;
  final GuiLayerType type;
  final String topic;
  final bool enable;
  final String colorArgb;
  final String dotRadius;

  const UserLayerSpec({
    required this.id,
    required this.type,
    required this.topic,
    this.enable = true,
    this.colorArgb = '',
    this.dotRadius = '',
  });

  bool get needsTopic =>
      type == GuiLayerType.laser ||
      type == GuiLayerType.path ||
      type == GuiLayerType.localCostmap ||
      type == GuiLayerType.globalCostmap ||
      type == GuiLayerType.pointCloud ||
      type == GuiLayerType.robotFootprint;

  static bool isReservedBuiltinId(String id) {
    const reserved = {
      'grid',
      'globalCostmap',
      'localCostmap',
      'laser',
      'pointCloud',
      'globalPath',
      'localPath',
      'tracePath',
      'topology',
      'robotFootprint',
    };
    return reserved.contains(id);
  }

  static UserLayerSpec? tryParse(Map<String, dynamic> m) {
    final id = m['id']?.toString().trim() ?? '';
    if (id.isEmpty || isReservedBuiltinId(id)) {
      return null;
    }
    final type = GuiLayerType.fromJson(m['type'] ?? m['layerType']);
    if (type == null ||
        type == GuiLayerType.unspecified ||
        !type.allowsCustomStream) {
      return null;
    }
    final topic = m['topic']?.toString().trim() ?? '';
    if (topic.isEmpty) {
      return null;
    }
    return UserLayerSpec(
      id: id,
      type: type,
      topic: topic,
      enable: m['enable'] != false,
      colorArgb: m['colorArgb']?.toString() ?? '',
      dotRadius: m['dotRadius']?.toString() ?? '',
    );
  }

  Map<String, dynamic> toSettingsJson() {
    return {
      'id': id,
      'type': type.toJson(),
      'topic': topic,
      'enable': enable,
      if (colorArgb.isNotEmpty) 'colorArgb': colorArgb,
      if (dotRadius.isNotEmpty) 'dotRadius': dotRadius,
    };
  }

  UserLayerSpec copyWith({
    String? id,
    GuiLayerType? type,
    String? topic,
    bool? enable,
    String? colorArgb,
    String? dotRadius,
  }) {
    return UserLayerSpec(
      id: id ?? this.id,
      type: type ?? this.type,
      topic: topic ?? this.topic,
      enable: enable ?? this.enable,
      colorArgb: colorArgb ?? this.colorArgb,
      dotRadius: dotRadius ?? this.dotRadius,
    );
  }
}
