import 'package:flutter/material.dart';

enum LayerType {
  grid,
  globalCostmap,
  localCostmap,
  laser,
  pointCloud,
  globalPath,
  localPath,
  tracePath,
  topology,
  robotFootprint,
}

LayerType? layerTypeFromStorageId(String id) {
  for (final t in LayerType.values) {
    if (t.name == id) return t;
  }
  return null;
}

bool _isPathType(LayerType t) {
  return t == LayerType.globalPath ||
      t == LayerType.localPath ||
      t == LayerType.tracePath;
}

sealed class LayerConfig {
  bool enable;

  LayerConfig({required this.enable});

  LayerType get layerType;

  Map<String, dynamic> toJson();

  static LayerConfig defaultFor(LayerType t) {
    switch (t) {
      case LayerType.grid:
        return LayerToggleConfig(layerType: t, enable: true);
      case LayerType.globalCostmap:
      case LayerType.localCostmap:
      case LayerType.pointCloud:
        return LayerToggleConfig(layerType: t, enable: false);
      case LayerType.laser:
        return LayerLaserConfig(
          enable: true,
          colorArgb: Colors.red.toARGB32().toString(),
          dotRadius: '2.0',
        );
      case LayerType.globalPath:
        return LayerPathConfig(
          layerType: t,
          enable: true,
          colorArgb: const Color(0xFF2196F3).toARGB32().toString(),
        );
      case LayerType.localPath:
        return LayerPathConfig(
          layerType: t,
          enable: true,
          colorArgb: Colors.green.toARGB32().toString(),
        );
      case LayerType.tracePath:
        return LayerPathConfig(
          layerType: t,
          enable: true,
          colorArgb: Colors.yellow.toARGB32().toString(),
        );
      case LayerType.topology:
      case LayerType.robotFootprint:
        return LayerToggleConfig(layerType: t, enable: true);
    }
  }

  static LayerConfig fromStorageEntry(String storageId, Map<String, dynamic> j) {
    final t = layerTypeFromStorageId(storageId);
    if (t == null) {
      return LayerToggleConfig(layerType: LayerType.grid, enable: false);
    }
    if (t == LayerType.laser) {
      return LayerLaserConfig.fromJson(j);
    }
    if (_isPathType(t)) {
      return LayerPathConfig.fromJson(j, t);
    }
    return LayerToggleConfig.fromJson(j, t);
  }

  static LayerConfig mergeDecoded(
      String storageId, LayerConfig previous, Map<String, dynamic> raw) {
    final incoming = fromStorageEntry(storageId, raw);
    return switch (previous) {
      LayerToggleConfig p when incoming is LayerToggleConfig => LayerToggleConfig(
          layerType: p.layerType,
          enable: incoming.enable,
        ),
      LayerPathConfig p when incoming is LayerPathConfig => LayerPathConfig(
          layerType: p.layerType,
          enable: incoming.enable,
          colorArgb: incoming.colorArgb.isNotEmpty ? incoming.colorArgb : p.colorArgb,
        ),
      LayerLaserConfig p when incoming is LayerLaserConfig => LayerLaserConfig(
          enable: incoming.enable,
          colorArgb: incoming.colorArgb.isNotEmpty ? incoming.colorArgb : p.colorArgb,
          dotRadius: incoming.dotRadius.isNotEmpty ? incoming.dotRadius : p.dotRadius,
        ),
      _ => previous,
    };
  }
}

enum LayerJsonType { toggle, path, laser }

final class LayerToggleConfig extends LayerConfig {
  @override
  final LayerType layerType;

  LayerToggleConfig({required this.layerType, required super.enable});

  factory LayerToggleConfig.fromJson(Map<String, dynamic> j, LayerType t) {
    return LayerToggleConfig(
      layerType: t,
      enable: j['enable'] == true,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'type': LayerJsonType.toggle.name,
        'enable': enable,
      };
}

final class LayerPathConfig extends LayerConfig {
  @override
  final LayerType layerType;
  String colorArgb;

  LayerPathConfig({
    required this.layerType,
    required super.enable,
    required this.colorArgb,
  });

  factory LayerPathConfig.fromJson(Map<String, dynamic> j, LayerType t) {
    return LayerPathConfig(
      layerType: t,
      enable: j['enable'] == true,
      colorArgb: j['colorArgb']?.toString() ?? '',
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'type': LayerJsonType.path.name,
        'enable': enable,
        'colorArgb': colorArgb,
      };
}

final class LayerLaserConfig extends LayerConfig {
  @override
  LayerType get layerType => LayerType.laser;

  String colorArgb;
  String dotRadius;

  LayerLaserConfig({
    required super.enable,
    required this.colorArgb,
    required this.dotRadius,
  });

  factory LayerLaserConfig.fromJson(Map<String, dynamic> j) {
    return LayerLaserConfig(
      enable: j['enable'] == true,
      colorArgb: j['colorArgb']?.toString() ?? '',
      dotRadius: j['dotRadius']?.toString() ?? '',
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'type': LayerJsonType.laser.name,
        'enable': enable,
        'colorArgb': colorArgb,
        'dotRadius': dotRadius,
      };
}

LayerConfig patchLayerConfig(
  LayerConfig current, {
  bool? enable,
  String? colorArgb,
  String? dotRadius,
}) {
  return switch (current) {
    LayerToggleConfig c => LayerToggleConfig(
        layerType: c.layerType,
        enable: enable ?? c.enable,
      ),
    LayerPathConfig c => LayerPathConfig(
        layerType: c.layerType,
        enable: enable ?? c.enable,
        colorArgb: colorArgb ?? c.colorArgb,
      ),
    LayerLaserConfig c => LayerLaserConfig(
        enable: enable ?? c.enable,
        colorArgb: colorArgb ?? c.colorArgb,
        dotRadius: dotRadius ?? c.dotRadius,
      ),
  };
}

Color layerConfigColorArgb(LayerConfig cfg, Color fallback) {
  final s = switch (cfg) {
    LayerPathConfig(:final colorArgb) => colorArgb,
    LayerLaserConfig(:final colorArgb) => colorArgb,
    _ => '',
  };
  if (s.isEmpty) return fallback;
  final i = int.tryParse(s);
  if (i == null) return fallback;
  return Color(i);
}

double layerConfigLaserDotRadius(LayerConfig cfg) {
  if (cfg is! LayerLaserConfig) return 2.0;
  final v = double.tryParse(cfg.dotRadius) ?? 2.0;
  return v.clamp(0.5, 12.0);
}
