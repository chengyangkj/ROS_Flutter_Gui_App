enum GuiLayerType {
  unspecified(0),
  laser(1),
  path(2),
  localCostmap(3),
  globalCostmap(4),
  pointCloud(5),
  robotFootprint(6);

  const GuiLayerType(this.protoValue);
  final int protoValue;

  static GuiLayerType? fromProto(int v) {
    for (final t in GuiLayerType.values) {
      if (t.protoValue == v) {
        return t;
      }
    }
    return null;
  }

  static GuiLayerType? fromJson(dynamic raw) {
    if (raw == null) {
      return null;
    }
    if (raw is int) {
      return fromProto(raw);
    }
    if (raw is String) {
      switch (raw) {
        case 'laser':
          return GuiLayerType.laser;
        case 'path':
          return GuiLayerType.path;
        case 'localCostmap':
          return GuiLayerType.localCostmap;
        case 'globalCostmap':
          return GuiLayerType.globalCostmap;
        case 'pointCloud':
          return GuiLayerType.pointCloud;
        case 'robotFootprint':
          return GuiLayerType.robotFootprint;
        default:
          return null;
      }
    }
    return null;
  }

  dynamic toJson() {
    switch (this) {
      case GuiLayerType.unspecified:
        return 0;
      case GuiLayerType.laser:
        return 'laser';
      case GuiLayerType.path:
        return 'path';
      case GuiLayerType.localCostmap:
        return 'localCostmap';
      case GuiLayerType.globalCostmap:
        return 'globalCostmap';
      case GuiLayerType.pointCloud:
        return 'pointCloud';
      case GuiLayerType.robotFootprint:
        return 'robotFootprint';
    }
  }

  bool get allowsCustomStream {
    return this != GuiLayerType.unspecified;
  }
}
