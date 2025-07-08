import 'dart:typed_data';

class PointCloud2 {
  Header? header;
  int? height;
  int? width;
  List<PointField>? fields;
  bool? isBigendian;
  int? pointStep;
  int? rowStep;
  Uint8List? data;
  bool? isDense;

  PointCloud2({
    this.header,
    this.height,
    this.width,
    this.fields,
    this.isBigendian,
    this.pointStep,
    this.rowStep,
    this.data,
    this.isDense,
  });

  PointCloud2.fromJson(Map<String, dynamic> json) {
    try {
      header = json['header'] != null ? Header.fromJson(json['header']) : null;
      height = json['height'] as int? ?? 0;
      width = json['width'] as int? ?? 0;
      isBigendian = json['is_bigendian'] as bool? ?? false;
      pointStep = json['point_step'] as int? ?? 0;
      rowStep = json['row_step'] as int? ?? 0;
      isDense = json['is_dense'] as bool? ?? false;

      // 解析字段信息
      if (json['fields'] != null) {
        fields = (json['fields'] as List<dynamic>)
            .map((field) => PointField.fromJson(field))
            .toList();
      }

      // 解析点云数据
      if (json['data'] != null) {
        List<int> dataList = List<int>.from(json['data']);
        data = Uint8List.fromList(dataList);
      }
    } catch (e) {
      print("Error parsing PointCloud2 data: $e");
      data = Uint8List(0);
      fields = [];
    }
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = <String, dynamic>{};
    if (header != null) {
      data['header'] = header!.toJson();
    }
    data['height'] = height;
    data['width'] = width;
    if (fields != null) {
      data['fields'] = fields!.map((field) => field.toJson()).toList();
    }
    data['is_bigendian'] = isBigendian;
    data['point_step'] = pointStep;
    data['row_step'] = rowStep;
    if (this.data != null) {
      data['data'] = this.data!.toList();
    }
    data['is_dense'] = isDense;
    return data;
  }

  // 获取点云中的点数量
  int get pointCount {
    return height! * width!;
  }

  // 解析点云数据为3D点列表
  List<Point3D> getPoints() {
    List<Point3D> points = [];
    
    if (data == null || fields == null || pointStep == null) {
      return points;
    }

    // 查找x, y, z字段的偏移量
    int? xOffset, yOffset, zOffset;
    for (PointField field in fields!) {
      switch (field.name) {
        case 'x':
          xOffset = field.offset;
          break;
        case 'y':
          yOffset = field.offset;
          break;
        case 'z':
          zOffset = field.offset;
          break;
      }
    }

    if (xOffset == null || yOffset == null || zOffset == null) {
      return points;
    }

    // 解析每个点
    for (int i = 0; i < pointCount; i++) {
      int baseOffset = i * pointStep!;
      
      double x = _getFloatFromBytes(data!, baseOffset + xOffset!);
      double y = _getFloatFromBytes(data!, baseOffset + yOffset!);
      double z = _getFloatFromBytes(data!, baseOffset + zOffset!);
      
      points.add(Point3D(x, y, z));
    }

    return points;
  }

  // 从字节数组中提取浮点数
  double _getFloatFromBytes(Uint8List bytes, int offset) {
    if (offset + 3 >= bytes.length) return 0.0;
    
    ByteData byteData = ByteData.sublistView(bytes, offset, offset + 4);
    return byteData.getFloat32(0, Endian.little);
  }
}

class PointField {
  String? name;
  int? offset;
  int? datatype;
  int? count;

  PointField({this.name, this.offset, this.datatype, this.count});

  PointField.fromJson(Map<String, dynamic> json) {
    name = json['name'] as String?;
    offset = json['offset'] as int? ?? 0;
    datatype = json['datatype'] as int? ?? 0;
    count = json['count'] as int? ?? 0;
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = <String, dynamic>{};
    data['name'] = name;
    data['offset'] = offset;
    data['datatype'] = datatype;
    data['count'] = count;
    return data;
  }
}

class Point3D {
  final double x;
  final double y;
  final double z;

  Point3D(this.x, this.y, this.z);

  @override
  String toString() {
    return 'Point3D(x: $x, y: $y, z: $z)';
  }
}

class Header {
  int? seq;
  Stamp? stamp;
  String? frameId;

  Header({this.seq, this.stamp, this.frameId});

  Header.fromJson(Map<String, dynamic> json) {
    seq = json['seq'];
    stamp = json['stamp'] != null ? Stamp.fromJson(json['stamp']) : null;
    frameId = json['frame_id'];
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = <String, dynamic>{};
    data['seq'] = seq;
    if (stamp != null) {
      data['stamp'] = stamp!.toJson();
    }
    data['frame_id'] = frameId;
    return data;
  }
}

class Stamp {
  int? secs;
  int? nsecs;

  Stamp({this.secs, this.nsecs});

  Stamp.fromJson(Map<String, dynamic> json) {
    secs = json['secs'];
    nsecs = json['nsecs'];
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = <String, dynamic>{};
    data['secs'] = secs;
    data['nsecs'] = nsecs;
    return data;
  }
} 