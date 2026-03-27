class LaserScan {
  Header? header;
  double? angleMin;
  double? angleMax;
  double? angleIncrement;
  double? timeIncrement;
  double? scanTime;
  double? rangeMin;
  double? rangeMax;
  List<double>? ranges;
  List<int>? intensities;

  LaserScan(
      {this.header,
      this.angleMin,
      this.angleMax,
      this.angleIncrement,
      this.timeIncrement,
      this.scanTime,
      this.rangeMin,
      this.rangeMax,
      this.ranges,
      this.intensities});

  LaserScan.fromJson(Map<String, dynamic> json) {
    try {
      header = json['header'] != null ? Header.fromJson(json['header']) : null;
      angleMin = (json['angle_min'] as num?)?.toDouble() ?? 0.0;
      angleMax = (json['angle_max'] as num?)?.toDouble() ?? 0.0;
      angleIncrement = (json['angle_increment'] as num?)?.toDouble() ?? 0.0;
      timeIncrement = (json['time_increment'] as num?)?.toDouble() ?? 0.0;
      scanTime = (json['scan_time'] as num?)?.toDouble() ?? 0.0;
      rangeMin = (json['range_min'] as num?)?.toDouble() ?? 0.0;
      rangeMax = (json['range_max'] as num?)?.toDouble() ?? 0.0;

      ranges = (json['ranges'] as List<dynamic>?)?.map((e) {
            if (e == null) return -1.0;
            if (e is num) return e.toDouble();
            return -1.0;
          }).toList() ??
          [];

      intensities = (json['intensities'] as List<dynamic>?)?.map((e) {
            if (e == null) return 0;
            if (e is num) return e.toInt();
            return 0;
          }).toList() ??
          [];
    } catch (e) {
      print("Error parsing LaserScan data: $e");
      // 设置默认值
      ranges = [];
      intensities = [];
    }
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = new Map<String, dynamic>();
    if (this.header != null) {
      data['header'] = this.header!.toJson();
    }
    data['angle_min'] = this.angleMin;
    data['angle_max'] = this.angleMax;
    data['angle_increment'] = this.angleIncrement;
    data['time_increment'] = this.timeIncrement;
    data['scan_time'] = this.scanTime;
    data['range_min'] = this.rangeMin;
    data['range_max'] = this.rangeMax;
    data['ranges'] = this.ranges;
    data['intensities'] = this.intensities;
    return data;
  }
}

class Header {
  int? seq;
  Stamp? stamp;
  String? frameId;

  Header({this.seq, this.stamp, this.frameId});

  Header.fromJson(Map<String, dynamic> json) {
    seq = json['seq'];
    stamp = json['stamp'] != null ? new Stamp.fromJson(json['stamp']) : null;
    frameId = json['frame_id'];
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = new Map<String, dynamic>();
    data['seq'] = this.seq;
    if (this.stamp != null) {
      data['stamp'] = this.stamp!.toJson();
    }
    data['frame_id'] = this.frameId;
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
    final Map<String, dynamic> data = new Map<String, dynamic>();
    data['secs'] = this.secs;
    data['nsecs'] = this.nsecs;
    return data;
  }
}
