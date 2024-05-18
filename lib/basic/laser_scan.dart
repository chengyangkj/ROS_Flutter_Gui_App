class LaserScan {
  Header? header;
  int? angleMin;
  double? angleMax;
  double? angleIncrement;
  int? timeIncrement;
  int? scanTime;
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
    header =
        json['header'] != null ? new Header.fromJson(json['header']) : null;
    angleMin = json['angle_min'];
    angleMax = json['angle_max'];
    angleIncrement = json['angle_increment'];
    timeIncrement = json['time_increment'];
    scanTime = json['scan_time'];
    rangeMin = json['range_min'];
    rangeMax = json['range_max'];
    ranges = (json['ranges'] as List<dynamic>).map((e) {
      // print("e:${e.runtimeType}");
      if (e == null) -1.toDouble();
      if (e is double) return e.toDouble();
      return -1.toDouble();
    }).toList();

    intensities = json['intensities'].cast<int>();
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
