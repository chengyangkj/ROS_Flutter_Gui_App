import 'package:ros_flutter_gui_app/basic/transform.dart';

class RobotPath {
  Header? header;
  List<Poses>? poses;

  RobotPath({this.header, this.poses});

  RobotPath.fromJson(Map<String, dynamic> json) {
    header =
        json['header'] != null ? new Header.fromJson(json['header']) : null;
    if (json['poses'] != null) {
      poses = <Poses>[];
      json['poses'].forEach((v) {
        poses!.add(new Poses.fromJson(v));
      });
    }
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = new Map<String, dynamic>();
    if (this.header != null) {
      data['header'] = this.header!.toJson();
    }
    if (this.poses != null) {
      data['poses'] = this.poses!.map((v) => v.toJson()).toList();
    }
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

class Poses {
  Header? header;
  Pose? pose;

  Poses({this.header, this.pose});

  Poses.fromJson(Map<String, dynamic> json) {
    header =
        json['header'] != null ? new Header.fromJson(json['header']) : null;
    pose = json['pose'] != null ? new Pose.fromJson(json['pose']) : null;
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = new Map<String, dynamic>();
    if (this.header != null) {
      data['header'] = this.header!.toJson();
    }
    if (this.pose != null) {
      data['pose'] = this.pose!.toJson();
    }
    return data;
  }
}

class Pose {
  Position? position;
  Orientation? orientation;

  Pose({this.position, this.orientation});

  Pose.fromJson(Map<String, dynamic> json) {
    position = json['position'] != null
        ? new Position.fromJson(json['position'])
        : null;
    orientation = json['orientation'] != null
        ? new Orientation.fromJson(json['orientation'])
        : null;
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = new Map<String, dynamic>();
    if (this.position != null) {
      data['position'] = this.position!.toJson();
    }
    if (this.orientation != null) {
      data['orientation'] = this.orientation!.toJson();
    }
    return data;
  }
}

