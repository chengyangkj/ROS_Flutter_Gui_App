import 'dart:math';

import 'package:ros_flutter_gui_app/basic/RobotPose.dart';

class RosTransform {
  RosTransform({
    required this.translation,
    required this.rotation,
  });

  final Position? translation;
  static const String translationKey = "translation";

  final Orientation? rotation;
  static const String rotationKey = "rotation";

  RobotPose getRobotPose() {
    return RobotPose(
        getXYZ()[0].toDouble(), getXYZ()[1].toDouble(), getRPY()[2].toDouble());
  }

  List<double> getXYZ() {
    return [translation!.x!, translation!.y!, translation!.z!];
  }

  List<double> getRPY() {
    double rx = rotation!.x!;
    double ry = rotation!.y!;
    double rz = rotation!.z!;
    double rw = rotation!.w!;

    // Calculate Euler angles from quaternion
    double roll = atan2(2 * (rw * rx + ry * rz), 1 - 2 * (rx * rx + ry * ry));
    double pitch = asin(2 * (rw * ry - rz * rx));
    double yaw = atan2(2 * (rw * rz + rx * ry), 1 - 2 * (ry * ry + rz * rz));

    return [roll, pitch, yaw];
  }

  RosTransform copyWith({
    Position? translation,
    Orientation? rotation,
  }) {
    return RosTransform(
      translation: translation ?? this.translation,
      rotation: rotation ?? this.rotation,
    );
  }

  factory RosTransform.fromJson(Map<String, dynamic> json) {
    return RosTransform(
      translation: json["translation"] == null
          ? null
          : Position.fromJson(json["translation"]),
      rotation:
          json["rotation"] == null ? null : Orientation.fromJson(json["rotation"]),
    );
  }

  Map<String, dynamic> toJson() => {
        "translation": translation?.toJson(),
        "rotation": rotation?.toJson(),
      };

  @override
  String toString() {
    return "$translation, $rotation, ";
  }
}
class Position {
  double? x;
  double? y;
  double? z;

  Position({this.x, this.y, this.z});

  Position.fromJson(Map<String, dynamic> json) {
    x = json['x'];
    y = json['y'];
    z = json['z'];
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = new Map<String, dynamic>();
    data['x'] = this.x;
    data['y'] = this.y;
    data['z'] = this.z;
    return data;
  }
}

class Orientation {
  double? x;
  double? y;
  double? z;
  double? w;

  Orientation({this.x, this.y, this.z, this.w});

  Orientation.fromJson(Map<String, dynamic> json) {
    x = json['x'];
    y = json['y'];
    z = json['z'];
    w = json['w'];
  }

  Map<String, dynamic> toJson() {
    final Map<String, dynamic> data = new Map<String, dynamic>();
    data['x'] = this.x;
    data['y'] = this.y;
    data['z'] = this.z;
    data['w'] = this.w;
    return data;
  }
}
