import 'dart:math';
import 'RobotPose.dart';

class TF {
  TF({
    required this.transforms,
  });

  final List<TransformElement> transforms;
  static const String transformsKey = "transforms";

  TF copyWith({
    List<TransformElement>? transforms,
  }) {
    return TF(
      transforms: transforms ?? this.transforms,
    );
  }

  factory TF.fromJson(Map<String, dynamic> json) {
    return TF(
      transforms: json["transforms"] == null
          ? []
          : List<TransformElement>.from(
              json["transforms"]!.map((x) => TransformElement.fromJson(x))),
    );
  }

  Map<String, dynamic> toJson() => {
        "transforms": transforms.map((x) => x?.toJson()).toList(),
      };
}

class TransformElement {
  TransformElement({
    required this.header,
    required this.childFrameId,
    required this.transform,
  });

  final Header? header;
  static const String headerKey = "header";

  final String childFrameId;
  static const String childFrameIdKey = "child_frame_id";

  final Transform? transform;
  static const String transformKey = "transform";

  TransformElement copyWith({
    Header? header,
    String? childFrameId,
    Transform? transform,
  }) {
    return TransformElement(
      header: header ?? this.header,
      childFrameId: childFrameId ?? this.childFrameId,
      transform: transform ?? this.transform,
    );
  }

  factory TransformElement.fromJson(Map<String, dynamic> json) {
    return TransformElement(
      header: json["header"] == null ? null : Header.fromJson(json["header"]),
      childFrameId: json["child_frame_id"] ?? "",
      transform: json["transform"] == null
          ? null
          : Transform.fromJson(json["transform"]),
    );
  }

  Map<String, dynamic> toJson() => {
        "header": header?.toJson(),
        "child_frame_id": childFrameId,
        "transform": transform?.toJson(),
      };

  @override
  String toString() {
    return "$header, $childFrameId, $transform, ";
  }

  @override
  bool operator ==(Object other) =>
      identical(this, other) ||
      other is TransformElement &&
          runtimeType == other.runtimeType &&
          childFrameId == other.childFrameId;

  @override
  int get hashCode =>
      header.hashCode ^ childFrameId.hashCode ^ transform.hashCode;
}

class Header {
  Header({
    required this.seq,
    required this.stamp,
    required this.frameId,
  });

  final num seq;
  static const String seqKey = "seq";

  final Stamp? stamp;
  static const String stampKey = "stamp";

  final String frameId;
  static const String frameIdKey = "frame_id";

  Header copyWith({
    num? seq,
    Stamp? stamp,
    String? frameId,
  }) {
    return Header(
      seq: seq ?? this.seq,
      stamp: stamp ?? this.stamp,
      frameId: frameId ?? this.frameId,
    );
  }

  factory Header.fromJson(Map<String, dynamic> json) {
    return Header(
      seq: json["seq"] ?? 0,
      stamp: json["stamp"] == null ? null : Stamp.fromJson(json["stamp"]),
      frameId: json["frame_id"] ?? "",
    );
  }

  Map<String, dynamic> toJson() => {
        "seq": seq,
        "stamp": stamp?.toJson(),
        "frame_id": frameId,
      };

  @override
  String toString() {
    return "$seq, $stamp, $frameId, ";
  }
}

class Stamp {
  Stamp({
    required this.secs,
    required this.nsecs,
  });

  final num secs;
  static const String secsKey = "secs";

  final num nsecs;
  static const String nsecsKey = "nsecs";

  Stamp copyWith({
    num? secs,
    num? nsecs,
  }) {
    return Stamp(
      secs: secs ?? this.secs,
      nsecs: nsecs ?? this.nsecs,
    );
  }

  factory Stamp.fromJson(Map<String, dynamic> json) {
    return Stamp(
      secs: json["secs"] ?? 0,
      nsecs: json["nsecs"] ?? 0,
    );
  }

  Map<String, dynamic> toJson() => {
        "secs": secs,
        "nsecs": nsecs,
      };

  @override
  String toString() {
    return "$secs, $nsecs, ";
  }
}

class Transform {
  Transform({
    required this.translation,
    required this.rotation,
  });

  final Ation? translation;
  static const String translationKey = "translation";

  final Ation? rotation;
  static const String rotationKey = "rotation";

  RobotPose getRobotPose() {
    return RobotPose(
        getXYZ()[0].toDouble(), getXYZ()[1].toDouble(), getRPY()[2].toDouble());
  }

  List<num> getXYZ() {
    return [translation!.x, translation!.y, translation!.z];
  }

  List<num> getRPY() {
    num rx = rotation!.x;
    num ry = rotation!.y;
    num rz = rotation!.z;
    num rw = rotation!.w;

    // Calculate Euler angles from quaternion
    double roll = atan2(2 * (rw * rx + ry * rz), 1 - 2 * (rx * rx + ry * ry));
    double pitch = asin(2 * (rw * ry - rz * rx));
    double yaw = atan2(2 * (rw * rz + rx * ry), 1 - 2 * (ry * ry + rz * rz));

    return [roll, pitch, yaw];
  }

  Transform copyWith({
    Ation? translation,
    Ation? rotation,
  }) {
    return Transform(
      translation: translation ?? this.translation,
      rotation: rotation ?? this.rotation,
    );
  }

  factory Transform.fromJson(Map<String, dynamic> json) {
    return Transform(
      translation: json["translation"] == null
          ? null
          : Ation.fromJson(json["translation"]),
      rotation:
          json["rotation"] == null ? null : Ation.fromJson(json["rotation"]),
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

class Ation {
  Ation({
    required this.x,
    required this.y,
    required this.z,
    required this.w,
  });

  final num x;
  static const String xKey = "x";

  final num y;
  static const String yKey = "y";

  final num z;
  static const String zKey = "z";

  final num w;
  static const String wKey = "w";

  Ation copyWith({
    num? x,
    num? y,
    num? z,
    num? w,
  }) {
    return Ation(
      x: x ?? this.x,
      y: y ?? this.y,
      z: z ?? this.z,
      w: w ?? this.w,
    );
  }

  factory Ation.fromJson(Map<String, dynamic> json) {
    return Ation(
      x: json["x"] ?? 0,
      y: json["y"] ?? 0,
      z: json["z"] ?? 0,
      w: json["w"] ?? 0,
    );
  }

  Map<String, dynamic> toJson() => {
        "x": x,
        "y": y,
        "z": z,
        "w": w,
      };

  @override
  String toString() {
    return "$x, $y, $z, $w, ";
  }
}
