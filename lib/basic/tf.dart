import 'dart:math';
import 'package:ros_flutter_gui_app/basic/transform.dart';

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

  final RosTransform? transform;
  static const String transformKey = "transform";

  TransformElement copyWith({
    Header? header,
    String? childFrameId,
    RosTransform? transform,
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
          : RosTransform.fromJson(json["transform"]),
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
  static const String secsKey = "sec";

  final num nsecs;
  static const String nsecsKey = "nanosec";

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
      secs: json["sec"] ?? 0,
      nsecs: json["nanosec"] ?? 0,
    );
  }

  Map<String, dynamic> toJson() => {
        "sec": secs,
        "nanosec": nsecs,
      };

  @override
  String toString() {
    return "$secs, $nsecs, ";
  }
}
