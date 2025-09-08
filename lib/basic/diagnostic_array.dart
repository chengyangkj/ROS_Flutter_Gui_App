import 'package:ros_flutter_gui_app/basic/diagnostic_status.dart';

class DiagnosticArray {
  Header? header;
  List<DiagnosticStatus> status;

  DiagnosticArray({
    this.header,
    this.status = const [],
  });

  factory DiagnosticArray.fromJson(Map<String, dynamic> json) {
    return DiagnosticArray(
      header: json['header'] != null ? Header.fromJson(json['header']) : null,
      status: (json['status'] as List<dynamic>?)
          ?.map((item) => DiagnosticStatus.fromJson(item))
          .toList() ?? [],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'header': header?.toJson(),
      'status': status.map((item) => item.toJson()).toList(),
    };
  }
}

class Header {
  int? seq;
  Map<String, dynamic>? stamp;
  String? frameId;

  Header({
    this.seq,
    this.stamp,
    this.frameId,
  });

  factory Header.fromJson(Map<String, dynamic> json) {
    return Header(
      seq: json['seq'],
      stamp: json['stamp'] != null ? Map<String, dynamic>.from(json['stamp']) : null,
      frameId: json['frame_id'],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'seq': seq,
      'stamp': stamp,
      'frame_id': frameId,
    };
  }
}