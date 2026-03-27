import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:vector_math/vector_math_64.dart' as vm;
import 'package:oktoast/oktoast.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_array.dart' as darr;
import 'package:ros_flutter_gui_app/basic/diagnostic_status.dart' as dstatus;
import 'package:ros_flutter_gui_app/basic/key_value.dart';
import 'package:ros_flutter_gui_app/basic/math.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/basic/pointcloud2.dart';
import 'package:ros_flutter_gui_app/basic/topology_map.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/protobuf/geometry_msgs.pb.dart' as gpb;
import 'package:ros_flutter_gui_app/protobuf/nav_msgs.pb.dart' as npb;
import 'package:ros_flutter_gui_app/protobuf/robot_message.pb.dart';
import 'package:ros_flutter_gui_app/protobuf/sensor_msg.pb.dart' as sensor_msg;
import 'package:ros_flutter_gui_app/protobuf/diagnostic_msgs.pb.dart' as dipb;
import 'package:ros_flutter_gui_app/protobuf/action_msgs.pb.dart' as apb;
import 'package:ros_flutter_gui_app/provider/diagnostic_manager.dart';
import 'package:ros_flutter_gui_app/provider/http_channel.dart';
import 'package:ros_flutter_gui_app/provider/map_manager.dart';

class TopicWithSchemaName {
  final String name;
  final String schemaName;
  TopicWithSchemaName({required this.name, required this.schemaName});
}

class LaserData {
  RobotPose robotPose;
  List<vm.Vector2> laserPoseBaseLink;
  LaserData({required this.robotPose, required this.laserPoseBaseLink});
}

class RobotSpeed {
  double vx;
  double vy;
  double vw;
  RobotSpeed({required this.vx, required this.vy, required this.vw});
}

class WsChannel {
  final HttpChannel _http = HttpChannel();
  WebSocketChannel? _wsChannel;
  StreamSubscription<dynamic>? _wsSub;

  Timer? cmdVelTimer;
  bool isReconnect_ = false;

  bool manualCtrlMode_ = false;
  ValueNotifier<double> battery_ = ValueNotifier(78);
  ValueNotifier<Uint8List> imageData = ValueNotifier(Uint8List(0));
  ValueNotifier<Uint8List?> imageFrame = ValueNotifier<Uint8List?>(null);
  RobotSpeed cmdVel_ = RobotSpeed(vx: 0, vy: 0, vw: 0);
  ValueNotifier<RobotSpeed> robotSpeed_ =
      ValueNotifier(RobotSpeed(vx: 0, vy: 0, vw: 0));
  String backendHost_ = "";
  int backendHttpPort_ = 7684;
  Status rosConnectState_ = Status.none;
  String? _streamError;
  String? _activeImageTopicRaw;
  String? _activeImageTopicNormalized;
  bool imageStreamStarting_ = false;
  int? _lastImagePushMs;

  ValueNotifier<OccupancyMap> get map_ => mapManager.occupancyMap;
  ValueNotifier<TopologyMap> get topologyMap_ => mapManager.topologyMap;
  ValueNotifier<RobotPose> robotPoseMap = ValueNotifier(RobotPose.zero());
  ValueNotifier<List<vm.Vector2>> laserBasePoint_ = ValueNotifier([]);
  ValueNotifier<List<vm.Vector2>> localPath = ValueNotifier([]);
  ValueNotifier<List<vm.Vector2>> globalPath = ValueNotifier([]);
  ValueNotifier<List<vm.Vector2>> tracePath = ValueNotifier([]);
  ValueNotifier<LaserData> laserPointData = ValueNotifier(
      LaserData(robotPose: RobotPose(0, 0, 0), laserPoseBaseLink: []));
  ValueNotifier<ActionStatus> navStatus_ = ValueNotifier(ActionStatus.unknown);
  ValueNotifier<List<vm.Vector2>> robotFootprint = ValueNotifier([]);
  ValueNotifier<OccupancyMap> localCostmap = ValueNotifier(OccupancyMap());
  ValueNotifier<OccupancyMap> globalCostmap = ValueNotifier(OccupancyMap());
  ValueNotifier<List<Point3D>> pointCloud2Data = ValueNotifier([]);
  ValueNotifier<darr.DiagnosticArray> diagnosticData =
      ValueNotifier(darr.DiagnosticArray());
  late DiagnosticManager diagnosticManager;
  late MapManager mapManager;

  WsChannel() {
    diagnosticManager = DiagnosticManager();
    mapManager = MapManager();
    mapManager.init();

    globalSetting.init().then((success) {
      Timer.periodic(const Duration(seconds: 5), (timer) async {
        if (isReconnect_ && rosConnectState_ != Status.connected) {
          showToast(
            "lost connection to $backendHost_:$backendHttpPort_ try reconnect...",
            position: ToastPosition.bottom,
            backgroundColor: Colors.black.withValues(alpha: 0.8),
            textStyle: const TextStyle(color: Colors.white),
          );
          final error = await connectBackend(backendHost_, backendHttpPort_);
          if (error.isEmpty) {
            showToast(
              "reconnect success!",
              position: ToastPosition.bottom,
              backgroundColor: Colors.green.withValues(alpha: 0.8),
              textStyle: const TextStyle(color: Colors.white),
            );
          } else {
            showToast(
              "reconnect failed: $error",
              position: ToastPosition.bottom,
              backgroundColor: Colors.red.withValues(alpha: 0.8),
              textStyle: const TextStyle(color: Colors.white),
            );
          }
        }
      });
    });
  }

  String? get streamError => _streamError;
  bool get imageStreamStarting => imageStreamStarting_;

  static String normalizeTopic(String t) {
    if (t.isEmpty) return t;
    return t.startsWith('/') ? t : '/$t';
  }

  static String _wsAuthority(String host, int port) {
    final h = host.trim();
    if (h.contains(':') && !h.startsWith('[')) {
      return '[$h]:$port';
    }
    return '$h:$port';
  }

  static Uri robotWebSocketUri(String host, int port) {
    final pageHttps = kIsWeb && Uri.base.scheme == 'https';
    final scheme = pageHttps ? 'wss' : 'ws';
    final authority = _wsAuthority(host, port);
    return Uri.parse('$scheme://$authority/ws/robot');
  }

  Future<String> connectBackend(String host, int httpPort) async {
    backendHost_ = host.trim();
    backendHttpPort_ = httpPort;
    globalSetting.setRobotIp(host);
    globalSetting.setHttpServerPort(httpPort.toString());
    rosConnectState_ = Status.connecting;

    await _teardownWs();

    try {
      final settings = await _http.getGuiSettings();
      globalSetting.applyBackendGuiSettings(settings);
    } catch (e) {
      rosConnectState_ = Status.none;
      return 'settings: $e';
    }

    if (kIsWeb) {
      final ping = Uri.parse('http://${_wsAuthority(host, httpPort)}/tiles/');
      try {
        final r = await http.get(ping).timeout(const Duration(seconds: 5));
        if (r.statusCode >= 500) {
          rosConnectState_ = Status.none;
          return 'HTTP ${r.statusCode}';
        }
      } catch (e) {
        rosConnectState_ = Status.none;
        return 'HTTP unreachable: $e';
      }
    }

    final wsErr = await _attachRobotWebSocket(host, httpPort);
    if (wsErr.isNotEmpty) {
      rosConnectState_ = Status.none;
      return wsErr;
    }

    rosConnectState_ = Status.connected;
    if (!isReconnect_) {
      isReconnect_ = true;
    }
    return '';
  }

  Future<void> _teardownWs() async {
    await _wsSub?.cancel();
    _wsSub = null;
    try {
      await _wsChannel?.sink.close();
    } catch (_) {}
    _wsChannel = null;
  }

  Future<String> _attachRobotWebSocket(String host, int httpPort) async {
    final uri = robotWebSocketUri(host, httpPort);
    debugPrint('backend ws $uri');
    _streamError = null;
    _wsChannel = WebSocketChannel.connect(uri);
    _wsSub = _wsChannel!.stream.listen(
      (dynamic data) => scheduleMicrotask(() => _onWsBinary(data)),
      onError: (Object e) {
        _streamError = e.toString();
        rosConnectState_ = Status.errored;
        scheduleMicrotask(() async {
          await _teardownWs();
        });
      },
      onDone: () {
        if (rosConnectState_ == Status.connected) {
          rosConnectState_ = Status.errored;
        }
      },
      cancelOnError: true,
    );
    try {
      await _wsChannel!.ready.timeout(const Duration(seconds: 15));
    } on TimeoutException {
      await _teardownWs();
      return 'ws handshake timeout';
    } catch (e) {
      await _teardownWs();
      return 'ws: $e';
    }
    return '';
  }

  void _onWsBinary(dynamic data) {
    Uint8List? raw;
    if (data is Uint8List) {
      raw = data;
    } else if (data is List<int>) {
      raw = Uint8List.fromList(data);
    }
    if (raw == null) return;

    final RobotMessage msg;
    try {
      msg = RobotMessage.fromBuffer(raw);
    } catch (_) {
      return;
    }

    switch (msg.whichPayload()) {
      case RobotMessage_Payload.image:
        _onStreamImage(msg.image);
        break;
      case RobotMessage_Payload.heartbeat:
        break;
      case RobotMessage_Payload.laserScanMap:
        _onLaserScanMap(msg.laserScanMap);
        break;
      case RobotMessage_Payload.robotPoseMap:
        _onRobotPoseMap(msg.robotPoseMap);
        break;
      case RobotMessage_Payload.pathLocal:
        _onPathPb(msg.pathLocal, localPath);
        break;
      case RobotMessage_Payload.pathGlobal:
        _onPathPb(msg.pathGlobal, globalPath);
        break;
      case RobotMessage_Payload.pathTrace:
        _onPathPb(msg.pathTrace, tracePath);
        break;
      case RobotMessage_Payload.odometry:
        _onOdometryPb(msg.odometry);
        break;
      case RobotMessage_Payload.battery:
        _onBatteryPb(msg.battery);
        break;
      case RobotMessage_Payload.footprint:
        _onFootprintPb(msg.footprint);
        break;
      case RobotMessage_Payload.localCostmap:
        localCostmap.value = _occupancyFromProto(msg.localCostmap);
        break;
      case RobotMessage_Payload.globalCostmap:
        globalCostmap.value = _occupancyFromProto(msg.globalCostmap);
        break;
      case RobotMessage_Payload.pointcloudMap:
        _onPointcloudMap(msg.pointcloudMap);
        break;
      case RobotMessage_Payload.diagnostic:
        _onDiagnosticPb(msg.diagnostic);
        break;
      case RobotMessage_Payload.navStatus:
        _onNavStatusPb(msg.navStatus);
        break;
      case RobotMessage_Payload.dynamicLayer:
        _onDynamicLayer(msg.dynamicLayer);
        break;
      case RobotMessage_Payload.transformLookupResponse:
      case RobotMessage_Payload.notSet:
        break;
    }
  }

  ValueNotifier<List<vm.Vector2>>? _pathNotifierForLayerId(String id) {
    switch (id) {
      case 'globalPath':
        return globalPath;
      case 'localPath':
        return localPath;
      case 'tracePath':
        return tracePath;
      default:
        return null;
    }
  }

  void _onDynamicLayer(DynamicLayerStream dl) {
    final id = dl.layerId;
    switch (dl.whichStream()) {
      case DynamicLayerStream_Stream.laserScan:
        _onLaserScanMap(dl.laserScan);
        break;
      case DynamicLayerStream_Stream.path:
        final out = _pathNotifierForLayerId(id);
        if (out != null) {
          _onPathPb(dl.path, out);
        }
        break;
      case DynamicLayerStream_Stream.costmap:
        final occ = _occupancyFromProto(dl.costmap);
        if (id == 'globalCostmap') {
          globalCostmap.value = occ;
        } else if (id == 'localCostmap' || id.isEmpty) {
          localCostmap.value = occ;
        }
        break;
      case DynamicLayerStream_Stream.pointcloud:
        _onPointcloudMap(dl.pointcloud);
        break;
      case DynamicLayerStream_Stream.footprint:
        _onFootprintPb(dl.footprint);
        break;
      case DynamicLayerStream_Stream.notSet:
        break;
    }
  }

  void _onStreamImage(ImageFrame img) {
    final topic = normalizeTopic(img.topic);
    if (topic != _activeImageTopicNormalized) return;
    if (img.data.isEmpty) return;
    final now = DateTime.now().millisecondsSinceEpoch;
    if (_lastImagePushMs != null && now - _lastImagePushMs! < 66) return;
    _lastImagePushMs = now;
    final u8 = Uint8List.fromList(img.data);
    imageFrame.value = u8;
    imageData.value = u8;
  }

  void _onLaserScanMap(sensor_msg.LaserScanMapFrame lm) {
    final pose = robotPoseMap.value;
    final n = lm.x.length < lm.y.length ? lm.x.length : lm.y.length;
    final pts = <vm.Vector2>[];
    for (var i = 0; i < n; i++) {
      pts.add(vm.Vector2(lm.x[i].toDouble(), lm.y[i].toDouble()));
    }
    laserBasePoint_.value = pts;
    laserPointData.value = LaserData(robotPose: pose, laserPoseBaseLink: pts);
  }

  RobotPose _poseFromGpbPose(gpb.Pose p) {
    final o = p.orientation;
    final q = vm.Quaternion(o.x, o.y, o.z, o.w);
    final euler = quaternionToEuler(q);
    final yaw = euler[0];
    return RobotPose(p.position.x, p.position.y, yaw);
  }

  void _onRobotPoseMap(gpb.PoseStamped ps) {
    robotPoseMap.value = _poseFromGpbPose(ps.pose);
  }

  void _onPathPb(npb.Path path, ValueNotifier<List<vm.Vector2>> out) {
    final pts = <vm.Vector2>[];
    for (final p in path.poses) {
      final pose = p.pose;
      pts.add(vm.Vector2(pose.position.x, pose.position.y));
    }
    out.value = pts;
  }

  void _onOdometryPb(npb.Odometry o) {
    final lin = o.twist.linear;
    final ang = o.twist.angular;
    robotSpeed_.value =
        RobotSpeed(vx: lin.x, vy: lin.y, vw: ang.z);
  }

  void _onBatteryPb(sensor_msg.BatteryState b) {
    final p = b.percentage;
    battery_.value = p > 0 ? p.toDouble() : battery_.value;
  }

  void _onFootprintPb(gpb.PolygonStamped poly) {
    final pts = <vm.Vector2>[];
    for (final p in poly.polygon.points) {
      pts.add(vm.Vector2(p.x, p.y));
    }
    robotFootprint.value = pts;
  }

  OccupancyMap _occupancyFromProto(npb.OccupancyGridProto g) {
    final info = g.info;
    final origin = info.origin;
    final pos = origin.position;
    final orient = origin.orientation;
    final q = vm.Quaternion(orient.x, orient.y, orient.z, orient.w);
    final euler = quaternionToEuler(q);
    final occ = OccupancyMap();
    occ.mapConfig.resolution = info.resolution.toDouble();
    occ.mapConfig.width = info.width;
    occ.mapConfig.height = info.height;
    occ.mapConfig.originX = pos.x;
    occ.mapConfig.originY = pos.y;
    occ.mapConfig.originTheta = euler[0];
    final w = info.width;
    final h = info.height;
    final dataList = g.data;
    occ.data = List.generate(
      h,
      (i) => List.generate(w, (j) => 0),
    );
    for (var i = 0; i < dataList.length; i++) {
      final x = i ~/ w;
      final y = i % w;
      occ.data[x][y] = dataList[i];
    }
    occ.setFlip();
    return occ;
  }

  void _onPointcloudMap(sensor_msg.PointCloud2MapFrame pc) {
    final n = pc.x.length;
    final ny = pc.y.length;
    final nz = pc.z.length;
    final m = n < ny ? n : ny;
    final out = <Point3D>[];
    for (var i = 0; i < m; i++) {
      final z = i < nz ? pc.z[i] : 0.0;
      out.add(Point3D(pc.x[i], pc.y[i], z));
    }
    pointCloud2Data.value = out;
  }

  void _onDiagnosticPb(dipb.DiagnosticArray arr) {
    final list = <dstatus.DiagnosticStatus>[];
    for (final s in arr.status) {
      final vals = <KeyValue>[];
      for (final kv in s.values) {
        vals.add(KeyValue(key: kv.key, value: kv.value));
      }
      list.add(dstatus.DiagnosticStatus(
        level: s.level,
        name: s.name,
        message: s.message,
        hardwareId: s.hardwareId,
        values: vals,
      ));
    }
    final header = arr.hasHeader()
        ? darr.Header(frameId: arr.header.frameId)
        : null;
    diagnosticData.value =
        darr.DiagnosticArray(header: header, status: list);
    diagnosticManager.updateDiagnosticStates(diagnosticData.value);
  }

  void _onNavStatusPb(apb.GoalStatusArray arr) {
    if (arr.statusList.isEmpty) return;
    final last = arr.statusList.last;
    navStatus_.value = ActionStatus.fromValue(last.status);
  }

  Future<void> beginImageStream(String topic) async {
    imageStreamStarting_ = true;
    _streamError = null;
    _lastImagePushMs = null;
    imageFrame.value = null;
    final prev = _activeImageTopicRaw;
    if (prev != null && prev.isNotEmpty && prev != topic) {
      try {
        await _http.postSubImage(prev, false);
      } catch (_) {}
    }
    _activeImageTopicRaw = null;
    _activeImageTopicNormalized = null;
    try {
      await _http.postSubImage(topic, true);
      _activeImageTopicRaw = topic;
      _activeImageTopicNormalized = normalizeTopic(topic);
    } catch (e) {
      _streamError = '$e';
    }
    imageStreamStarting_ = false;
  }

  Future<void> endImageStream() async {
    _lastImagePushMs = null;
    imageFrame.value = null;
    imageStreamStarting_ = false;
    final raw = _activeImageTopicRaw;
    _activeImageTopicRaw = null;
    _activeImageTopicNormalized = null;
    if (raw != null && raw.isNotEmpty) {
      try {
        await _http.postSubImage(raw, false);
      } catch (_) {}
    }
  }

  Future<String> connect(String url) async {
    final u = Uri.parse(url);
    final host = u.host.isNotEmpty ? u.host : backendHost_;
    final port = u.port > 0 ? u.port : backendHttpPort_;
    return connectBackend(host, port);
  }

  void closeConnection() {
    robotFootprint.value = [];
    laserBasePoint_.value = [];
    localPath.value = [];
    globalPath.value = [];
    tracePath.value = [];
    laserPointData.value.laserPoseBaseLink.clear();
    laserPointData.value.robotPose = RobotPose.zero();
    robotSpeed_.value = RobotSpeed(vx: 0, vy: 0, vw: 0);
    robotPoseMap.value = RobotPose.zero();
    navStatus_.value = ActionStatus.unknown;
    battery_.value = 0;
    imageData.value = Uint8List(0);
    imageFrame.value = null;
    pointCloud2Data.value = [];
    localCostmap.value = OccupancyMap();
    globalCostmap.value = OccupancyMap();
    diagnosticData.value = darr.DiagnosticArray();
    cmdVel_.vx = 0;
    cmdVel_.vy = 0;
    cmdVel_.vw = 0;
    unawaited(endImageStream());
    unawaited(_teardownWs());
    rosConnectState_ = Status.none;
  }

  ValueNotifier<OccupancyMap> get map => map_;

  List<TopicWithSchemaName> get topics => [];
  Map<String, dynamic> get datatypes => {};
  int get currentRosVersion => 2;

  void refreshTopics() {}

  Future<Map<String, dynamic>> sendTopologyGoal(String name) async {
    debugPrint('sendTopologyGoal not supported on backend: $name');
    return {'is_success': false, 'message': 'not_supported'};
  }

  Future<void> sendNavigationGoal(RobotPose pose) async {
    await _http.postRobotNavGoal(pose.x, pose.y, pose.theta);
  }

  Future<void> sendEmergencyStop() async {
    await sendSpeed(0, 0, 0);
  }

  Future<void> sendCancelNav() async {
    await _http.postRobotCancelNav();
  }

  void destroyConnection() {
    closeConnection();
  }

  void setVx(double vx) => cmdVel_.vx = vx;
  void setVy(double vy) => cmdVel_.vy = vy;
  void setVw(double vw) => cmdVel_.vw = vw;

  void startMunalCtrl() {
    cmdVelTimer =
        Timer.periodic(const Duration(milliseconds: 100), (timer) async {
      await sendSpeed(cmdVel_.vx, cmdVel_.vy, cmdVel_.vw);
    });
  }

  void stopMunalCtrl() {
    cmdVelTimer?.cancel();
    cmdVelTimer = null;
    cmdVel_.vx = 0;
    cmdVel_.vy = 0;
    cmdVel_.vw = 0;
    sendSpeed(0, 0, 0);
  }

  Future<void> sendSpeed(double vx, double vy, double vw) async {
    await _http.postRobotCmdVel(vx, vy, vw);
  }

  Future<void> sendRelocPose(RobotPose pose) async {
    await _http.postRobotInitialPose(pose.x, pose.y, pose.theta);
  }

  Future<void> updateTopologyMap(TopologyMap updatedMap) async {
    mapManager.updateTopologyMap(updatedMap);
  }
}

enum Status { none, connecting, connected, errored }
