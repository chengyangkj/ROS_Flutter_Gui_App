import 'dart:async';
import 'package:roslibdart/roslibdart.dart';

class TopicWithSchemaName {
  String name;
  String schemaName;
  TopicWithSchemaName({required this.name, required this.schemaName});
}

class SubscribePayload {
  String topic;
  String type;
  Function(Map<String, dynamic> message)? callback;
  SubscribePayload({required this.topic, required this.type, this.callback});
}

class AdvertiseOptions {
  String topic;
  String schemaName;
  AdvertiseOptions({required this.topic, required this.schemaName});
}

class TopicStats {
  int numMessages;
  TopicStats({required this.numMessages});
}

enum PlayerPresence {
  notPresent,
  initializing,
  present,
  reconnecting,
}

class RosBridgePlayer {
  late Ros ros;
  String url;
  String id;
  bool closed = false;
  List<TopicWithSchemaName>? providerTopics;
  Map<String, TopicStats> providerTopicsStats = {};
  Map<String, dynamic>? providerDatatypes;
  Map<String, Topic> topicSubscriptions = {};
  List<SubscribePayload> requestedSubscriptions = [];
  Timer? requestTopicsTimeout;
  Map<String, Topic> topicPublishers = {};
  List<AdvertiseOptions> advertisements = [];
  int receivedBytes = 0;
  PlayerPresence presence = PlayerPresence.notPresent;
  Map<String, Future<String>> serviceTypeCache = {};
  int rosVersion = 1;
  DateTime? start;
  DateTime? clockTime;
  
  // 回调函数
  Function(PlayerPresence presence, List<TopicWithSchemaName>? topics, 
           Map<String, dynamic>? datatypes)? listener;

  RosBridgePlayer({
    required this.url,
    required this.id,
  }) {
    presence = PlayerPresence.initializing;
    start = DateTime.now();
    _open();
  }

  void _open() {
    if (closed) return;
    
    ros = Ros(url: url);
    
    // 设置状态监听器
    ros.statusStream.listen(
      (Status status) {
        if (status == Status.connected) {
          print("连接到 $url");
          if (closed) return;
          presence = PlayerPresence.present;
          _setupPublishers();
          requestTopics(forceUpdate: true);
        } else if (status == Status.errored) {
          presence = PlayerPresence.reconnecting;
          _handleConnectionError();
        } else if (status == Status.closed) {
          presence = PlayerPresence.reconnecting;
          _handleConnectionClose();
        }
        _emitState();
      },
      onError: (error) {
        print("ROS连接错误: $error");
        presence = PlayerPresence.reconnecting;
        _emitState();
      },
    );
    
    // 尝试连接
    ros.connect().then((error) {
      if (error.isNotEmpty) {
        print("连接失败: $error");
        presence = PlayerPresence.reconnecting;
        _emitState();
      }
    });
  }

  void _handleConnectionError() {
    _emitState();
    // 3秒后重连
    Timer(const Duration(seconds: 3), () {
      if (!closed) {
        _open();
      }
    });
  }

  void _handleConnectionClose() {
    if (requestTopicsTimeout != null) {
      requestTopicsTimeout!.cancel();
    }
    
    for (final entry in topicSubscriptions.entries) {
      entry.value.unsubscribe();
    }
    topicSubscriptions.clear();
    
    ros.close();
    ros = Ros(url: url);
    
    _emitState();
    
    // 3秒后重连
    Timer(const Duration(seconds: 3), () {
      if (!closed) {
        _open();
      }
    });
  }

  bool _topicsChanged(List<TopicWithSchemaName> newTopics) {
    if (providerTopics == null || newTopics.length != providerTopics!.length) {
      return true;
    }
    
    for (int i = 0; i < providerTopics!.length; i++) {
      if (providerTopics![i].name != newTopics[i].name ||
          providerTopics![i].schemaName != newTopics[i].schemaName) {
        return true;
      }
    }
    return false;
  }

  Future<void> requestTopics({bool forceUpdate = false}) async {
    if (requestTopicsTimeout != null) {
      requestTopicsTimeout!.cancel();
    }
    
    if (ros.status != Status.connected || closed) {
      return;
    }

    try {
      final result = await ros.getTopicsAndRawTypes();
      
      final topics = <String>[];
      final types = <String>[];
      final typedefsFullText = <String>[];
      
      if (result['topics'] != null) {
        topics.addAll(List<String>.from(result['topics']));
      }
      if (result['types'] != null) {
        types.addAll(List<String>.from(result['types']));
      }
      if (result['typedefs_full_text'] != null) {
        typedefsFullText.addAll(List<String>.from(result['typedefs_full_text']));
      }

      final topicsMissingDatatypes = <String>[];
      final topicsWithSchema = <TopicWithSchemaName>[];
      final datatypeDescriptions = <Map<String, dynamic>>[];

      // 自动检测ROS版本
      if (types.contains("rcl_interfaces/msg/Log")) {
        rosVersion = 2;
      } else if (types.contains("rosgraph_msgs/Log")) {
        rosVersion = 1;
      } else {
        rosVersion = 1;
      }

      for (int i = 0; i < topics.length; i++) {
        final topicName = topics[i];
        final type = i < types.length ? types[i] : null;
        final messageDefinition = i < typedefsFullText.length ? typedefsFullText[i] : null;

        if (type == null || messageDefinition == null) {
          topicsMissingDatatypes.add(topicName);
          continue;
        }
        
        topicsWithSchema.add(TopicWithSchemaName(name: topicName, schemaName: type));
        datatypeDescriptions.add({
          'type': type,
          'messageDefinition': messageDefinition,
        });
      }

      // 检查topics是否有变化
      final sortedTopics = List<TopicWithSchemaName>.from(topicsWithSchema);
      sortedTopics.sort((a, b) => a.name.compareTo(b.name));
      
      if (!forceUpdate && !_topicsChanged(sortedTopics)) {
        return;
      }

      // 移除已删除topics的统计信息
      final topicsSet = <String>{};
      for (final topic in sortedTopics) {
        topicsSet.add(topic.name);
      }
      final topicsToRemove = <String>[];
      for (final topic in providerTopicsStats.keys) {
        if (!topicsSet.contains(topic)) {
          topicsToRemove.add(topic);
        }
      }
      for (final topic in topicsToRemove) {
        providerTopicsStats.remove(topic);
      }

      providerTopics = sortedTopics;
      providerDatatypes = _bagConnectionsToDatatypes(datatypeDescriptions, ros2: rosVersion == 2);

      // 重新订阅
      _resubscribeTopics();
      
    } catch (error) {
      print("获取topics失败: $error");
    } finally {
      _emitState();
      
      // 3秒后再次请求topics
      requestTopicsTimeout = Timer(const Duration(seconds: 3), () {
        requestTopics();
      });
    }
  }

  void _subscribeSingleTopic(SubscribePayload subscription) {
    if (ros.status != Status.connected || closed) {
      print("ROS未连接或已关闭 订阅topic失败: ${subscription.topic}");
      return;
    }

    final topicName = subscription.topic;

    // 如果已经订阅，先取消订阅
    if (topicSubscriptions.containsKey(topicName)) {
      topicSubscriptions[topicName]!.unsubscribe();
      topicSubscriptions.remove(topicName);
    }
    
    final topic = Topic(
      ros: ros,
      name: topicName,
      type: subscription.type,
      queueSize: 1,
      reconnectOnClose: true,
    );

    topic.subscribe((message) async {
      try {
      
        // // 更新消息计数
        // var stats = providerTopicsStats[topicName];
        // if (stats == null) {
        //   stats = TopicStats(numMessages: 0);
        //   providerTopicsStats[topicName] = stats;
        // }
        // stats.numMessages++;

        // 调用用户回调
        if (subscription.callback != null) {
          subscription.callback!(message);
        }
      } catch (error) {
        print("解析消息失败 $topicName: $error");
      }
    });
    
    topicSubscriptions[topicName] = topic;
  }

  void _resubscribeTopics() {
    if (ros.status != Status.connected || closed) {
      return;
    }

    // 订阅所有请求的topics
    for (final subscription in requestedSubscriptions) {
      // 复用_subscribeSingleTopic的逻辑
      _subscribeSingleTopic(subscription);
    }
  }

  void _emitState() {
    if (listener == null || closed) return;

    listener!(presence, providerTopics, providerDatatypes);
  }

  void setListener(Function(PlayerPresence presence, List<TopicWithSchemaName>? topics, 
                           Map<String, dynamic>? datatypes) listener) {
    this.listener = listener;
    _emitState();
  }

  void close() {
    closed = true;
    if (ros.status == Status.connected) {
      ros.close();
    }
  }

  // 订阅topic
  void subscribe(String topicName, String messageType, Function(Map<String, dynamic> message)? callback) {
    // 检查是否已经订阅
    if (requestedSubscriptions.any((sub) => sub.topic == topicName)) {
      return;
    }
    
    final subscription = SubscribePayload(topic: topicName, type: messageType, callback: callback);
    requestedSubscriptions.add(subscription);
    
    // 如果连接正常，立即订阅这个新的topic
    if (ros.status == Status.connected && !closed) {
      _subscribeSingleTopic(subscription);
    }
  }

  // 取消订阅topic
  void unsubscribe(String topicName) {
    requestedSubscriptions.removeWhere((sub) => sub.topic == topicName);
    
    if (topicSubscriptions.containsKey(topicName)) {
      topicSubscriptions[topicName]!.unsubscribe();
      topicSubscriptions.remove(topicName);
      providerTopicsStats.remove(topicName);
    }
  }

  // 发布消息
  void publish(String topicName, Map<String, dynamic> message) {
    final publisher = topicPublishers[topicName];
    if (publisher == null) {
      throw Exception("尝试发布到未注册的topic: $topicName");
    }
    publisher.publish(message);
  }

  // 注册发布者
  void advertise(String topicName, String messageType) {
    final advertisement = AdvertiseOptions(topic: topicName, schemaName: messageType);
    advertisements.add(advertisement);
    _setupPublishers();
  }

  // 取消发布
  void unadvertise(String topicName) {
    advertisements.removeWhere((adv) => adv.topic == topicName);
    
    if (topicPublishers.containsKey(topicName)) {
      topicPublishers[topicName]!.unadvertise();
      topicPublishers.remove(topicName);
    }
  }

  // 调用服务
  Future<Map<String, dynamic>> callService(String service, Map<String, dynamic> request) async {
    if (ros.status != Status.connected) {
      throw Exception("未连接");
    }

    final serviceType = await _getServiceType(service);
    
    final serviceObj = Service(
      ros: ros,
      name: service,
      type: serviceType,
    );

    return await serviceObj.call(request);
  }

  Future<String> _getServiceType(String service) async {
    if (ros.status != Status.connected) {
      throw Exception("未连接");
    }

    final existing = serviceTypeCache[service];
    if (existing != null) {
      return await existing;
    }

    final serviceTypeFuture = Future<String>(() async {
      // 这里需要实现获取服务类型的逻辑
      // 由于roslibdart可能没有直接的方法，我们返回一个默认值
      return "std_srvs/Empty";
    });

    serviceTypeCache[service] = serviceTypeFuture;
    return await serviceTypeFuture;
  }

  void _setupPublishers() {
    if (ros.status != Status.connected) {
      return;
    }

    if (advertisements.isEmpty) {
      return;
    }

    for (final advertisement in advertisements) {
      if (topicPublishers.containsKey(advertisement.topic)) {
        continue;
      }
      
      final topic = Topic(
        ros: ros,
        name: advertisement.topic,
        type: advertisement.schemaName,
        queueSize: 0,
        reconnectOnClose: true,
      );
      topicPublishers[advertisement.topic] = topic;
      topic.advertise();
    }
  }

  Map<String, dynamic> _bagConnectionsToDatatypes(List<Map<String, dynamic>> datatypeDescriptions, {bool ros2 = false}) {
    final datatypes = <String, dynamic>{};
    
    for (final description in datatypeDescriptions) {
      final type = description['type'] as String;
      final messageDefinition = description['messageDefinition'] as String;
      
      datatypes[type] = {
        'type': type,
        'messageDefinition': messageDefinition,
        'ros2': ros2,
      };
    }
    
    return datatypes;
  }

  // 获取ROS连接状态
  Status get connectionStatus => ros.status;
  
  // 获取当前topics
  List<TopicWithSchemaName> get topics => providerTopics ?? [];
  
  // 获取数据类型
  Map<String, dynamic> get datatypes => providerDatatypes ?? {};
  
  // 获取ROS版本
  int get currentRosVersion => rosVersion;
  
  // 手动触发topics请求
  void refreshTopics() {
    requestTopics(forceUpdate: true);
  }
}