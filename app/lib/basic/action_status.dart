enum ActionStatus {
  unknown(0),
  accepted(1),
  executing(2),
  canceling(3),
  succeeded(4),
  canceled(5),
  aborted(6);

  final int value;
  const ActionStatus(this.value);

  static ActionStatus fromValue(int value) {
    return ActionStatus.values.firstWhere(
      (status) => status.value == value,
      orElse: () => ActionStatus.unknown,
    );
  }

  @override
  String toString() {
    return name; // 直接返回枚举的名称，不包含前缀
  }
}

class GoalStatusArray {
  final List<GoalStatus> statusList;

  GoalStatusArray({required this.statusList});

  factory GoalStatusArray.fromJson(Map<String, dynamic> json) {
    return GoalStatusArray(
      statusList: (json['status_list'] as List)
          .map((e) => GoalStatus.fromJson(e))
          .toList(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'status_list': statusList.map((e) => e.toJson()).toList(),
    };
  }
}

class GoalStatus {
  final GoalInfo goalInfo;
  final ActionStatus status;

  GoalStatus({required this.goalInfo, required this.status});

  factory GoalStatus.fromJson(Map<String, dynamic> json) {
    return GoalStatus(
      goalInfo: GoalInfo.fromJson(json['goal_info']),
      status: ActionStatus.fromValue(json['status']),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'goal_info': goalInfo.toJson(),
      'status': status.value,
    };
  }
}

class GoalInfo {
  final GoalId goalId;
  final TimeStamp stamp;

  GoalInfo({required this.goalId, required this.stamp});

  factory GoalInfo.fromJson(Map<String, dynamic> json) {
    return GoalInfo(
      goalId: GoalId.fromJson(json['goal_id']),
      stamp: TimeStamp.fromJson(json['stamp']),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'goal_id': goalId.toJson(),
      'stamp': stamp.toJson(),
    };
  }
}

class GoalId {
  final String uuid;

  GoalId({required this.uuid});

  factory GoalId.fromJson(Map<String, dynamic> json) {
    return GoalId(
      uuid: json['uuid'],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'uuid': uuid,
    };
  }
}

class TimeStamp {
  final int sec;
  final int nanosec;

  TimeStamp({required this.sec, required this.nanosec});

  factory TimeStamp.fromJson(Map<String, dynamic> json) {
    return TimeStamp(
      sec: json['sec'],
      nanosec: json['nanosec'],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'sec': sec,
      'nanosec': nanosec,
    };
  }
}
