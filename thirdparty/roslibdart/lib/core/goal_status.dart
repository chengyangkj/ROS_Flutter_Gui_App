// Copyright (c) 2019 Conrad Heidebrecht.

/// Goal status enums for ROS actions.
enum GoalStatus {
  /// The goal has been accepted and is awaiting execution.
  pending,
  /// The goal is currently being executed by the action server.
  active,
  /// The goal was preempted by a user request.
  preempted,
  /// The goal was achieved successfully by the action server.
  succeeded,
  /// The goal was aborted during execution by the action server due to some failure.
  aborted,
  /// The goal was rejected by the action server without being processed.
  rejected,
  /// The goal received a cancel request after it started executing and has since completed its execution.
  canceled,
  /// The goal was canceled by a user request.
  canceledByUser,
}

/// Extension to provide status codes for GoalStatus.
extension GoalStatusExtension on GoalStatus {
  /// Get the numeric status code for the goal status.
  int get statusCode {
    switch (this) {
      case GoalStatus.pending:
        return 0;
      case GoalStatus.active:
        return 1;
      case GoalStatus.preempted:
        return 2;
      case GoalStatus.succeeded:
        return 3;
      case GoalStatus.aborted:
        return 4;
      case GoalStatus.rejected:
        return 5;
      case GoalStatus.canceled:
        return 6;
      case GoalStatus.canceledByUser:
        return 7;
    }
  }

  /// Get the string representation of the goal status.
  String get statusString {
    switch (this) {
      case GoalStatus.pending:
        return 'PENDING';
      case GoalStatus.active:
        return 'ACTIVE';
      case GoalStatus.preempted:
        return 'PREEMPTED';
      case GoalStatus.succeeded:
        return 'SUCCEEDED';
      case GoalStatus.aborted:
        return 'ABORTED';
      case GoalStatus.rejected:
        return 'REJECTED';
      case GoalStatus.canceled:
        return 'CANCELED';
      case GoalStatus.canceledByUser:
        return 'CANCELED_BY_USER';
    }
  }
}
