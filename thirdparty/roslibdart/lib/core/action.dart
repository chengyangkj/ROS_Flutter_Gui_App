// Copyright (c) 2019 Conrad Heidebrecht.

import 'dart:async';
import 'ros.dart';
import 'request.dart';
import 'goal_status.dart';

/// Callback function to handle action goals when the action is advertised.
typedef ActionGoalHandler = Future<void> Function(Map<String, dynamic> goal, String id);

/// Callback function to handle action goal cancellation.
typedef ActionCancelHandler = Future<void> Function(String id);

/// Callback function to handle action goal results.
typedef ActionResultCallback = Future<void> Function(Map<String, dynamic> result);

/// Callback function to handle action goal feedback.
typedef ActionFeedbackCallback = Future<void> Function(Map<String, dynamic> feedback);

/// Callback function to handle action goal failures.
typedef ActionFailedCallback = Future<void> Function(String error);

/// Wrapper to interact with ROS actions.
class Action {
  Action({
    required this.ros,
    required this.name,
    required this.type,
    this.reconnectOnClose = true,
  });

  /// The ROS connection.
  Ros ros;

  /// Name of the action.
  String name;

  /// Action type the action uses.
  String type;

  /// Action type alias for compatibility.
  String get actionType => type;

  /// Action goal ID provided by [ros].
  String? actionGoalId;

  /// Advertiser ID provided by [ros].
  String? advertiseId;

  /// Checks whether or not the action is currently advertised.
  bool get isAdvertised => advertiseId != null;

  /// Flag to enable readvertisement on a ROS connection close event.
  bool reconnectOnClose;

  /// Stream subscription for action messages.
  StreamSubscription<Map<String, dynamic>>? _subscription;

  /// Action goal handler callback.
  ActionGoalHandler? _actionCallback;

  /// Action cancel handler callback.
  ActionCancelHandler? _cancelCallback;

  /// Send an action goal. Returns the feedback in the feedback callback while the action is running
  /// and the result in the result callback when the action is completed.
  /// Does nothing if this action is currently advertised.
  ///
  /// [goal] - The action goal to send.
  /// [resultCallback] - The callback function when the action is completed.
  /// [feedbackCallback] - The callback function when the action publishes feedback.
  /// [failedCallback] - The callback function when the action failed.
  Future<String?> sendGoal(
    Map<String, dynamic> goal, {
    ActionResultCallback? resultCallback,
    ActionFeedbackCallback? feedbackCallback,
    ActionFailedCallback? failedCallback,
  }) async {
    if (isAdvertised) {
      return null;
    }

    actionGoalId = 'send_action_goal:$name:${ros.subscribers + ros.advertisers + ros.publishers + ros.serviceCallers}';

    if (resultCallback != null || failedCallback != null || feedbackCallback != null) {
      _subscription = ros.stream.listen((Map<String, dynamic> message) async {
        if (message['id'] != actionGoalId) {
          return;
        }

        if (message['result'] != null && message['result'] == false) {
          if (failedCallback != null) {
            await failedCallback(message['values']?.toString() ?? 'Unknown error');
          }
        } else if (message['op'] == 'action_feedback' && feedbackCallback != null) {
          await feedbackCallback(message['values'] ?? {});
        } else if (message['op'] == 'action_result' && resultCallback != null) {
          await resultCallback(message['values'] ?? {});
        }
      });
    }

    final call = Request(
      op: 'send_action_goal',
      id: actionGoalId,
      type: type,
      topic: name,
      args: goal,
      values: {'feedback': true},
    );

    await safeSend(call);
    return actionGoalId;
  }

  /// Cancel an action goal.
  ///
  /// [id] - The ID of the action goal to cancel.
  Future<void> cancelGoal(String id) async {
    final call = Request(
      op: 'cancel_action_goal',
      id: id,
      topic: name,
    );
    await safeSend(call);
  }

  /// Advertise the action. This turns the Action object from a client
  /// into a server. The callback will be called with every goal sent to this action.
  ///
  /// [actionCallback] - This works similarly to the callback for a C++ action.
  /// [cancelCallback] - A callback function to execute when the action is canceled.
  Future<void> advertise(
    ActionGoalHandler actionCallback, {
    ActionCancelHandler? cancelCallback,
  }) async {
    if (isAdvertised) {
      return;
    }

    _actionCallback = actionCallback;
    _cancelCallback = cancelCallback;

    advertiseId = ros.requestAdvertiser(name);
    await safeSend(Request(
      op: 'advertise_action',
      id: advertiseId,
      type: type,
      topic: name,
    ));

    // Listen for action goals
    _subscription = ros.stream.listen((Map<String, dynamic> message) async {
      if (message['topic'] == name && message['op'] == 'send_action_goal') {
        await _executeAction(message);
      }
    });

    // If the ROS connection closes show that we're not advertising anymore.
    watchForClose();
  }

  /// Unadvertise a previously advertised action.
  Future<void> unadvertise() async {
    if (!isAdvertised) {
      return;
    }

    await safeSend(Request(
      op: 'unadvertise_action',
      id: advertiseId,
      topic: name,
    ));

    await _subscription?.cancel();
    _subscription = null;
    advertiseId = null;
  }

  /// Helper function that executes an action by calling the provided
  /// action callback with the auto-generated ID as a user-accessible input.
  /// Should not be called manually.
  Future<void> _executeAction(Map<String, dynamic> rosbridgeRequest) async {
    final id = rosbridgeRequest['id'] as String?;
    final args = rosbridgeRequest['args'] as Map<String, dynamic>? ?? {};

    if (id != null) {
      // Listen for cancellation events
      ros.stream.listen((Map<String, dynamic> message) async {
        if (message['id'] == id && 
            message['op'] == 'cancel_action_goal' && 
            _cancelCallback != null) {
          await _cancelCallback!(id);
        }
      });
    }

    // Call the action goal execution function provided.
    if (_actionCallback != null) {
      await _actionCallback!(args, id ?? '');
    }
  }

  /// Helper function to send action feedback inside an action handler.
  ///
  /// [id] - The action goal ID.
  /// [feedback] - The feedback to send.
  Future<void> sendFeedback(String id, Map<String, dynamic> feedback) async {
    final call = Request(
      op: 'action_feedback',
      id: id,
      topic: name,
      values: feedback,
    );
    await safeSend(call);
  }

  /// Helper function to set an action as succeeded.
  ///
  /// [id] - The action goal ID.
  /// [result] - The result to set.
  Future<void> setSucceeded(String id, Map<String, dynamic> result) async {
    final call = Request(
      op: 'action_result',
      id: id,
      topic: name,
      values: result,
      args: {'status': GoalStatus.succeeded.statusCode, 'result': true},
    );
    await safeSend(call);
  }

  /// Helper function to set an action as canceled.
  ///
  /// [id] - The action goal ID.
  /// [result] - The result to set.
  Future<void> setCanceled(String id, Map<String, dynamic> result) async {
    final call = Request(
      op: 'action_result',
      id: id,
      topic: name,
      values: result,
      args: {'status': GoalStatus.canceled.statusCode, 'result': true},
    );
    await safeSend(call);
  }

  /// Helper function to set an action as failed.
  ///
  /// [id] - The action goal ID.
  Future<void> setFailed(String id) async {
    final call = Request(
      op: 'action_result',
      id: id,
      topic: name,
      args: {'status': GoalStatus.aborted.statusCode, 'result': false},
    );
    await safeSend(call);
  }

  /// Wait for the connection to close and then reset advertising variables.
  Future<void> watchForClose() async {
    if (!reconnectOnClose) {
      await ros.statusStream.firstWhere((s) => s == Status.closed);
      advertiseId = null;
    }
  }

  /// Safely send a [message] to ROS.
  Future<void> safeSend(Request message) async {
    // Send the message but if we're not connected and the [reconnectOnClose] flag
    // is set, wait for ROS to reconnect and then resend the [message].
    ros.send(message);
    if (reconnectOnClose && ros.status != Status.connected) {
      await ros.statusStream.firstWhere((s) => s == Status.connected);
      ros.send(message);
    }
  }

  /// Dispose of resources.
  Future<void> dispose() async {
    await _subscription?.cancel();
    _subscription = null;
    advertiseId = null;
    actionGoalId = null;
  }
}
