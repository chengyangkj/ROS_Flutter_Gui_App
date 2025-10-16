// Copyright (c) 2019 Conrad Heidebrecht.

import 'package:roslibdart/core/core.dart';

/// Example usage of the Action class.
void main() async {
  // Create ROS connection
  final ros = Ros();
  await ros.connect(url: 'ws://localhost:9090');

  // Create an action client
  final action = Action(
    ros: ros,
    name: '/fibonacci',
    type: 'example_interfaces/Fibonacci',
  );

  // Send a goal
  final goalId = await action.sendGoal(
    {'order': 10},
    resultCallback: (result) async {
      print('Action completed with result: $result');
    },
    feedbackCallback: (feedback) async {
      print('Action feedback: $feedback');
    },
    failedCallback: (error) async {
      print('Action failed: $error');
    },
  );

  print('Sent goal with ID: $goalId');

  // Cancel the goal after 5 seconds
  Future.delayed(Duration(seconds: 5), () {
    if (goalId != null) {
      action.cancelGoal(goalId);
      print('Cancelled goal: $goalId');
    }
  });

  // Example of advertising an action (server side)
  final serverAction = Action(
    ros: ros,
    name: '/my_action',
    type: 'my_package/MyAction',
  );

  await serverAction.advertise(
    (goal, id) async {
      print('Received goal: $goal with ID: $id');
      
      // Simulate some work
      for (int i = 0; i < 10; i++) {
        await Future.delayed(Duration(milliseconds: 500));
        await serverAction.sendFeedback(id, {'progress': i * 10});
      }
      
      // Complete the action
      await serverAction.setSucceeded(id, {'result': 'Success!'});
    },
    cancelCallback: (id) async {
      print('Goal cancelled: $id');
      await serverAction.setCanceled(id, {'result': 'Cancelled'});
    },
  );

  print('Action server advertised');

  // Keep the program running
  await Future.delayed(Duration(seconds: 10));
  
  // Clean up
  await action.dispose();
  await serverAction.unadvertise();
  await ros.close();
}
