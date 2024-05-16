import 'dart:collection';

import 'tf.dart';
import 'RobotPose.dart';

class TF2Dart {
  Map<String, Set<String>> adj = {}; //边
  Map<String, List<TransformElement>> adjTrasnform = {};
  void updateTF(TF data) {
    for (var trans in data.transforms) {
      if (!adj.containsKey(trans.header!.frameId)) {
        adj[trans.header!.frameId] = {};
      }
      adj[trans.header!.frameId]?.add(trans.childFrameId);

      if (!adjTrasnform.containsKey(trans.header!.frameId)) {
        adjTrasnform[trans.header!.frameId] = [];
      }

      adjTrasnform[trans.header!.frameId]!
          .removeWhere((element) => element.childFrameId == trans.childFrameId);

      adjTrasnform[trans.header!.frameId]?.add(trans);
    }
  }

  RobotPose lookUpForTransform(String from, String to) {
    var path = shortPath(from, to);
    if (path.isEmpty) {
      throw Exception("No transform find from $from to $to");
    }
    RobotPose pose = RobotPose(0, 0, 0);
    for (int i = 0; i < path.length - 1; i++) {
      String curr = path[i];
      String next = path[i + 1];
      var transform = adjTrasnform[curr]!
          .firstWhere((element) => element.childFrameId == next);
      pose = absoluteSum(pose, transform.transform!.getRobotPose());
    }
    return pose;
  }

  List<String> shortPath(String from, String to) {
    if (!adj.containsKey(from) || adj[from]!.isEmpty) return [];

    // BFS
    Queue<List<String>> queue = Queue<List<String>>();
    Set<String> visited = {};
    queue.add([from]);
    while (queue.isNotEmpty) {
      var path = queue.removeFirst();
      var last = path.last;
      if (last == to) {
        return path;
      }
      if (visited.contains(last)) {
        continue;
      }
      visited.add(last);
      var neighbor = adj[last] ?? {};
      for (var trans in neighbor) {
        if (visited.contains(neighbor)) continue;
        List<String> newPath = List.from(path);
        newPath.add(trans);
        queue.add(newPath);
      }
    }
    return [];
  }
}
