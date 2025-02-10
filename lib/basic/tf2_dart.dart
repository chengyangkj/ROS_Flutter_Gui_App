import 'dart:collection';

import 'tf.dart';
import 'RobotPose.dart';

class TF2Dart {
  Map<String, Set<String>> adj = {}; //边
  Map<String, List<TransformElement>> adjTrasnform = {};

  void updateTF(TF data) {
    // 添加双向边，因为TF转换可以双向进行
    for (var trans in data.transforms) {
      String parentFrame = trans.header!.frameId;
      String childFrame = trans.childFrameId;

      // 添加正向边
      if (!adj.containsKey(parentFrame)) {
        adj[parentFrame] = {};
      }
      adj[parentFrame]?.add(childFrame);

      // 添加反向边
      if (!adj.containsKey(childFrame)) {
        adj[childFrame] = {};
      }
      adj[childFrame]?.add(parentFrame);

      // 存储转换关系
      if (!adjTrasnform.containsKey(parentFrame)) {
        adjTrasnform[parentFrame] = [];
      }
      adjTrasnform[parentFrame]!
          .removeWhere((element) => element.childFrameId == childFrame);
      adjTrasnform[parentFrame]?.add(trans);
    }
  }

  RobotPose lookUpForTransform(String from, String to) {
    try {
      var path = shortPath(from, to);
      if (path.isEmpty) {
        print("Warning: No path found from $from to $to");
        return RobotPose(0, 0, 0); // 返回默认位置而不是抛出异常
      }

      RobotPose pose = RobotPose(0, 0, 0);
      for (int i = 0; i < path.length - 1; i++) {
        String curr = path[i];
        String next = path[i + 1];

        var transformList = adjTrasnform[curr];
        if (transformList == null || transformList.isEmpty) {
          print("Warning: No transform found from $curr to $next");
          continue;
        }

        var transform = transformList.firstWhere(
          (element) => element.childFrameId == next,
          orElse: () => TransformElement(
            header: Header(seq: 0, stamp: null, frameId: curr),
            childFrameId: next,
            transform: null,
          ),
        );

        if (transform.transform != null) {
          pose = absoluteSum(pose, transform.transform!.getRobotPose());
        }
      }
      return pose;
    } catch (e) {
      print("Error in lookUpForTransform: $e");
      return RobotPose(0, 0, 0); // 返回默认位置
    }
  }

  List<String> shortPath(String from, String to) {
    if (from == to) return [from];
    if (!adj.containsKey(from)) {
      print("Warning: Frame '$from' not found in TF tree");
      return [];
    }

    Map<String, String> parent = {};
    Queue<String> queue = Queue<String>();
    Set<String> visited = {from};

    queue.add(from);
    parent[from] = "";

    while (queue.isNotEmpty) {
      String current = queue.removeFirst();
      if (current == to) {
        // 构建路径
        List<String> path = [];
        String node = to;
        while (node != "") {
          path.insert(0, node);
          node = parent[node] ?? "";
        }
        return path;
      }

      for (String next in (adj[current] ?? {})) {
        if (!visited.contains(next)) {
          visited.add(next);
          queue.add(next);
          parent[next] = current;
        }
      }
    }

    print("Warning: No path found between '$from' and '$to'");
    return [];
  }
}
