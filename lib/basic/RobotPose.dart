import 'dart:math';
import 'package:vector_math/vector_math_64.dart';

class RobotPose {
  double x;
  double y;
  double theta;

  RobotPose(this.x, this.y, this.theta);

  // 构造函数
  RobotPose.fromJson(Map<String, dynamic> json)
      : x = json['x'],
        y = json['y'],
        theta = json['theta'];

  // 方法
  Map<String, dynamic> toJson() => {
        'x': x,
        'y': y,
        'theta': theta,
      };
  @override
  String toString() => 'RobotPose(x: $x, y: $y, theta: $theta)';
  RobotPose operator +(RobotPose other) =>
      RobotPose(x + other.x, y + other.y, theta + other.theta);
  RobotPose operator -(RobotPose other) =>
      RobotPose(x - other.x, y - other.y, theta - other.theta);
}

/*
@desc 两个位姿的和 p2表示增量。该函数表示在P1的位姿上，加上一个P2的增量
*/
RobotPose absoluteSum(RobotPose p1, RobotPose p2) {
  double s = sin(p1.theta);
  double c = cos(p1.theta);
  return RobotPose(c * p2.x - s * p2.y, s * p2.x + c * p2.y, p2.theta) + p1;
}

/*
@desc 两个位姿的差值，算出来P1在以P2为原点的坐标系里面的坐标。
*/
RobotPose absoluteDifference(RobotPose p1, RobotPose p2) {
  RobotPose delta = p1 - p2;
  delta.theta = atan2(sin(delta.theta), cos(delta.theta));
  double s = sin(p2.theta), c = cos(p2.theta);
  return RobotPose(
      c * delta.x + s * delta.y, -s * delta.x + c * delta.y, delta.theta);
}

double deg2rad(double deg) => deg * pi / 180;

double rad2deg(double rad) => rad * 180 / pi;
