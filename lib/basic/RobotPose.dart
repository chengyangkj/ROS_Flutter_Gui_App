import 'dart:math';

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

RobotPose absoluteSum(RobotPose p1, RobotPose p2) {
  double s = sin(p1.theta);
  double c = cos(p1.theta);
  return RobotPose(c * p2.x - s * p2.y, s * p2.x + c * p2.y, p2.theta) + p1;
}

double deg2rad(double deg) => deg * pi / 180;

double rad2deg(double rad) => rad * 180 / pi;
