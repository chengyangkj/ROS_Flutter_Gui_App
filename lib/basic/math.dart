import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart' as vm;

// 将欧拉角（弧度）转换为四元数
vm.Quaternion eulerToQuaternion(
    double yawRadians, double pitchRadians, double rollRadians) {
  return vm.Quaternion.euler(
      rollRadians, pitchRadians, yawRadians); // 注意：输入顺序在这里是 roll, pitch, yaw
}

List<double> quaternionToEuler(vm.Quaternion quaternion) {
  // Quaternion components
  double w = quaternion.w;
  double x = quaternion.x;
  double y = quaternion.y;
  double z = quaternion.z;

  // Calculate roll (x-axis rotation)
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double roll = math.atan2(sinr_cosp, cosr_cosp);

  // Calculate pitch (y-axis rotation)
  double sinp = 2.0 * (w * y - z * x);
  double pitch;
  if (sinp.abs() >= 1) {
    pitch = math.pi / 2 * sinp.sign; // Use 90 degrees if out of range
  } else {
    pitch = math.asin(sinp);
  }

  // Calculate yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = math.atan2(siny_cosp, cosy_cosp);
  return [yaw, pitch, roll];
}
