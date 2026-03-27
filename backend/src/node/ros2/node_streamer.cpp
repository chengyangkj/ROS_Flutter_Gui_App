#include "node/ros2/node.hpp"

#include "core/map/json.hpp"
#include "core/robot_stream/robot_message_hub.hpp"
#include "common/config/config.hpp"
#include "common/logger/logger.h"
#include "robot_message.pb.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cstdio>
#include <cmath>
#include <cstring>

using namespace std::placeholders;

namespace ros_gui_backend {

namespace {

std::string NormalizeTopicName(const std::string& topic) {
  if (topic.empty()) {
    return topic;
  }
  if (topic[0] == '/') {
    return topic;
  }
  return "/" + topic;
}

struct Pose2 {
  double x;
  double y;
  double th;
};

Pose2 AbsSum(const Pose2& p1, const Pose2& p2) {
  const double s = std::sin(p1.th);
  const double c = std::cos(p1.th);
  return {c * p2.x - s * p2.y + p1.x, s * p2.x + c * p2.y + p1.y, p1.th + p2.th};
}

Pose2 TfToPose2(const geometry_msgs::msg::Transform& t) {
  return {t.translation.x, t.translation.y, tf2::getYaw(t.rotation)};
}

bool LookupLatestTfPose(tf2_ros::Buffer& buf, const std::string& target_frame, const std::string& source_frame,
    Pose2* out, std::string* err) {
  try {
    auto st = buf.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    *out = TfToPose2(st.transform);
    return true;
  } catch (const std::exception& e) {
    if (err) {
      *err = e.what();
    }
    return false;
  }
}

void BroadcastRobotMsg(const ros_gui_backend::pb::RobotMessage& m) {
  RobotMessageHub::Instance().BroadcastRobotMsg(m);
}

void FillOccGridProto(const nav_msgs::msg::OccupancyGrid& in, ros_gui_backend::pb::OccupancyGridProto* out) {
  out->set_frame_id(in.header.frame_id);
  auto* hdr = out->mutable_header();
  hdr->set_frame_id(in.header.frame_id);
  hdr->mutable_stamp()->set_sec(static_cast<int32_t>(in.header.stamp.sec));
  hdr->mutable_stamp()->set_nanosec(static_cast<uint32_t>(in.header.stamp.nanosec));
  auto* info = out->mutable_info();
  info->mutable_map_load_time()->set_sec(static_cast<int32_t>(in.info.map_load_time.sec));
  info->mutable_map_load_time()->set_nanosec(static_cast<uint32_t>(in.info.map_load_time.nanosec));
  info->set_resolution(static_cast<float>(in.info.resolution));
  info->set_width(in.info.width);
  info->set_height(in.info.height);
  auto* o = info->mutable_origin();
  o->mutable_position()->set_x(in.info.origin.position.x);
  o->mutable_position()->set_y(in.info.origin.position.y);
  o->mutable_position()->set_z(in.info.origin.position.z);
  o->mutable_orientation()->set_x(in.info.origin.orientation.x);
  o->mutable_orientation()->set_y(in.info.origin.orientation.y);
  o->mutable_orientation()->set_z(in.info.origin.orientation.z);
  o->mutable_orientation()->set_w(in.info.origin.orientation.w);
  out->clear_data();
  for (int8_t v : in.data) {
    out->add_data(static_cast<int32_t>(v));
  }
}

void FillPoseStampedMap(
    const std::string& map_frame, const rclcpp::Time& stamp, const Pose2& p, ros_gui_backend::pb::PoseStamped* ps) {
  ps->mutable_header()->set_frame_id(map_frame);
  ps->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(stamp.seconds()));
  ps->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(stamp.nanoseconds() % 1000000000ull));
  ps->mutable_pose()->mutable_position()->set_x(p.x);
  ps->mutable_pose()->mutable_position()->set_y(p.y);
  ps->mutable_pose()->mutable_position()->set_z(0);
  tf2::Quaternion q;
  q.setRPY(0, 0, p.th);
  ps->mutable_pose()->mutable_orientation()->set_x(q.x());
  ps->mutable_pose()->mutable_orientation()->set_y(q.y());
  ps->mutable_pose()->mutable_orientation()->set_z(q.z());
  ps->mutable_pose()->mutable_orientation()->set_w(q.w());
}

void FillPathInMapFrame(const std::string& map_frame, const nav_msgs::msg::Path& path, const Pose2& trans_to_map,
    ros_gui_backend::pb::Path* out_path) {
  out_path->mutable_header()->set_frame_id(map_frame);
  out_path->mutable_header()->mutable_stamp()->set_sec(path.header.stamp.sec);
  out_path->mutable_header()->mutable_stamp()->set_nanosec(path.header.stamp.nanosec);
  for (const auto& ps : path.poses) {
    Pose2 local{ps.pose.position.x, ps.pose.position.y, 0};
    const Pose2 m = AbsSum(trans_to_map, local);
    auto* pst = out_path->add_poses();
    FillPoseStampedMap(map_frame, rclcpp::Time(path.header.stamp), m, pst);
  }
}

}  // namespace

void RosGuiNode::PoseTimerTick() {
  Pose2 pose{};
  std::string e;
  if (!LookupLatestTfPose(*tf_buffer_, gui_settings_.MapFrameName, gui_settings_.BaseLinkFrameName, &pose, &e)) {
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  FillPoseStampedMap(gui_settings_.MapFrameName, now(), pose, out.mutable_robot_pose_map());
  BroadcastRobotMsg(out);
}

void RosGuiNode::ClearGuiStreamsLocked() {
  stream_subs_.clear();
  cmd_vel_pub_.reset();
  nav_goal_pub_.reset();
  reloc_pub_.reset();
  nav_cancel_pub_.reset();
}

void RosGuiNode::SetupGuiStreamsLocked() {
  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = stream_callback_group_;

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(NormalizeTopicName(gui_settings_.SpeedCtrlTopic), 10);
  nav_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(NormalizeTopicName(gui_settings_.NavGoalTopic), 10);
  reloc_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      NormalizeTopicName(gui_settings_.RelocTopic), 10);
  const std::string cancel_topic = NormalizeTopicName(gui_settings_.NavGoalTopic) + "/cancel";
  nav_cancel_pub_ = create_publisher<std_msgs::msg::Empty>(cancel_topic, 10);

  auto add = [this, &sub_opt](auto sub) {
    stream_subs_.push_back(std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(sub));
  };

  add(create_subscription<sensor_msgs::msg::LaserScan>(
      NormalizeTopicName(gui_settings_.LaserTopic), rclcpp::SensorDataQoS(),
      std::bind(&RosGuiNode::OnLaser, this, _1), sub_opt));

  add(create_subscription<nav_msgs::msg::Path>(
      NormalizeTopicName(gui_settings_.LocalPathTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnLocalPath, this, _1), sub_opt));

  add(create_subscription<nav_msgs::msg::Path>(
      NormalizeTopicName(gui_settings_.GlobalPathTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnGlobalPath, this, _1), sub_opt));

  add(create_subscription<nav_msgs::msg::Path>(
      NormalizeTopicName(gui_settings_.TracePathTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnTracePath, this, _1), sub_opt));

  add(create_subscription<nav_msgs::msg::Odometry>(
      NormalizeTopicName(gui_settings_.OdomTopic), rclcpp::QoS(10), std::bind(&RosGuiNode::OnOdom, this, _1),
      sub_opt));

  add(create_subscription<sensor_msgs::msg::BatteryState>(
      NormalizeTopicName(gui_settings_.BatteryTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnBattery, this, _1), sub_opt));

  add(create_subscription<geometry_msgs::msg::PolygonStamped>(
      NormalizeTopicName(gui_settings_.RobotFootprintTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnFootprint, this, _1), sub_opt));

  add(create_subscription<nav_msgs::msg::OccupancyGrid>(
      NormalizeTopicName(gui_settings_.LocalCostmapTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnLocalCostmap, this, _1), sub_opt));

  add(create_subscription<nav_msgs::msg::OccupancyGrid>(
      NormalizeTopicName(gui_settings_.GlobalCostmapTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnGlobalCostmap, this, _1), sub_opt));

  add(create_subscription<sensor_msgs::msg::PointCloud2>(
      NormalizeTopicName(gui_settings_.PointCloud2Topic), rclcpp::SensorDataQoS(),
      std::bind(&RosGuiNode::OnPointCloud2, this, _1), sub_opt));

  add(create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      NormalizeTopicName(gui_settings_.DiagnosticTopic), rclcpp::QoS(10),
      std::bind(&RosGuiNode::OnDiagnostic, this, _1), sub_opt));

  add(create_subscription<action_msgs::msg::GoalStatusArray>(
      NormalizeTopicName(gui_settings_.NavToPoseStatusTopic), rclcpp::QoS(100),
      std::bind(&RosGuiNode::OnGoalStatus, this, _1), sub_opt));

  add(create_subscription<action_msgs::msg::GoalStatusArray>(
      NormalizeTopicName(gui_settings_.NavThroughPosesStatusTopic), rclcpp::QoS(100),
      std::bind(&RosGuiNode::OnGoalStatus, this, _1), sub_opt));

  last_local_costmap_push_ = std::chrono::steady_clock::time_point{};
  last_global_costmap_push_ = std::chrono::steady_clock::time_point{};
}

bool RosGuiNode::ReloadGuiStreams(const GuiAppSettings& settings) {
  std::lock_guard<std::mutex> lk(stream_mu_);
  gui_settings_ = settings;
  ClearGuiStreamsLocked();
  SetupGuiStreamsLocked();
  return true;
}

bool RosGuiNode::PublishCmdVel(double vx, double vy, double vw) {
  std::lock_guard<std::mutex> lk(stream_mu_);
  if (!cmd_vel_pub_) {
    return false;
  }
  geometry_msgs::msg::Twist t;
  t.linear.x = vx;
  t.linear.y = vy;
  t.linear.z = 0;
  t.angular.x = 0;
  t.angular.y = 0;
  t.angular.z = vw;
  cmd_vel_pub_->publish(t);
  return true;
}

bool RosGuiNode::PublishNavGoal(double x, double y, double roll, double pitch, double yaw) {
  std::lock_guard<std::mutex> lk(stream_mu_);
  if (!nav_goal_pub_) {
    return false;
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = now();
  ps.header.frame_id = gui_settings_.MapFrameName;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  ps.pose.orientation = tf2::toMsg(q);
  nav_goal_pub_->publish(ps);
  return true;
}

bool RosGuiNode::PublishInitialPose(double x, double y, double roll, double pitch, double yaw) {
  std::lock_guard<std::mutex> lk(stream_mu_);
  if (!reloc_pub_) {
    return false;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped pc;
  pc.header.stamp = now();
  pc.header.frame_id = gui_settings_.MapFrameName;
  pc.pose.pose.position.x = x;
  pc.pose.pose.position.y = y;
  pc.pose.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  pc.pose.pose.orientation = tf2::toMsg(q);
  for (double& c : pc.pose.covariance) {
    c = 0;
  }
  pc.pose.covariance[0] = 0.1;
  pc.pose.covariance[7] = 0.1;
  pc.pose.covariance[14] = 0.1;
  pc.pose.covariance[21] = 0.1;
  pc.pose.covariance[28] = 0.1;
  pc.pose.covariance[35] = 0.1;
  reloc_pub_->publish(pc);
  return true;
}

bool RosGuiNode::PublishNavCancel() {
  std::lock_guard<std::mutex> lk(stream_mu_);
  if (!nav_cancel_pub_) {
    return false;
  }
  nav_cancel_pub_->publish(std_msgs::msg::Empty());
  return true;
}

bool RosGuiNode::LookupTransform(const std::string& target_frame, const std::string& source_frame,
    std::string* json_out, std::string* err) {
  if (!tf_buffer_) {
    if (err) {
      *err = "tf buffer not ready";
    }
    return false;
  }
  geometry_msgs::msg::TransformStamped st;
  try {
    st = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const std::exception& e) {
    if (err) {
      *err = e.what();
    }
    return false;
  }
  const double yaw = tf2::getYaw(st.transform.rotation);
  nlohmann::json j;
  j["ok"] = true;
  j["translation"] = {{"x", st.transform.translation.x}, {"y", st.transform.translation.y},
      {"z", st.transform.translation.z}};
  j["rotation"] = {{"x", st.transform.rotation.x}, {"y", st.transform.rotation.y},
      {"z", st.transform.rotation.z}, {"w", st.transform.rotation.w}};
  j["yaw"] = yaw;
  if (json_out) {
    *json_out = j.dump();
  }
  return true;
}

void RosGuiNode::OnLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  Pose2 trans_v2{};
  std::string e;
  if (!LookupLatestTfPose(
          *tf_buffer_, gui_settings_.BaseLinkFrameName, msg->header.frame_id, &trans_v2, &e)) {
    LOGGER_ERROR(
        "OnLaser TF {} <- {} failed: {}", gui_settings_.BaseLinkFrameName, msg->header.frame_id, e);
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  auto* lm = out.mutable_laser_scan();
  lm->mutable_header()->set_frame_id(gui_settings_.BaseLinkFrameName);
  lm->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(msg->header.stamp.sec));
  lm->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(msg->header.stamp.nanosec));
  const float a0 = msg->angle_min;
  const float inc = msg->angle_increment;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {

      continue;
    }
    float a = a0 + static_cast<float>(i) * inc;
    const Pose2 hit_in_laser{r * std::cos(static_cast<double>(a)), r * std::sin(static_cast<double>(a)), 0};
    const Pose2 hit_base = AbsSum(trans_v2, hit_in_laser);
    lm->add_x(static_cast<float>(hit_base.x));
    lm->add_y(static_cast<float>(hit_base.y));
  }
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnLocalPath(const nav_msgs::msg::Path::SharedPtr msg) {
  Pose2 trans_v2{};
  std::string e;
  if (!LookupLatestTfPose(
          *tf_buffer_, gui_settings_.MapFrameName, msg->header.frame_id, &trans_v2, &e)) {
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  FillPathInMapFrame(gui_settings_.MapFrameName, *msg, trans_v2, out.mutable_path_local());
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnGlobalPath(const nav_msgs::msg::Path::SharedPtr msg) {
  Pose2 trans_v2{};
  std::string e;
  if (!LookupLatestTfPose(
          *tf_buffer_, gui_settings_.MapFrameName, msg->header.frame_id, &trans_v2, &e)) {
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  FillPathInMapFrame(gui_settings_.MapFrameName, *msg, trans_v2, out.mutable_path_global());
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnTracePath(const nav_msgs::msg::Path::SharedPtr msg) {
  Pose2 trans_v2{};
  std::string e;
  if (!LookupLatestTfPose(
          *tf_buffer_, gui_settings_.MapFrameName, msg->header.frame_id, &trans_v2, &e)) {
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  FillPathInMapFrame(gui_settings_.MapFrameName, *msg, trans_v2, out.mutable_path_trace());
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  ros_gui_backend::pb::RobotMessage out;
  auto* odom = out.mutable_odometry();
  odom->mutable_header()->set_frame_id(msg->header.frame_id);
  odom->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(msg->header.stamp.sec));
  odom->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(msg->header.stamp.nanosec));
  odom->set_child_frame_id(msg->child_frame_id);
  auto* pose = odom->mutable_pose()->mutable_pose();
  pose->mutable_position()->set_x(msg->pose.pose.position.x);
  pose->mutable_position()->set_y(msg->pose.pose.position.y);
  pose->mutable_position()->set_z(msg->pose.pose.position.z);
  pose->mutable_orientation()->set_x(msg->pose.pose.orientation.x);
  pose->mutable_orientation()->set_y(msg->pose.pose.orientation.y);
  pose->mutable_orientation()->set_z(msg->pose.pose.orientation.z);
  pose->mutable_orientation()->set_w(msg->pose.pose.orientation.w);
  odom->mutable_pose()->clear_covariance();
  for (double c : msg->pose.covariance) {
    odom->mutable_pose()->add_covariance(c);
  }
  odom->mutable_twist()->mutable_linear()->set_x(msg->twist.twist.linear.x);
  odom->mutable_twist()->mutable_linear()->set_y(msg->twist.twist.linear.y);
  odom->mutable_twist()->mutable_linear()->set_z(msg->twist.twist.linear.z);
  odom->mutable_twist()->mutable_angular()->set_x(msg->twist.twist.angular.x);
  odom->mutable_twist()->mutable_angular()->set_y(msg->twist.twist.angular.y);
  odom->mutable_twist()->mutable_angular()->set_z(msg->twist.twist.angular.z);
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  ros_gui_backend::pb::RobotMessage out;
  auto* b = out.mutable_battery();
  b->mutable_header()->set_frame_id(msg->header.frame_id);
  b->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(msg->header.stamp.sec));
  b->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(msg->header.stamp.nanosec));
  int pct = 0;
  if (msg->percentage >= 0.0f && msg->percentage <= 1.0f) {
    pct = static_cast<int>(std::lround(static_cast<double>(msg->percentage) * 100.0));
  } else {
    pct = static_cast<int>(std::lround(static_cast<double>(msg->percentage)));
  }
  b->set_percentage(pct);
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
  Pose2 trans_v2{};
  std::string e;
  if (!LookupLatestTfPose(*tf_buffer_, gui_settings_.MapFrameName, msg->header.frame_id, &trans_v2, &e)) {
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  auto* poly = out.mutable_footprint();
  poly->mutable_header()->set_frame_id(gui_settings_.MapFrameName);
  poly->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(msg->header.stamp.sec));
  poly->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(msg->header.stamp.nanosec));
  for (const auto& pt : msg->polygon.points) {
    const Pose2 local{pt.x, pt.y, 0};
    const Pose2 m = AbsSum(trans_v2, local);
    auto* p = poly->mutable_polygon()->add_points();
    p->set_x(m.x);
    p->set_y(m.y);
    p->set_z(0);
  }
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  const auto t = std::chrono::steady_clock::now();
  if (last_local_costmap_push_.time_since_epoch().count() != 0) {
    if (t - last_local_costmap_push_ < std::chrono::seconds(5)) {
      return;
    }
  }
  last_local_costmap_push_ = t;
  ros_gui_backend::pb::RobotMessage out;
  FillOccGridProto(*msg, out.mutable_local_costmap());
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  const auto t = std::chrono::steady_clock::now();
  if (last_global_costmap_push_.time_since_epoch().count() != 0) {
    if (t - last_global_costmap_push_ < std::chrono::seconds(5)) {
      return;
    }
  }
  last_global_costmap_push_ = t;
  ros_gui_backend::pb::RobotMessage out;
  FillOccGridProto(*msg, out.mutable_global_costmap());
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  Pose2 trans_v2{};
  std::string e;
  if (!LookupLatestTfPose(*tf_buffer_, gui_settings_.MapFrameName, cloud->header.frame_id, &trans_v2, &e)) {
    return;
  }
  uint32_t x_off = 0, y_off = 0, z_off = 0;
  bool hx = false, hy = false, hz = false;
  for (const auto& f : cloud->fields) {
    if (f.name == "x") {
      x_off = f.offset;
      hx = true;
    } else if (f.name == "y") {
      y_off = f.offset;
      hy = true;
    } else if (f.name == "z") {
      z_off = f.offset;
      hz = true;
    }
  }
  if (!hx || !hy) {
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  auto* pcm = out.mutable_pointcloud_map();
  pcm->mutable_header()->set_frame_id(gui_settings_.MapFrameName);
  pcm->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(cloud->header.stamp.sec));
  pcm->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(cloud->header.stamp.nanosec));
  const size_t n = static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height);
  const size_t step = cloud->point_step;
  const uint8_t* base = cloud->data.data();
  float xf = 0, yf = 0, zf = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* row = base + i * step;
    std::memcpy(&xf, row + x_off, sizeof(float));
    std::memcpy(&yf, row + y_off, sizeof(float));
    zf = 0;
    if (hz) {
      std::memcpy(&zf, row + z_off, sizeof(float));
    }
    const Pose2 hit_map = AbsSum(trans_v2, Pose2{xf, yf, 0});
    pcm->add_x(static_cast<double>(hit_map.x));
    pcm->add_y(static_cast<double>(hit_map.y));
    pcm->add_z(static_cast<double>(zf));
  }
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnDiagnostic(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
  ros_gui_backend::pb::RobotMessage out;
  auto* arr = out.mutable_diagnostic();
  arr->mutable_header()->set_frame_id(msg->header.frame_id);
  arr->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(msg->header.stamp.sec));
  arr->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(msg->header.stamp.nanosec));
  for (const auto& s : msg->status) {
    auto* ds = arr->add_status();
    ds->set_level(s.level);
    ds->set_name(s.name);
    ds->set_message(s.message);
    ds->set_hardware_id(s.hardware_id);
    for (const auto& kv : s.values) {
      auto* p = ds->add_values();
      p->set_key(kv.key);
      p->set_value(kv.value);
    }
  }
  BroadcastRobotMsg(out);
}

void RosGuiNode::OnGoalStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
  if (msg->status_list.empty()) {
    return;
  }
  ros_gui_backend::pb::RobotMessage out;
  auto* arr = out.mutable_nav_status();
  arr->mutable_header()->set_frame_id("");
  const auto st = now();
  arr->mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(st.seconds()));
  arr->mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(st.nanoseconds() % 1000000000ull));
  for (const auto& s : msg->status_list) {
    auto* g = arr->add_status_list();
    g->set_status(static_cast<uint32_t>(s.status));
    std::string u;
    u.reserve(s.goal_info.goal_id.uuid.size() * 2);
    for (uint8_t b : s.goal_info.goal_id.uuid) {
      char buf[4];
      std::snprintf(buf, sizeof(buf), "%02x", b);
      u += buf;
    }
    g->mutable_goal_info()->mutable_goal_id()->set_uuid(u);
    g->mutable_goal_info()->mutable_stamp()->set_sec(static_cast<int32_t>(s.goal_info.stamp.sec));
    g->mutable_goal_info()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(s.goal_info.stamp.nanosec));
  }
  BroadcastRobotMsg(out);
}

}  // namespace ros_gui_backend
