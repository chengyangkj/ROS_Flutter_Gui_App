#include "node/ros2/node.hpp"
#include "common/config/config.hpp"
#include "core/map/map_io.hpp"
#include "core/robot_stream/robot_message_hub.hpp"
#include "node/ros2/convert.hpp"
#include "common/logger/logger.h"
#include "robot_message.pb.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

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

void EraseImageSlots(const std::string& map_key,
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>& raw,
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr>&
        comp) {
  raw.erase(map_key);
  comp.erase(map_key);
}

bool ResolveImageTopic(rclcpp::Node& node, const std::string& request_topic, std::string* graph_topic,
    std::string* msg_type, std::string* err) {
  const std::string normalized = NormalizeTopicName(request_topic);
  std::map<std::string, std::vector<std::string>> names_types;
  try {
    names_types = node.get_topic_names_and_types();
  } catch (const std::exception& e) {
    if (err) {
      *err = std::string("failed to query topic graph: ") + e.what();
    }
    return false;
  }
  auto it = names_types.find(normalized);
  if (it == names_types.end()) {
    it = names_types.find(request_topic);
  }
  if (it == names_types.end()) {
    if (err) {
      *err = "topic not found or no publisher: " + request_topic;
    }
    return false;
  }
  if (it->second.empty()) {
    if (err) {
      *err = "topic has no message type: " + it->first;
    }
    return false;
  }
  static constexpr const char* kImage = "sensor_msgs/msg/Image";
  static constexpr const char* kCompressed = "sensor_msgs/msg/CompressedImage";
  std::string chosen;
  for (const auto& ty : it->second) {
    if (ty == kImage) {
      chosen = kImage;
      break;
    }
  }
  if (chosen.empty()) {
    for (const auto& ty : it->second) {
      if (ty == kCompressed) {
        chosen = kCompressed;
        break;
      }
    }
  }
  if (chosen.empty()) {
    if (err) {
      std::string joined;
      for (size_t i = 0; i < it->second.size(); ++i) {
        if (i > 0) {
          joined += ", ";
        }
        joined += it->second[i];
      }
      *err = "unsupported image message type on topic " + it->first + ": " + joined;
    }
    return false;
  }
  *graph_topic = it->first;
  *msg_type = std::move(chosen);
  return true;
}

constexpr const char* kPubMapTopic = "/map_manager/map";
constexpr const char* kSubMapTopic = "/map";
constexpr double kSaveMapTimeoutSec = 2.0;
constexpr double kFreeThreshDefault = 0.25;
constexpr double kOccupiedThreshDefault = 0.65;

}  // namespace

RosGuiNode::RosGuiNode() : rclcpp::Node("ros_gui_app_backend") {}

RosGuiNode::~RosGuiNode() {}

void RosGuiNode::PublishMapUpdate() {
  MapManager* mm = MapManager::Instance();
  if (mm && mm->IsMapAvailable() && occ_pub_) {
    occ_pub_->publish(OccGridPubMsg{mm->GetMapData(), mm->GetFrameId(), now()});
  }
}

bool RosGuiNode::Init(const GuiAppSettings& gui_app) {
  gui_settings_ = gui_app;
  MapManager::Instance()->SetOnMapUpdateCallback([this]() { PublishMapUpdate(); });

  const std::string pub_topic = kPubMapTopic;
  const std::string sub_topic = kSubMapTopic;

  const std::string prefix = get_name() + std::string("/");
  get_map_service_ = create_service<nav_msgs::srv::GetMap>(
      prefix + "map", std::bind(&RosGuiNode::GetMapCallback, this, _1, _2, _3));
  load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
      prefix + "load_map", std::bind(&RosGuiNode::LoadMapCallback, this, _1, _2, _3));
  save_map_service_ = create_service<nav2_msgs::srv::SaveMap>(
      prefix + "save_map", std::bind(&RosGuiNode::SaveMapCallback, this, _1, _2, _3));

  occ_pub_ = create_publisher<OccGridPubAdaptedType>(
      pub_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  raw_occ_map_sub_ = create_subscription<OccGridDataAdaptedType>(
      sub_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&RosGuiNode::RawOccMapUpdateCallback, this, _1));

  image_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);

  stream_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), false);

  {
    std::lock_guard<std::mutex> lk(stream_mu_);
    SetupGuiStreamsLocked();
  }

  pose_timer_ =
      create_wall_timer(std::chrono::milliseconds(50), std::bind(&RosGuiNode::PoseTimerTick, this));

  PublishMapUpdate();
  return true;
}

void RosGuiNode::Run() {
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 0u);
  executor.add_node(shared_from_this());
  executor.spin();
}

void RosGuiNode::Shutdown() {
  rclcpp::shutdown();
}

bool RosGuiNode::LoadMapResponseFromYaml(const std::string& yaml_file,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) {
  MapManager* map_manager = MapManager::Instance();
  if (!map_manager) return false;
  LOAD_MAP_STATUS status = map_manager->LoadMapFromYaml(yaml_file);
  if (status == MAP_DOES_NOT_EXIST) {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
    return false;
  }
  if (status == INVALID_MAP_METADATA) {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
    return false;
  }
  if (status == INVALID_MAP_DATA) {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
    return false;
  }
  FillOccGridMsg(map_manager->GetMapData(), response->map, map_manager->GetFrameId(), now());
  response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  return true;
}

void RosGuiNode::GetMapCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
  MapManager* mm = MapManager::Instance();
  if (mm) {
    FillOccGridMsg(mm->GetMapData(), response->map, mm->GetFrameId(), now());
  }
}

void RosGuiNode::LoadMapCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) {
  if (LoadMapResponseFromYaml(request->map_url, response)) {
    MapManager* mm = MapManager::Instance();
    if (mm) {
      occ_pub_->publish(OccGridPubMsg{mm->GetMapData(), mm->GetFrameId(), now()});
    }
  }
}

void RosGuiNode::SaveMapCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response) {
  SaveParameters save_parameters;
  save_parameters.map_file_name = request->map_url;
  save_parameters.image_format = request->image_format;
  save_parameters.free_thresh = request->free_thresh;
  save_parameters.occupied_thresh = request->occupied_thresh;
  try {
    save_parameters.mode = map_mode_from_string(request->map_mode);
  } catch (std::invalid_argument&) {
    save_parameters.mode = MapMode::Trinary;
  }
  response->result = SaveMapTopicToFile(request->map_topic, save_parameters);
}

void RosGuiNode::RawOccMapUpdateCallback(const std::shared_ptr<OccupancyGridData> msg) {
  MapManager::Instance()->UpdateMap(*msg);
}

bool RosGuiNode::SaveMapTopicToFile(const std::string& map_topic,
    const SaveParameters& save_parameters) {
  std::string topic = map_topic.empty() ? "map" : map_topic;
  SaveParameters params = save_parameters;
  if (params.free_thresh == 0.0) params.free_thresh = kFreeThreshDefault;
  if (params.occupied_thresh == 0.0) params.occupied_thresh = kOccupiedThreshDefault;

  std::promise<std::shared_ptr<OccupancyGridData>> prom;
  auto future_result = prom.get_future();
  auto map_cb = [&prom](std::shared_ptr<OccupancyGridData> msg) { prom.set_value(msg); };

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local().reliable();

  auto callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = callback_group;
  auto map_sub = create_subscription<OccGridDataAdaptedType>(topic, map_qos, map_cb, opt);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 0u);
  executor.add_callback_group(callback_group, get_node_base_interface());
  rclcpp::Duration save_timeout(rclcpp::Duration::from_seconds(kSaveMapTimeoutSec));
  auto timeout = save_timeout.to_chrono<std::chrono::nanoseconds>();
  if (executor.spin_until_future_complete(future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    LOGGER_ERROR("Failed to receive map from topic");
    return false;
  }

  map_sub.reset();
  auto map_data = future_result.get();
  return saveMapToFile(*map_data, params);
}

bool RosGuiNode::SetRobotStreamImageSubscription(
    const std::string& topic, bool subscribe, std::string* error_message) {
  std::lock_guard<std::mutex> lock(image_mu_);
  const std::string map_key = NormalizeTopicName(topic);
  if (!subscribe) {
    if (topic.empty()) {
      LOGGER_INFO("robot stream image: unsubscribe all topics");
      image_subs_.clear();
      compressed_image_subs_.clear();
    } else {
      LOGGER_INFO("robot stream image: unsubscribe topic={} (key={})", topic, map_key);
      EraseImageSlots(map_key, image_subs_, compressed_image_subs_);
    }
    return true;
  }
  if (topic.empty()) {
    if (error_message) {
      *error_message = "topic is empty";
    }
    LOGGER_ERROR("robot stream image: subscribe rejected: empty topic");
    return false;
  }

  std::string graph_topic;
  std::string msg_type;
  if (!ResolveImageTopic(*this, topic, &graph_topic, &msg_type, error_message)) {
    const std::string err = error_message ? *error_message : std::string();
    LOGGER_ERROR(
        "robot stream image: subscribe failed request_topic={} map_key={} err={}", topic, map_key, err);
    return false;
  }

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = image_callback_group_;

  EraseImageSlots(map_key, image_subs_, compressed_image_subs_);

  static constexpr const char* kImage = "sensor_msgs/msg/Image";
  if (msg_type == kImage) {
    image_subs_[map_key] = create_subscription<sensor_msgs::msg::Image>(
        graph_topic,
        rclcpp::SensorDataQoS(),
        [this, map_key](sensor_msgs::msg::Image::SharedPtr m) {
          OnCameraImage(map_key, std::move(m));
        },
        sub_opts);
    LOGGER_INFO(
        "robot stream image: subscribed sensor_msgs/Image request_topic={} graph_topic={} key={}",
        topic, graph_topic, map_key);
  } else {
    compressed_image_subs_[map_key] = create_subscription<sensor_msgs::msg::CompressedImage>(
        graph_topic,
        rclcpp::SensorDataQoS(),
        [this, map_key](sensor_msgs::msg::CompressedImage::SharedPtr m) {
          OnCompressedImage(map_key, std::move(m));
        },
        sub_opts);
    LOGGER_INFO(
        "robot stream image: subscribed CompressedImage request_topic={} graph_topic={} msg_type={} "
        "key={}",
        topic, graph_topic, msg_type, map_key);
  }
  return true;
}

void RosGuiNode::OnCameraImage(
    const std::string& ros_topic, const sensor_msgs::msg::Image::SharedPtr msg) {
  ros_gui_backend::pb::RobotMessage out;
  ros_gui_backend::pb::ImageFrame* frame = out.mutable_image();
  frame->set_topic(ros_topic);
  frame->set_frame_id(msg->header.frame_id);
  frame->set_width(msg->width);
  frame->set_height(msg->height);
  frame->set_step(msg->step);
  frame->set_stamp_sec(msg->header.stamp.sec);
  frame->set_stamp_nanosec(static_cast<uint32_t>(msg->header.stamp.nanosec));

  const std::string& enc = msg->encoding;
  if (enc == "jpeg" || enc == "jpg" || enc == "png") {
    frame->set_encoding(enc == "png" ? "png" : "jpeg");
    frame->set_data(msg->data.data(), msg->data.size());
  } else {
    try {
      cv::Mat bgr;
      if (enc == "rgb8") {
        cv::Mat rgb(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
            const_cast<uint8_t*>(msg->data.data()), static_cast<size_t>(msg->step));
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      } else if (enc == "bgr8") {
        bgr = cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
            const_cast<uint8_t*>(msg->data.data()), static_cast<size_t>(msg->step));
      } else if (enc == "rgba8") {
        cv::Mat rgba(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4,
            const_cast<uint8_t*>(msg->data.data()), static_cast<size_t>(msg->step));
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
      } else if (enc == "bgra8") {
        cv::Mat bgra(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4,
            const_cast<uint8_t*>(msg->data.data()), static_cast<size_t>(msg->step));
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
      } else if (enc == "mono8") {
        cv::Mat gray(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC1,
            const_cast<uint8_t*>(msg->data.data()), static_cast<size_t>(msg->step));
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      } else if (enc == "yuv422_yuy2" || enc == "yuyv") {
        int w = static_cast<int>(msg->width);
        const int h = static_cast<int>(msg->height);
        if (w < 2) {
          return;
        }
        w &= ~1;
        cv::Mat yuyv(h, w, CV_8UC2, const_cast<uint8_t*>(msg->data.data()),
            static_cast<size_t>(msg->step));
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);
      } else if (enc == "uyvy" || enc == "yuv422_uyvy") {
        int w = static_cast<int>(msg->width);
        const int h = static_cast<int>(msg->height);
        if (w < 2) {
          return;
        }
        w &= ~1;
        cv::Mat uyvy(h, w, CV_8UC2, const_cast<uint8_t*>(msg->data.data()),
            static_cast<size_t>(msg->step));
        cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);
      } else {
        frame->set_encoding(enc);
        frame->set_data(msg->data.data(), msg->data.size());
        RobotMessageHub::Instance().BroadcastRobotMsg(out);
        return;
      }
      std::vector<uint8_t> buf;
      if (!cv::imencode(".jpg", bgr, buf)) {
        return;
      }
      frame->set_encoding("jpeg");
      frame->set_data(buf.data(), buf.size());
    } catch (const std::exception& e) {
      LOGGER_ERROR("error on handle image: {}", e.what());
      return;
    }
  }

  RobotMessageHub::Instance().BroadcastRobotMsg(out);
}

void RosGuiNode::OnCompressedImage(
    const std::string& ros_topic, const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  ros_gui_backend::pb::RobotMessage out;
  ros_gui_backend::pb::ImageFrame* frame = out.mutable_image();
  frame->set_topic(ros_topic);
  frame->set_frame_id(msg->header.frame_id);
  frame->set_stamp_sec(msg->header.stamp.sec);
  frame->set_stamp_nanosec(static_cast<uint32_t>(msg->header.stamp.nanosec));
  frame->set_width(0);
  frame->set_height(0);
  frame->set_step(0);
  std::string fmt = msg->format;
  std::string lower = fmt;
  for (char& c : lower) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  if (lower.find("jpeg") != std::string::npos || lower.find("jpg") != std::string::npos) {
    frame->set_encoding("jpeg");
  } else if (lower.find("png") != std::string::npos) {
    frame->set_encoding("png");
  } else {
    frame->set_encoding(fmt.empty() ? "jpeg" : fmt);
  }
  frame->set_data(msg->data.data(), msg->data.size());
  RobotMessageHub::Instance().BroadcastRobotMsg(out);
}

}  // namespace ros_gui_backend
