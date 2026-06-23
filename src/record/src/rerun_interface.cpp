#include "record/rerun_interface.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace record::rerun_interface {

namespace {

double timestamp_from_header(const std_msgs::msg::Header& header) {
  return rclcpp::Time(header.stamp.sec, header.stamp.nanosec).seconds();
}

std::string joint_name_at(const sensor_msgs::msg::JointState::SharedPtr& msg, size_t index) {
  if (index < msg->name.size() && !msg->name[index].empty()) {
    return msg->name[index];
  }
  return "joint_" + std::to_string(index);
}

}  // namespace

std::vector<uint8_t> compress_image_to_jpeg(const cv::Mat& image, int quality) {
  std::vector<uint8_t> compressed_data;
  std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, quality};
  cv::Mat bgr_image;
  cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
  cv::imencode(".jpg", bgr_image, compressed_data, compression_params);
  return compressed_data;
}

rerun::WidthHeight width_height(const cv::Mat& image) {
  return rerun::WidthHeight(static_cast<size_t>(image.cols), static_cast<size_t>(image.rows));
}

void log_rgb_image(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const sensor_msgs::msg::Image::SharedPtr& msg,
    const ImageOptions& options) {
  rec.set_time_timestamp_secs_since_epoch("timestamp", timestamp_from_header(msg->header));

  try {
    cv::Mat image = cv_bridge::toCvCopy(msg, "rgb8")->image;
    if (options.enable_compression.value_or(true)) {
      const int quality = options.compression_quality.value_or(80);
      const auto compressed_data = compress_image_to_jpeg(image, quality);
      rec.log(
          entity_path,
          rerun::EncodedImage::from_bytes(compressed_data, rerun::components::MediaType::jpeg()));
    } else {
      rec.log(
          entity_path,
          rerun::Image::from_rgb24(
              rerun::Collection<uint8_t>::borrow(image.data, image.total() * image.channels()),
              width_height(image)));
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("record.rerun_interface"), "RGB image log failed: %s", e.what());
  }
}

void log_depth_image(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const sensor_msgs::msg::Image::SharedPtr& msg) {
  rec.set_time_timestamp_secs_since_epoch("timestamp", timestamp_from_header(msg->header));

  try {
    cv::Mat depth_image = cv_bridge::toCvCopy(msg, "16UC1")->image;
    cv::Mat depth_float;
    depth_image.convertTo(depth_float, CV_32F);

    rec.log(
        entity_path,
        rerun::DepthImage(
            rerun::Collection<float>::borrow(
                reinterpret_cast<const float*>(depth_float.data),
                depth_float.total()),
            width_height(depth_float))
            .with_meter(1000.0f));
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("record.rerun_interface"), "Depth image log failed: %s", e.what());
  }
}

void log_joint_states(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const sensor_msgs::msg::JointState::SharedPtr& msg) {
  rec.set_time_timestamp_secs_since_epoch("timestamp", timestamp_from_header(msg->header));

  for (size_t i = 0; i < msg->position.size(); ++i) {
    rec.log(entity_path + "/position/" + joint_name_at(msg, i), rerun::Scalars(msg->position[i]));
  }

  for (size_t i = 0; i < msg->velocity.size(); ++i) {
    rec.log(entity_path + "/velocity/" + joint_name_at(msg, i), rerun::Scalars(msg->velocity[i]));
  }

  for (size_t i = 0; i < msg->effort.size(); ++i) {
    rec.log(entity_path + "/effort/" + joint_name_at(msg, i), rerun::Scalars(msg->effort[i]));
  }
}

void log_text(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const std::string& text,
    double timestamp_sec) {
  rec.set_time_timestamp_secs_since_epoch("timestamp", timestamp_sec);
  rec.log(entity_path, rerun::TextLog(text));
}

}  // namespace record::rerun_interface
