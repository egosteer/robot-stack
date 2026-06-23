#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

struct ImageOptions {
  std::optional<bool> enable_compression = true;
  std::optional<int> compression_quality = 80;
};

namespace record::rerun_interface {

void log_rgb_image(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const sensor_msgs::msg::Image::SharedPtr& msg,
    const ImageOptions& options);

void log_depth_image(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const sensor_msgs::msg::Image::SharedPtr& msg);

void log_joint_states(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const sensor_msgs::msg::JointState::SharedPtr& msg);

void log_text(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const std::string& text,
    double timestamp_sec);

std::vector<uint8_t> compress_image_to_jpeg(const cv::Mat& image, int quality);
rerun::WidthHeight width_height(const cv::Mat& image);

}  // namespace record::rerun_interface
