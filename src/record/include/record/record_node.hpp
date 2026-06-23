#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rerun.hpp>

class RecordNode : public rclcpp::Node {
public:
  RecordNode();
  ~RecordNode() override;

private:
  using JointState = sensor_msgs::msg::JointState;
  using Image = sensor_msgs::msg::Image;
  using String = std_msgs::msg::String;

  void initialize_parameters();
  void setup_subscriptions();
  void setup_services();

  void joint_callback(const JointState::SharedPtr msg, const std::string& entity_path);
  void rgb_callback(const Image::SharedPtr msg, const std::string& entity_path);
  void depth_callback(const Image::SharedPtr msg, const std::string& entity_path);
  void text_callback(const String::SharedPtr msg, const std::string& entity_path);

  void handle_toggle_recording(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void handle_discard_recording(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool start_recording(std::string& message);
  bool stop_recording(bool keep_file, std::string& message);
  bool discard_recording(std::string& message);
  bool delete_recording_file(const std::string& file_path, std::string& message);

  void enqueue_task(std::function<void(const rerun::RecordingStream&)> task);
  void data_processing_worker();
  void wait_for_queue_empty();

  int find_next_episode_id() const;
  int episode_id_from_path(const std::string& file_path) const;
  std::string episode_path(int episode_id) const;
  void log_recording_time(const std::string& entity_path);

  std::string data_folder_;
  bool enable_compression_{true};
  int compression_quality_{80};
  int queue_size_{1000};

  std::mutex recording_mutex_;
  std::unique_ptr<rerun::RecordingStream> rec_;
  std::atomic<bool> recording_{false};
  int current_episode_id_{0};
  std::string current_recording_file_;
  std::string last_recording_file_;
  bool last_recording_discardable_{false};

  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::condition_variable queue_drained_condition_;
  std::queue<std::function<void(const rerun::RecordingStream&)>> task_queue_;
  size_t active_tasks_{0};
  bool should_stop_{false};
  std::unique_ptr<std::thread> data_processing_thread_;

  std::vector<rclcpp::Subscription<JointState>::SharedPtr> joint_subs_;
  std::vector<rclcpp::Subscription<Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<String>::SharedPtr> text_subs_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr discard_service_;
};
