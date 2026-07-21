#include "record/record_node.hpp"
#include "record/rerun_interface.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <regex>
#include <sstream>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

RecordNode::RecordNode() : Node("record_node") {
  initialize_parameters();

  std::filesystem::create_directories(data_folder_);
  current_episode_id_ = find_next_episode_id();

  setup_subscriptions();
  setup_services();

  data_processing_thread_ = std::make_unique<std::thread>(&RecordNode::data_processing_worker, this);

  RCLCPP_INFO(this->get_logger(), "Record node ready. data_folder=%s next_episode=%06d",
              data_folder_.c_str(), current_episode_id_);
}

RecordNode::~RecordNode() {
  std::string ignored;
  if (recording_) {
    stop_recording(true, ignored);
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    should_stop_ = true;
  }
  queue_condition_.notify_all();

  if (data_processing_thread_ && data_processing_thread_->joinable()) {
    data_processing_thread_->join();
  }
}

void RecordNode::initialize_parameters() {
  this->declare_parameter("data_folder", "recordings");
  this->declare_parameter("enable_compression", true);
  this->declare_parameter("compression_quality", 80);
  this->declare_parameter("queue_size", 1000);

  data_folder_ = this->get_parameter("data_folder").as_string();
  enable_compression_ = this->get_parameter("enable_compression").as_bool();
  compression_quality_ = this->get_parameter("compression_quality").as_int();
  queue_size_ = this->get_parameter("queue_size").as_int();
  if (queue_size_ < 1) {
    queue_size_ = 1;
  }
}

void RecordNode::setup_subscriptions() {
  auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;

  auto add_joint_sub = [this, &options](const std::string& topic) {
    joint_subs_.push_back(this->create_subscription<JointState>(
        topic,
        queue_size_,
        [this, topic](const JointState::SharedPtr msg) { joint_callback(msg, topic); },
        options));
  };

  auto add_image_sub = [this, &options](const std::string& topic, bool depth) {
    rclcpp::QoS image_qos(rclcpp::KeepLast(static_cast<size_t>(queue_size_)));
    image_qos.best_effort();
    image_subs_.push_back(this->create_subscription<Image>(
        topic,
        image_qos,
        [this, topic, depth](const Image::SharedPtr msg) {
          if (depth) {
            depth_callback(msg, topic);
          } else {
            rgb_callback(msg, topic);
          }
        },
        options));
  };

  auto add_text_sub = [this, &options](const std::string& topic) {
    text_subs_.push_back(this->create_subscription<String>(
        topic,
        10,
        [this, topic](const String::SharedPtr msg) { text_callback(msg, topic); },
        options));
  };

  add_joint_sub("/action/left_hand/joints");
  add_joint_sub("/action/right_hand/joints");
  add_joint_sub("/state/left_hand/joints");
  add_joint_sub("/state/right_hand/joints");

  add_joint_sub("/action/left_arm/joints");
  add_joint_sub("/action/right_arm/joints");
  add_joint_sub("/state/left_arm/joints");
  add_joint_sub("/state/right_arm/joints");

  add_image_sub("/camera/head/rgb", false);
  add_image_sub("/camera/head/depth", true);
  add_image_sub("/camera/chest/rgb", false);
  add_image_sub("/camera/chest/depth", true);

  add_text_sub("/commander");
  add_text_sub("/interface/instruction");
}

void RecordNode::setup_services() {
  toggle_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/toggle_recording",
      std::bind(&RecordNode::handle_toggle_recording, this, std::placeholders::_1, std::placeholders::_2));

  discard_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/discard_recording",
      std::bind(&RecordNode::handle_discard_recording, this, std::placeholders::_1, std::placeholders::_2));
}

void RecordNode::joint_callback(const JointState::SharedPtr msg, const std::string& entity_path) {
  enqueue_task([msg, entity_path](const rerun::RecordingStream& rec) {
    record::rerun_interface::log_joint_states(rec, entity_path, msg);
  });
}

void RecordNode::rgb_callback(const Image::SharedPtr msg, const std::string& entity_path) {
  ImageOptions image_options;
  image_options.enable_compression = enable_compression_;
  image_options.compression_quality = compression_quality_;

  enqueue_task([msg, entity_path, image_options](const rerun::RecordingStream& rec) {
    record::rerun_interface::log_rgb_image(rec, entity_path, msg, image_options);
  });
}

void RecordNode::depth_callback(const Image::SharedPtr msg, const std::string& entity_path) {
  enqueue_task([msg, entity_path](const rerun::RecordingStream& rec) {
    record::rerun_interface::log_depth_image(rec, entity_path, msg);
  });
}

void RecordNode::text_callback(const String::SharedPtr msg, const std::string& entity_path) {
  const double timestamp_sec = this->now().seconds();
  enqueue_task([msg, entity_path, timestamp_sec](const rerun::RecordingStream& rec) {
    record::rerun_interface::log_text(rec, entity_path, msg->data, timestamp_sec);
  });
}

void RecordNode::handle_toggle_recording(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (request->data) {
    response->success = start_recording(response->message);
  } else {
    response->success = stop_recording(true, response->message);
  }
}

void RecordNode::handle_discard_recording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  response->success = discard_recording(response->message);
}

bool RecordNode::start_recording(std::string& message) {
  std::lock_guard<std::mutex> lock(recording_mutex_);

  if (recording_) {
    message = "recording is already running";
    return false;
  }

  current_recording_file_ = episode_path(current_episode_id_);
  last_recording_discardable_ = false;
  last_recording_file_.clear();

  rec_ = std::make_unique<rerun::RecordingStream>("egosteer_record");
  auto save_result = rec_->save(current_recording_file_);
  if (save_result.is_err()) {
    message = "failed to save recording: " + save_result.description;
    rec_.reset();
    current_recording_file_.clear();
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return false;
  }

  recording_ = true;
  log_recording_time("/recording_info/start_time");

  message = current_recording_file_;
  RCLCPP_INFO(this->get_logger(), "Started recording: %s", current_recording_file_.c_str());
  return true;
}

bool RecordNode::stop_recording(bool keep_file, std::string& message) {
  std::lock_guard<std::mutex> lock(recording_mutex_);

  if (!recording_) {
    message = "recording is not running";
    return false;
  }

  const std::string file_path = current_recording_file_;
  recording_ = false;
  wait_for_queue_empty();

  if (rec_) {
    log_recording_time(keep_file ? "/recording_info/end_time" : "/recording_info/discard_time");
    rec_->flush_blocking();
    rec_.reset();
  }

  current_recording_file_.clear();

  if (keep_file) {
    last_recording_file_ = file_path;
    last_recording_discardable_ = true;
    current_episode_id_ += 1;
    message = file_path;
    RCLCPP_INFO(this->get_logger(), "Stopped recording: %s", file_path.c_str());
    return true;
  }

  last_recording_file_.clear();
  last_recording_discardable_ = false;
  return delete_recording_file(file_path, message);
}

bool RecordNode::discard_recording(std::string& message) {
  std::unique_lock<std::mutex> lock(recording_mutex_);
  if (recording_) {
    lock.unlock();
    return stop_recording(false, message);
  }

  if (!last_recording_discardable_ || last_recording_file_.empty()) {
    message = "no recent recording to discard";
    return false;
  }

  const std::string file_path = last_recording_file_;
  last_recording_file_.clear();
  last_recording_discardable_ = false;

  const int deleted_episode_id = episode_id_from_path(file_path);
  if (!delete_recording_file(file_path, message)) {
    last_recording_file_ = file_path;
    last_recording_discardable_ = true;
    return false;
  }

  if (deleted_episode_id >= 0 && current_episode_id_ == deleted_episode_id + 1) {
    current_episode_id_ = deleted_episode_id;
  }
  return true;
}

bool RecordNode::delete_recording_file(const std::string& file_path, std::string& message) {
  try {
    if (file_path.empty()) {
      message = "recording path is empty";
      return false;
    }

    if (!std::filesystem::exists(file_path)) {
      message = "recording file does not exist: " + file_path;
      return false;
    }

    std::filesystem::remove(file_path);
    message = file_path;
    RCLCPP_INFO(this->get_logger(), "Discarded recording: %s", file_path.c_str());
    return true;
  } catch (const std::exception& e) {
    message = "failed to delete recording: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return false;
  }
}

void RecordNode::enqueue_task(std::function<void(const rerun::RecordingStream&)> task) {
  if (!recording_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!recording_) {
      return;
    }
    task_queue_.push(std::move(task));
  }
  queue_condition_.notify_one();
}

void RecordNode::data_processing_worker() {
  while (true) {
    std::function<void(const rerun::RecordingStream&)> task;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait(lock, [this] { return should_stop_ || !task_queue_.empty(); });

      if (should_stop_ && task_queue_.empty()) {
        break;
      }

      task = std::move(task_queue_.front());
      task_queue_.pop();
      active_tasks_ += 1;
    }

    try {
      if (rec_) {
        task(*rec_);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "record task failed: %s", e.what());
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      active_tasks_ -= 1;
      if (task_queue_.empty() && active_tasks_ == 0) {
        queue_drained_condition_.notify_all();
      }
    }
  }
}

void RecordNode::wait_for_queue_empty() {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  queue_drained_condition_.wait(lock, [this] {
    return task_queue_.empty() && active_tasks_ == 0;
  });
}

int RecordNode::find_next_episode_id() const {
  int max_id = -1;
  const std::regex episode_pattern(R"(episode_(\d+)\.rrd)");

  if (!std::filesystem::exists(data_folder_)) {
    return 0;
  }

  for (const auto& entry : std::filesystem::directory_iterator(data_folder_)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    std::smatch match;
    const std::string filename = entry.path().filename().string();
    if (std::regex_match(filename, match, episode_pattern)) {
      max_id = std::max(max_id, std::stoi(match[1].str()));
    }
  }

  return max_id + 1;
}

int RecordNode::episode_id_from_path(const std::string& file_path) const {
  const std::regex episode_pattern(R"(episode_(\d+)\.rrd)");
  std::smatch match;
  const std::string filename = std::filesystem::path(file_path).filename().string();
  if (!std::regex_match(filename, match, episode_pattern)) {
    return -1;
  }
  return std::stoi(match[1].str());
}

std::string RecordNode::episode_path(int episode_id) const {
  std::stringstream ss;
  ss << data_folder_ << "/episode_" << std::setfill('0') << std::setw(6) << episode_id << ".rrd";
  return ss.str();
}

void RecordNode::log_recording_time(const std::string& entity_path) {
  const double timestamp_sec = this->now().seconds();
  const auto now = std::chrono::system_clock::now();
  const auto time_t_now = std::chrono::system_clock::to_time_t(now);
  record::rerun_interface::log_text(*rec_, entity_path, std::ctime(&time_t_now), timestamp_sec);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecordNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
