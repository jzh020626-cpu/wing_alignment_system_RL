#include <memory>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <Eigen/Dense>

class AdaptiveKalmanFilter {
public:
  AdaptiveKalmanFilter(double R=0.005, double Qs=1e-5, double Qd=0.5)
  : R_(R), Qs_(Qs), Qd_(Qd), Q_(Qs), x_(0.0), P_(1.0), initialized_(false) {}

  double update(double z) {
    if (!initialized_) {
      x_ = z;
      P_ = 1.0;
      initialized_ = true;
      return x_;
    }

    double x_pred = x_;
    double P_pred = P_ + Q_;
    double innov = z - x_pred;

    Q_ = (std::abs(innov) > 5.0 * std::sqrt(std::max(1e-12, R_))) ? Qd_ : Qs_;
    P_pred = P_ + Q_;

    double S = P_pred + R_;
    double K = P_pred / std::max(1e-12, S);

    x_ = x_pred + K * innov;
    P_ = (1.0 - K) * P_pred;
    return x_;
  }

  void reset(double x0=0.0) {
    x_ = x0;
    P_ = 1.0;
    Q_ = Qs_;
    initialized_ = false;
  }

private:
  double R_, Qs_, Qd_, Q_;
  double x_, P_;
  bool initialized_;
};

class ForceMonitor : public rclcpp::Node
{
public:
  ForceMonitor() : Node("force_monitor")
  {
    declare_parameter<std::string>("topic_force_in", "/huatai1_force");
    declare_parameter<std::string>("topic_stop_out", "/wing_alignment/emergency_stop");

    declare_parameter<int>("calib_frames", 50);
    declare_parameter<double>("threshold", 50.0);
    declare_parameter<double>("release_ratio", 0.5);
    declare_parameter<bool>("latched", true);

    declare_parameter<int>("trigger_count", 3);
    declare_parameter<int>("release_count", 3);

    declare_parameter<double>("stop_pub_hz", 10.0);

    declare_parameter<bool>("use_norm_threshold", false);
    declare_parameter<double>("threshold_norm", 0.0);

    declare_parameter<std::string>("topic_force_filtered_out", "");
    declare_parameter<double>("filtered_pub_hz", 30.0);

    declare_parameter<bool>("calib_guard_enable", true);
    declare_parameter<double>("calib_guard_ratio", 3.0);
    declare_parameter<int>("calib_guard_count", 2);

    declare_parameter<bool>("force_timeout_enable", false);
    declare_parameter<double>("force_timeout_sec", 0.5);
    declare_parameter<double>("force_physical_max_axis", 1000.0);
    declare_parameter<double>("force_physical_max_norm", 1200.0);
    declare_parameter<double>("force_spike_jump_norm", 400.0);

    get_parameter("calib_frames", calib_frames_);
    get_parameter("threshold", threshold_);
    get_parameter("release_ratio", release_ratio_);
    get_parameter("latched", latched_);
    get_parameter("trigger_count", trigger_count_);
    get_parameter("release_count", release_count_);
    get_parameter("stop_pub_hz", stop_pub_hz_);

    get_parameter("use_norm_threshold", use_norm_threshold_);
    get_parameter("threshold_norm", threshold_norm_);

    get_parameter("topic_force_filtered_out", topic_force_filtered_out_);
    get_parameter("filtered_pub_hz", filtered_pub_hz_);

    get_parameter("calib_guard_enable", calib_guard_enable_);
    get_parameter("calib_guard_ratio", calib_guard_ratio_);
    get_parameter("calib_guard_count", calib_guard_count_);

    get_parameter("force_timeout_enable", force_timeout_enable_);
    get_parameter("force_timeout_sec", force_timeout_sec_);
    get_parameter("force_physical_max_axis", force_physical_max_axis_);
    get_parameter("force_physical_max_norm", force_physical_max_norm_);
    get_parameter("force_spike_jump_norm", force_spike_jump_norm_);

    calib_frames_ = std::max(1, calib_frames_);
    trigger_count_ = std::max(1, trigger_count_);
    release_count_ = std::max(1, release_count_);
    stop_pub_hz_ = std::max(0.5, stop_pub_hz_);

    filtered_pub_hz_ = std::max(1.0, filtered_pub_hz_);
    calib_guard_ratio_ = std::max(1.0, calib_guard_ratio_);
    calib_guard_count_ = std::max(1, calib_guard_count_);

    force_timeout_sec_ = std::max(0.05, force_timeout_sec_);
    force_physical_max_axis_ = std::max(1.0, force_physical_max_axis_);
    force_physical_max_norm_ = std::max(force_physical_max_axis_, force_physical_max_norm_);
    force_spike_jump_norm_ = std::max(0.0, force_spike_jump_norm_);

    if (threshold_norm_ <= 1e-9) threshold_norm_ = threshold_;

    std::string tin, tout;
    get_parameter("topic_force_in", tin);
    get_parameter("topic_stop_out", tout);

    rclcpp::QoS stop_qos(rclcpp::KeepLast(1));
    stop_qos.reliable();
    stop_qos.transient_local();

    pub_stop_ = create_publisher<std_msgs::msg::Bool>(tout, stop_qos);

    sub_force_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      tin, rclcpp::SensorDataQoS(),
      std::bind(&ForceMonitor::cb_, this, std::placeholders::_1));

    srv_reset_ = create_service<std_srvs::srv::Trigger>(
      "force_monitor/reset",
      std::bind(&ForceMonitor::reset_srv_, this, std::placeholders::_1, std::placeholders::_2));

    if (!topic_force_filtered_out_.empty()) {
      pub_filtered_ = create_publisher<std_msgs::msg::Float32MultiArray>(topic_force_filtered_out_, 10);
      timer_filtered_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / filtered_pub_hz_),
        std::bind(&ForceMonitor::timer_filtered_cb_, this));
    }

    filters_.emplace_back(0.005, 1e-5, 0.5);
    filters_.emplace_back(0.005, 1e-5, 0.5);
    filters_.emplace_back(0.005, 1e-5, 0.5);

    timer_pub_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / stop_pub_hz_),
      std::bind(&ForceMonitor::timer_pub_cb_, this));

    RCLCPP_INFO(
      get_logger(),
      "force_monitor started. in=%s out=%s calib=%d thr=%.2f rel=%.2f latched=%d trigN=%d relN=%d pub=%.1fHz timeout_en=%d timeout=%.2fs",
      tin.c_str(), tout.c_str(),
      calib_frames_, threshold_, threshold_ * release_ratio_, static_cast<int>(latched_),
      trigger_count_, release_count_, stop_pub_hz_,
      static_cast<int>(force_timeout_enable_), force_timeout_sec_);

    set_stop_state_(false);
  }
private:
  void reset_srv_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    calibrated_ = false;
    triggered_ = false;
    buf_.clear();
    over_cnt_ = 0;
    under_cnt_ = 0;
    calib_guard_over_cnt_ = 0;
    last_filtered_ = Eigen::Vector3d::Zero();
    offset_ = Eigen::Vector3d::Zero();
    have_valid_force_msg_ = false;
    have_last_valid_raw_ = false;
    last_force_msg_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_valid_raw_ = Eigen::Vector3d::Zero();
    for (auto & f : filters_) f.reset(0.0);
    set_stop_state_(false);
    res->success = true;
    res->message = "force monitor reset";
    RCLCPP_WARN(get_logger(), "Force monitor reset.");
  }

  void set_stop_state_(bool v)
  {
    triggered_ = v;
    std_msgs::msg::Bool m;
    m.data = v;
    pub_stop_->publish(m);
  }

  void timer_pub_cb_()
  {
    if (force_timeout_enable_ && have_valid_force_msg_) {
      const double age = (now() - last_force_msg_time_).seconds();
      if (age > force_timeout_sec_) {
        if (!triggered_) {
          set_stop_state_(true);
          RCLCPP_ERROR(get_logger(), "[FORCE_TIMEOUT] no valid force data for %.3fs (> %.3fs) -> STOP!", age, force_timeout_sec_);
        }
      }
    }
    std_msgs::msg::Bool m;
    m.data = triggered_;
    pub_stop_->publish(m);
  }

  void timer_filtered_cb_()
  {
    if (!pub_filtered_) return;
    std_msgs::msg::Float32MultiArray m;
    m.data.resize(4);
    m.data[0] = static_cast<float>(last_filtered_.x());
    m.data[1] = static_cast<float>(last_filtered_.y());
    m.data[2] = static_cast<float>(last_filtered_.z());
    m.data[3] = static_cast<float>(last_filtered_.norm());
    pub_filtered_->publish(m);
  }

  Eigen::Vector3d mean_of_(const std::deque<Eigen::Vector3d>& q) const
  {
    if (q.empty()) return Eigen::Vector3d::Zero();
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto & v : q) sum += v;
    return sum / static_cast<double>(q.size());
  }

  bool finite_vec3_(const Eigen::Vector3d & v) const
  {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  bool valid_force_sample_(const Eigen::Vector3d & raw)
  {
    if (!finite_vec3_(raw)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "[FORCE_INVALID] non-finite raw force received -> ignore");
      return false;
    }

    const double raw_norm = raw.norm();
    if ((!std::isfinite(raw_norm)) ||
        (std::abs(raw.x()) > force_physical_max_axis_) ||
        (std::abs(raw.y()) > force_physical_max_axis_) ||
        (std::abs(raw.z()) > force_physical_max_axis_) ||
        (raw_norm > force_physical_max_norm_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[FORCE_INVALID] raw force exceeds physical limit raw=[%.2f %.2f %.2f] norm=%.2f -> ignore",
        raw.x(), raw.y(), raw.z(), raw_norm);
      return false;
    }

    if (have_last_valid_raw_) {
      const double jump_norm = (raw - last_valid_raw_).norm();
      if (std::isfinite(jump_norm) && jump_norm > force_spike_jump_norm_) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[FORCE_INVALID] raw force spike rejected raw=[%.2f %.2f %.2f] prev=[%.2f %.2f %.2f] jump=%.2f",
          raw.x(), raw.y(), raw.z(), last_valid_raw_.x(), last_valid_raw_.y(), last_valid_raw_.z(), jump_norm);
        return false;
      }
    }

    return true;
  }

  bool is_over_threshold_(const Eigen::Vector3d & v, double axis_thr, double norm_thr) const
  {
    if (use_norm_threshold_) {
      return v.norm() > norm_thr;
    }
    return (std::abs(v.x()) > axis_thr || std::abs(v.y()) > axis_thr || std::abs(v.z()) > axis_thr);
  }

  bool is_under_release_(const Eigen::Vector3d & v, double axis_rel, double norm_rel) const
  {
    if (use_norm_threshold_) {
      return v.norm() < norm_rel;
    }
    return (std::abs(v.x()) < axis_rel && std::abs(v.y()) < axis_rel && std::abs(v.z()) < axis_rel);
  }

  void cb_(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[FORCE_INVALID] incoming force array size=%zu < 3", msg->data.size());
      return;
    }
    Eigen::Vector3d raw(msg->data[0], msg->data[1], msg->data[2]);
    if (!valid_force_sample_(raw)) {
      return;
    }
    have_last_valid_raw_ = true;
    last_valid_raw_ = raw;
    have_valid_force_msg_ = true;
    last_force_msg_time_ = now();
    if (!calibrated_) {
      if (calib_guard_enable_ && static_cast<int>(buf_.size()) >= 3) {
        const Eigen::Vector3d mu_hist = mean_of_(buf_);
        const Eigen::Vector3d r = raw - mu_hist;
        const bool calib_over = is_over_threshold_(r, calib_guard_ratio_ * threshold_, calib_guard_ratio_ * threshold_norm_);
        if (calib_over) calib_guard_over_cnt_++; else calib_guard_over_cnt_ = 0;
        if (calib_over) {
          if (calib_guard_over_cnt_ >= calib_guard_count_) {
            calib_guard_over_cnt_ = 0;
            buf_.clear();
            if (!triggered_) {
              set_stop_state_(true);
              RCLCPP_ERROR(get_logger(), "[CALIB_GUARD] anomaly during calibration -> STOP! raw=[%.2f %.2f %.2f] mu_hist=[%.2f %.2f %.2f]", raw.x(), raw.y(), raw.z(), mu_hist.x(), mu_hist.y(), mu_hist.z());
            }
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[CALIB_GUARD] suspicious sample rejected during calibration");
          }
          return;
        }
      }
      buf_.push_back(raw);
      if (static_cast<int>(buf_.size()) >= calib_frames_) {
        offset_ = mean_of_(buf_);
        if (!finite_vec3_(offset_)) {
          RCLCPP_ERROR(get_logger(), "[CALIB] computed non-finite offset -> calibration reset");
          buf_.clear(); calibrated_ = false; for (auto & f : filters_) f.reset(0.0); return;
        }
        buf_.clear(); calibrated_ = true; for (auto & f : filters_) f.reset(0.0);
        RCLCPP_INFO(get_logger(), "Calibrated offset = [%.2f %.2f %.2f]", offset_.x(), offset_.y(), offset_.z());
      }
      return;
    }
    Eigen::Vector3d z = raw - offset_;
    if (!finite_vec3_(z)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "[FORCE_INVALID] raw-offset became non-finite -> ignore");
      return;
    }
    Eigen::Vector3d f;
    for (int i = 0; i < 3; ++i) f[i] = filters_[i].update(z[i]);
    if (!finite_vec3_(f)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "[FORCE_INVALID] filtered force became non-finite -> STOP!");
      if (!triggered_) set_stop_state_(true);
      return;
    }
    last_filtered_ = f;
    const double thr_axis = threshold_;
    const double thr_norm = threshold_norm_;
    const double rel_axis = threshold_ * release_ratio_;
    const double rel_norm = threshold_norm_ * release_ratio_;
    const bool over = is_over_threshold_(f, thr_axis, thr_norm);
    const bool under_release = is_under_release_(f, rel_axis, rel_norm);
    if (!triggered_) {
      if (over) over_cnt_++; else over_cnt_ = 0;
      if (over_cnt_ >= trigger_count_) {
        over_cnt_ = 0; under_cnt_ = 0; set_stop_state_(true);
        RCLCPP_WARN(get_logger(), "Force STOP triggered! filtered=[%.2f %.2f %.2f] norm=%.2f", f.x(), f.y(), f.z(), f.norm());
      }
      return;
    }
    if (!latched_) {
      if (under_release) under_cnt_++; else under_cnt_ = 0;
      if (under_cnt_ >= release_count_) {
        under_cnt_ = 0; over_cnt_ = 0; set_stop_state_(false); RCLCPP_INFO(get_logger(), "Force STOP released.");
      }
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_force_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_filtered_;
  rclcpp::TimerBase::SharedPtr timer_filtered_;
  std::string topic_force_filtered_out_;
  double filtered_pub_hz_{30.0};
  Eigen::Vector3d last_filtered_{Eigen::Vector3d::Zero()};
  int calib_frames_{50};
  double threshold_{50.0};
  double release_ratio_{0.5};
  bool latched_{true};
  int trigger_count_{3};
  int release_count_{3};
  double stop_pub_hz_{10.0};
  bool use_norm_threshold_{false};
  double threshold_norm_{50.0};
  bool calib_guard_enable_{true};
  double calib_guard_ratio_{3.0};
  int calib_guard_count_{2};
  int calib_guard_over_cnt_{0};
  bool force_timeout_enable_{false};
  double force_timeout_sec_{0.5};
  double force_physical_max_axis_{1000.0};
  double force_physical_max_norm_{1200.0};
  double force_spike_jump_norm_{400.0};
  bool have_valid_force_msg_{false};
  bool have_last_valid_raw_{false};
  rclcpp::Time last_force_msg_time_{0, 0, RCL_SYSTEM_TIME};
  Eigen::Vector3d last_valid_raw_{Eigen::Vector3d::Zero()};
  bool calibrated_{false};
  bool triggered_{false};
  int over_cnt_{0};
  int under_cnt_{0};
  std::deque<Eigen::Vector3d> buf_;
  Eigen::Vector3d offset_{Eigen::Vector3d::Zero()};
  std::vector<AdaptiveKalmanFilter> filters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceMonitor>());
  rclcpp::shutdown();
  return 0;
}
