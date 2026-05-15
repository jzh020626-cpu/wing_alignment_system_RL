#include <memory>
#include <vector>
#include <deque>
#include <cmath>
#include <string>
#include <algorithm>
#include <chrono>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

static inline builtin_interfaces::msg::Time time_to_msg(const rclcpp::Time & t)
{
  builtin_interfaces::msg::Time m;
  int64_t ns = t.nanoseconds();
  if (ns < 0) ns = 0;
  m.sec = static_cast<int32_t>(ns / 1000000000LL);
  m.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return m;
}

class QrDeltaPublisher : public rclcpp::Node
{
public:
  QrDeltaPublisher() : Node("qr_delta_publisher")
  {
    declare_parameter<std::string>("topic_pose_in", "object_position");
    declare_parameter<std::string>("topic_delta_out", "wing_alignment/delta");

    declare_parameter<std::string>("out_frame_id", "car_frame");
    declare_parameter<double>("deadband_m", 0.001);
    declare_parameter<double>("max_pub_hz", 30.0);

    declare_parameter<bool>("publish_enabled_on_start", false);
    declare_parameter<bool>("reject_near_zero_pose", true);

    declare_parameter<double>("pose_max_age_sec", 0.25);
    declare_parameter<bool>("enforce_pose_max_age", false);

    declare_parameter<bool>("require_nonzero_stamp", false);
    declare_parameter<std::string>("output_stamp_source", "now");
    declare_parameter<bool>("fallback_to_local_stamp_on_clock_offset", true);

    declare_parameter<double>("future_stamp_tolerance_sec", 1.0);
    declare_parameter<double>("clock_offset_warn_sec", 5.0);
    declare_parameter<double>("max_backward_jump_sec", 0.5);

    declare_parameter<std::string>("publish_mode", "absolute");

    declare_parameter<int>("zero_frames", 10);
    declare_parameter<double>("zero_std_max_m", 0.005);
    declare_parameter<double>("zero_max_wait_sec", 2.0);

    declare_parameter<double>("abs_max_m", 2.0);
    declare_parameter<double>("max_jump_m", 0.15);

    declare_parameter<int>("acquire_frames", 5);
    declare_parameter<double>("acquire_std_max_m", 0.030);
    declare_parameter<int>("reacquire_consistent_frames", 5);
    declare_parameter<double>("duplicate_position_eps", 1e-5);

    declare_parameter<std::vector<double>>(
      "T_camera_to_car",
      std::vector<double>{
        -1, 0, 0, 0.03,
         0,-1, 0,-0.075,
         0, 0, 1,-0.065,
         0, 0, 0, 1
      });

    declare_parameter<std::vector<double>>(
      "constant_R",
      std::vector<double>{
        -1, 0, 0,
         0,-1, 0,
         0, 0, 1
      });

    declare_parameter<std::vector<double>>(
      "vector_v",
      std::vector<double>{0.0, -0.085, 0.0});

    std::string in_topic, out_topic;
    get_parameter("topic_pose_in", in_topic);
    get_parameter("topic_delta_out", out_topic);
    get_parameter("out_frame_id", out_frame_id_);
    get_parameter("deadband_m", deadband_m_);
    get_parameter("max_pub_hz", max_pub_hz_);

    get_parameter("publish_enabled_on_start", enabled_);
    get_parameter("reject_near_zero_pose", reject_near_zero_pose_);

    get_parameter("pose_max_age_sec", pose_max_age_sec_);
    get_parameter("enforce_pose_max_age", enforce_pose_max_age_);

    get_parameter("require_nonzero_stamp", require_nonzero_stamp_);
    get_parameter("output_stamp_source", output_stamp_source_);
    get_parameter("fallback_to_local_stamp_on_clock_offset", fallback_to_local_stamp_on_clock_offset_);

    get_parameter("future_stamp_tolerance_sec", future_stamp_tolerance_sec_);
    get_parameter("clock_offset_warn_sec", clock_offset_warn_sec_);
    get_parameter("max_backward_jump_sec", max_backward_jump_sec_);

    get_parameter("publish_mode", publish_mode_str_);
    publish_mode_ = parse_publish_mode_(publish_mode_str_);

    get_parameter("zero_frames", zero_frames_);
    get_parameter("zero_std_max_m", zero_std_max_m_);
    get_parameter("zero_max_wait_sec", zero_max_wait_sec_);

    get_parameter("abs_max_m", abs_max_m_);
    get_parameter("max_jump_m", max_jump_m_);

    zero_frames_ = std::max(1, zero_frames_);
    zero_std_max_m_ = std::max(1e-6, zero_std_max_m_);
    zero_max_wait_sec_ = std::max(0.1, zero_max_wait_sec_);

    abs_max_m_ = std::max(0.01, abs_max_m_);
    max_jump_m_ = std::max(0.0, max_jump_m_);

    get_parameter("acquire_frames", acquire_frames_);
    get_parameter("acquire_std_max_m", acquire_std_max_m_);
    get_parameter("reacquire_consistent_frames", reacquire_consistent_frames_);
    get_parameter("duplicate_position_eps", duplicate_position_eps_);
    acquire_frames_ = std::max(2, acquire_frames_);
    acquire_std_max_m_ = std::max(1e-6, acquire_std_max_m_);
    reacquire_consistent_frames_ = std::max(2, reacquire_consistent_frames_);

    pose_max_age_sec_ = std::max(0.01, pose_max_age_sec_);
    future_stamp_tolerance_sec_ = std::max(0.0, future_stamp_tolerance_sec_);
    clock_offset_warn_sec_ = std::max(0.0, clock_offset_warn_sec_);
    max_backward_jump_sec_ = std::max(0.0, max_backward_jump_sec_);

    std::transform(
      output_stamp_source_.begin(),
      output_stamp_source_.end(),
      output_stamp_source_.begin(),
      [](unsigned char c) { return std::tolower(c); });

    if (output_stamp_source_ != "now" && output_stamp_source_ != "input") {
      output_stamp_source_ = "now";
    }

    load_matrices_();

    rclcpp::QoS out_qos(rclcpp::KeepLast(1));
    out_qos.best_effort();
    pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(out_topic, out_qos);

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      in_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&QrDeltaPublisher::pose_cb_, this, std::placeholders::_1));

    srv_zero_ = create_service<std_srvs::srv::Trigger>(
      "qr_delta/zero",
      std::bind(&QrDeltaPublisher::zero_srv_, this, std::placeholders::_1, std::placeholders::_2));

    srv_enable_ = create_service<std_srvs::srv::SetBool>(
      "qr_delta/enable",
      std::bind(&QrDeltaPublisher::enable_srv_, this, std::placeholders::_1, std::placeholders::_2));

    srv_reset_ = create_service<std_srvs::srv::Trigger>(
      "qr_delta/reset_tracking",
      std::bind(&QrDeltaPublisher::reset_tracking_srv_, this, std::placeholders::_1, std::placeholders::_2));

    housekeeping_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&QrDeltaPublisher::housekeeping_timer_cb_, this));

    last_pub_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "qr_delta_publisher started. in=%s out=%s deadband=%.3fmm max_pub=%.1fHz ns=%s enabled=%d mode=%s enforce_age=%d age_max=%.3fs",
      in_topic.c_str(),
      out_topic.c_str(),
      deadband_m_ * 1000.0,
      max_pub_hz_,
      this->get_namespace(),
      static_cast<int>(enabled_),
      publish_mode_name_().c_str(),
      static_cast<int>(enforce_pose_max_age_),
      pose_max_age_sec_);
  }

private:
  enum class PublishMode
  {
    ABSOLUTE = 0,
    RELATIVE_AFTER_ZERO = 1
  };

  enum class TrackState { INIT, ACQUIRE, TRACK };

  std::string track_state_name_() const
  {
    switch (track_state_) {
      case TrackState::INIT: return "INIT";
      case TrackState::ACQUIRE: return "ACQUIRE";
      case TrackState::TRACK: return "TRACK";
      default: return "UNKNOWN";
    }
  }

  PublishMode parse_publish_mode_(std::string s)
  {
    std::transform(
      s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return std::tolower(c); });

    if (s == "relative" || s == "relative_after_zero" || s == "zero_relative") {
      return PublishMode::RELATIVE_AFTER_ZERO;
    }
    return PublishMode::ABSOLUTE;
  }

  std::string publish_mode_name_() const
  {
    return (publish_mode_ == PublishMode::ABSOLUTE)
             ? std::string("absolute")
             : std::string("relative_after_zero");
  }

  void load_matrices_()
  {
    auto T = get_parameter("T_camera_to_car").as_double_array();
    if (T.size() != 16) {
      throw std::runtime_error("T_camera_to_car must have 16 elements.");
    }

    T_camera_to_car_.setIdentity();
    T_camera_to_car_.matrix() <<
      T[0],  T[1],  T[2],  T[3],
      T[4],  T[5],  T[6],  T[7],
      T[8],  T[9],  T[10], T[11],
      T[12], T[13], T[14], T[15];

    auto R = get_parameter("constant_R").as_double_array();
    if (R.size() != 9) {
      throw std::runtime_error("constant_R must have 9 elements.");
    }

    constant_R_ <<
      R[0], R[1], R[2],
      R[3], R[4], R[5],
      R[6], R[7], R[8];

    auto v = get_parameter("vector_v").as_double_array();
    if (v.size() != 3) {
      throw std::runtime_error("vector_v must have 3 elements.");
    }

    vector_v_ = Eigen::Vector3d(v[0], v[1], v[2]);
  }

  bool pose_valid_(const geometry_msgs::msg::PoseStamped & msg)
  {
    use_local_stamp_for_current_msg_ = false;
    const auto & p = msg.pose.position;

    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[POSE_INVALID] non-finite position received");
      return false;
    }

    if (
      reject_near_zero_pose_ &&
      std::abs(p.x) < 1e-4 &&
      std::abs(p.y) < 1e-4 &&
      std::abs(p.z) < 1e-4)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[POSE_INVALID] near-zero pose received");
      return false;
    }

    const rclcpp::Time t_msg(msg.header.stamp);
    const rclcpp::Time t_now = this->now();
    if (t_msg.nanoseconds() <= 0) {
      if (require_nonzero_stamp_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[POSE_INVALID] zero timestamp rejected because require_nonzero_stamp=true");
        return false;
      }
      use_local_stamp_for_current_msg_ = true;
      last_input_stamp_ = t_now;
      have_last_input_stamp_ = true;
      return true;
    }

    const double age = (t_now - t_msg).seconds();
    if (!std::isfinite(age)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[POSE_INVALID] non-finite stamp age");
      return false;
    }

    const bool large_clock_offset =
      (clock_offset_warn_sec_ > 0.0) && (std::abs(age) > clock_offset_warn_sec_);

    if (large_clock_offset && fallback_to_local_stamp_on_clock_offset_) {
      use_local_stamp_for_current_msg_ = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[POSE_CLOCK_OFFSET] input stamp differs from local now by %.3fs; fallback to local receive time for downstream timing",
        age);
    } else if (age < -future_stamp_tolerance_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[POSE_INVALID] input stamp too far in future: age=%.3fs tolerance=%.3fs",
        age, future_stamp_tolerance_sec_);
      return false;
    }

    if (have_last_input_stamp_) {
      const rclcpp::Time t_effective = use_local_stamp_for_current_msg_ ? t_now : t_msg;
      const double dt_in = (t_effective - last_input_stamp_).seconds();
      if (dt_in < -max_backward_jump_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[POSE_STAMP_WARN] input stamp backward dt=%.3fs (diagnostic only, not rejecting)",
          dt_in);
      }
    }

    if ((!use_local_stamp_for_current_msg_) && enforce_pose_max_age_ && age > pose_max_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[POSE_INVALID] pose age out of range: age=%.3fs (max=%.3fs)",
        age, pose_max_age_sec_);
      return false;
    }

    if ((!use_local_stamp_for_current_msg_) && (!enforce_pose_max_age_) && std::abs(age) > clock_offset_warn_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[POSE_CLOCK_OFFSET] input stamp differs from local now by %.3fs; accepting stream but check multi-machine clock sync",
        age);
    }

    last_input_stamp_ = use_local_stamp_for_current_msg_ ? t_now : t_msg;
    have_last_input_stamp_ = true;
    return true;
  }

  bool compute_delta_m_(const geometry_msgs::msg::PoseStamped & msg, Eigen::Vector3d & out_delta)
  {
    Eigen::Isometry3d qr_in_camera = Eigen::Isometry3d::Identity();
    qr_in_camera.translation() << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    Eigen::Isometry3d qr_in_car = T_camera_to_car_ * qr_in_camera;
    Eigen::Vector3d target_pos_in_car = qr_in_car.translation();

    Eigen::Quaterniond q(
      msg.pose.orientation.w,
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z
    );

    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[POSE_INVALID] non-finite quaternion");
      return false;
    }

    const double qn = q.norm();
    if (!std::isfinite(qn) || qn < 1e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[POSE_INVALID] invalid quaternion norm=%.6e", qn);
      return false;
    }

    q.normalize();
    Eigen::Matrix3d Rq = q.toRotationMatrix();
    Eigen::Vector3d offset = constant_R_ * Rq * vector_v_;
    out_delta = target_pos_in_car + offset;
    return out_delta.allFinite();
  }

  void zero_stats_(Eigen::Vector3d & mean, Eigen::Vector3d & stdv)
  {
    mean = Eigen::Vector3d::Zero();
    stdv = Eigen::Vector3d::Zero();

    if (zero_buf_.empty()) {
      return;
    }

    for (const auto & v : zero_buf_) {
      mean += v;
    }
    mean /= static_cast<double>(zero_buf_.size());

    Eigen::Vector3d var = Eigen::Vector3d::Zero();
    for (const auto & v : zero_buf_) {
      Eigen::Vector3d d = v - mean;
      var.x() += d.x() * d.x();
      var.y() += d.y() * d.y();
      var.z() += d.z() * d.z();
    }

    var /= std::max(1.0, static_cast<double>(zero_buf_.size()));
    stdv.x() = std::sqrt(std::max(0.0, var.x()));
    stdv.y() = std::sqrt(std::max(0.0, var.y()));
    stdv.z() = std::sqrt(std::max(0.0, var.z()));
  }

  rclcpp::Time choose_stamp_(const geometry_msgs::msg::PoseStamped & in, const rclcpp::Time & now_t)
  {
    if (!use_local_stamp_for_current_msg_ && output_stamp_source_ == "input") {
      const rclcpp::Time t_msg(in.header.stamp);
      if (t_msg.nanoseconds() > 0) {
        return t_msg;
      }
    }
    return now_t;
  }

  void start_zero_capture_if_needed_(const rclcpp::Time & t_now)
  {
    if (!zero_collecting_) {
      zero_collecting_ = true;
      zero_start_time_ = t_now;
      zero_buf_.clear();
      RCLCPP_WARN(
        get_logger(),
        "[ZERO] begin stable capture (mode=%s, zero_frames=%d, zero_std_max=%.4fm, timeout=%.2fs)",
        publish_mode_name_().c_str(), zero_frames_, zero_std_max_m_, zero_max_wait_sec_);
    }
  }

  void check_zero_capture_step_(const Eigen::Vector3d & delta_m, const rclcpp::Time & t_now)
  {
    start_zero_capture_if_needed_(t_now);

    zero_buf_.push_back(delta_m);
    while (static_cast<int>(zero_buf_.size()) > zero_frames_) {
      zero_buf_.pop_front();
    }

    Eigen::Vector3d mu, sd;
    zero_stats_(mu, sd);

    const int min_required_frames = std::min(5, zero_frames_);
    const bool has_enough_frames = (static_cast<int>(zero_buf_.size()) >= min_required_frames);

    const bool stable = has_enough_frames &&
                        (sd.x() <= zero_std_max_m_) &&
                        (sd.y() <= zero_std_max_m_) &&
                        (sd.z() <= zero_std_max_m_);

    if (stable) {
      zero_m_ = mu;
      zero_set_ = true;
      zero_collecting_ = false;
      zero_buf_.clear();
      have_last_rel_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Zero captured successfully: mean=[%.4f %.4f %.4f] std=[%.4f %.4f %.4f]",
        zero_m_.x(), zero_m_.y(), zero_m_.z(),
        sd.x(), sd.y(), sd.z());
    }
  }

  void housekeeping_timer_cb_()
  {
    // periodic diagnostic (every ~5s at 100ms timer)
    if (enabled_ && ++diag_counter_ >= 50) {
      diag_counter_ = 0;
      RCLCPP_INFO(get_logger(),
        "[DIAG] state=%s valid=%d dup=%d jump_streak=%d acq_buf=%zu "
        "ref=[%.4f %.4f %.4f]",
        track_state_name_().c_str(),
        total_valid_count_, duplicate_count_, jump_reject_streak_,
        acquire_buf_.size(),
        last_rel_.x(), last_rel_.y(), last_rel_.z());
    }

    if (!enabled_) {
      return;
    }

    if (publish_mode_ != PublishMode::RELATIVE_AFTER_ZERO) {
      return;
    }

    if (zero_set_ || !zero_collecting_) {
      return;
    }

    const rclcpp::Time t_now = now();
    const double elapsed = (t_now - zero_start_time_).seconds();
    if (elapsed < zero_max_wait_sec_) {
      return;
    }

    Eigen::Vector3d mu, sd;
    zero_stats_(mu, sd);

    RCLCPP_ERROR(
      get_logger(),
      "Zero capture TIMEOUT due to unstable/missing valid data "
      "(buf=%zu, mean=[%.4f %.4f %.4f], std=[%.4f %.4f %.4f], timeout=%.2fs). Restarting capture!",
      zero_buf_.size(),
      mu.x(), mu.y(), mu.z(),
      sd.x(), sd.y(), sd.z(),
      zero_max_wait_sec_);

    zero_collecting_ = false;
    zero_buf_.clear();
    have_last_rel_ = false;
  }

  void cluster_stats_(const std::deque<Eigen::Vector3d> & buf,
                      Eigen::Vector3d & mean, Eigen::Vector3d & stdv)
  {
    mean = Eigen::Vector3d::Zero();
    stdv = Eigen::Vector3d::Zero();
    if (buf.empty()) return;
    for (const auto & v : buf) mean += v;
    mean /= static_cast<double>(buf.size());
    Eigen::Vector3d var = Eigen::Vector3d::Zero();
    for (const auto & v : buf) {
      Eigen::Vector3d d = v - mean;
      var.x() += d.x() * d.x();
      var.y() += d.y() * d.y();
      var.z() += d.z() * d.z();
    }
    var /= std::max(1.0, static_cast<double>(buf.size()));
    stdv.x() = std::sqrt(std::max(0.0, var.x()));
    stdv.y() = std::sqrt(std::max(0.0, var.y()));
    stdv.z() = std::sqrt(std::max(0.0, var.z()));
  }

  void publish_delta_(const Eigen::Vector3d & out_m,
                      const geometry_msgs::msg::PoseStamped & msg,
                      const rclcpp::Time & t_now)
  {
    if (std::abs(out_m.x()) < deadband_m_ &&
        std::abs(out_m.y()) < deadband_m_ &&
        std::abs(out_m.z()) < deadband_m_) {
      last_pub_time_ = t_now;
      return;
    }
    geometry_msgs::msg::Vector3Stamped out;
    rclcpp::Time t_out = choose_stamp_(msg, t_now);
    out.header.stamp = time_to_msg(t_out);
    out.header.frame_id = out_frame_id_;
    out.vector.x = out_m.x();
    out.vector.y = out_m.y();
    out.vector.z = out_m.z();
    pub_->publish(out);
    last_pub_time_ = t_now;
  }

  void reset_tracking_state_()
  {
    track_state_ = TrackState::INIT;
    acquire_buf_.clear();
    jump_candidate_buf_.clear();
    jump_reject_streak_ = 0;
    have_last_rel_ = false;
    have_last_raw_delta_ = false;
    duplicate_count_ = 0;
    total_valid_count_ = 0;
  }

  void pose_cb_(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!enabled_) {
      return;
    }
    if (!pose_valid_(*msg)) {
      return;
    }

    const rclcpp::Time t_now = now();
    last_rx_time_ = t_now;
    total_valid_count_++;

    const double min_dt = (max_pub_hz_ > 1e-6) ? (1.0 / max_pub_hz_) : 0.0;
    if ((t_now - last_pub_time_).seconds() < min_dt) {
      return;
    }

    Eigen::Vector3d delta_m = Eigen::Vector3d::Zero();
    if (!compute_delta_m_(*msg, delta_m)) {
      return;
    }

    if (std::abs(delta_m.x()) > abs_max_m_ ||
        std::abs(delta_m.y()) > abs_max_m_ ||
        std::abs(delta_m.z()) > abs_max_m_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[DELTA_DROP] absolute out of bounds: [%.3f %.3f %.3f] max=%.3f",
        delta_m.x(), delta_m.y(), delta_m.z(), abs_max_m_);
      return;
    }

    // --- duplicate detection on raw computed delta ---
    if (have_last_raw_delta_) {
      if ((delta_m - last_raw_delta_).norm() < duplicate_position_eps_) {
        duplicate_count_++;
        last_pub_time_ = t_now;
        return;
      }
    }
    last_raw_delta_ = delta_m;
    have_last_raw_delta_ = true;

    // --- compute output (absolute or relative-after-zero) ---
    Eigen::Vector3d out_m = Eigen::Vector3d::Zero();
    if (publish_mode_ == PublishMode::RELATIVE_AFTER_ZERO) {
      if (!zero_set_) {
        check_zero_capture_step_(delta_m, t_now);
        last_pub_time_ = t_now;
        return;
      }
      out_m = delta_m - zero_m_;
    } else {
      out_m = delta_m;
    }

    // ========== INIT / ACQUIRE / TRACK state machine ==========
    switch (track_state_) {

    case TrackState::INIT:
      track_state_ = TrackState::ACQUIRE;
      acquire_buf_.clear();
      acquire_buf_.push_back(out_m);
      RCLCPP_INFO(get_logger(),
        "[TRACK] INIT -> ACQUIRE first_frame=[%.4f %.4f %.4f]",
        out_m.x(), out_m.y(), out_m.z());
      publish_delta_(out_m, *msg, t_now);
      break;

    case TrackState::ACQUIRE: {
      acquire_buf_.push_back(out_m);
      while (static_cast<int>(acquire_buf_.size()) > acquire_frames_) {
        acquire_buf_.pop_front();
      }

      // always publish during acquire — don't block downstream
      publish_delta_(out_m, *msg, t_now);

      // check if cluster is stable enough to enter TRACK
      if (static_cast<int>(acquire_buf_.size()) >= acquire_frames_) {
        Eigen::Vector3d mu, sd;
        cluster_stats_(acquire_buf_, mu, sd);
        if (sd.x() <= acquire_std_max_m_ &&
            sd.y() <= acquire_std_max_m_ &&
            sd.z() <= acquire_std_max_m_) {
          last_rel_ = mu;
          have_last_rel_ = true;
          jump_reject_streak_ = 0;
          jump_candidate_buf_.clear();
          track_state_ = TrackState::TRACK;
          RCLCPP_WARN(get_logger(),
            "[TRACK] ACQUIRE -> TRACK mean=[%.4f %.4f %.4f] std=[%.4f %.4f %.4f]",
            mu.x(), mu.y(), mu.z(), sd.x(), sd.y(), sd.z());
          acquire_buf_.clear();
        }
      }
      break;
    }

    case TrackState::TRACK: {
      if (max_jump_m_ > 1e-9 && have_last_rel_) {
        const double jump = (out_m - last_rel_).norm();
        if (jump > max_jump_m_) {
          // jump detected — might be spike or scene change
          jump_reject_streak_++;
          jump_candidate_buf_.push_back(out_m);
          while (static_cast<int>(jump_candidate_buf_.size()) > reacquire_consistent_frames_) {
            jump_candidate_buf_.pop_front();
          }

          // check if rejected frames form a consistent new cluster
          if (static_cast<int>(jump_candidate_buf_.size()) >= reacquire_consistent_frames_) {
            Eigen::Vector3d cm, cs;
            cluster_stats_(jump_candidate_buf_, cm, cs);
            if (cs.x() <= acquire_std_max_m_ &&
                cs.y() <= acquire_std_max_m_ &&
                cs.z() <= acquire_std_max_m_) {
              // consistent new cluster = scene change → reacquire
              RCLCPP_WARN(get_logger(),
                "[TRACK] TRACK -> ACQUIRE scene_change streak=%d "
                "old_ref=[%.4f %.4f %.4f] new=[%.4f %.4f %.4f] std=[%.4f %.4f %.4f]",
                jump_reject_streak_,
                last_rel_.x(), last_rel_.y(), last_rel_.z(),
                cm.x(), cm.y(), cm.z(), cs.x(), cs.y(), cs.z());
              track_state_ = TrackState::ACQUIRE;
              acquire_buf_ = jump_candidate_buf_;
              jump_candidate_buf_.clear();
              jump_reject_streak_ = 0;
              publish_delta_(out_m, *msg, t_now);
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[DELTA_DROP] jump=%.3f>max=%.3f streak=%d candidates_noisy std=[%.4f %.4f %.4f]",
                jump, max_jump_m_, jump_reject_streak_, cs.x(), cs.y(), cs.z());
            }
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
              "[DELTA_DROP] jump=%.3f>max=%.3f streak=%d/%d "
              "ref=[%.4f %.4f %.4f] cur=[%.4f %.4f %.4f]",
              jump, max_jump_m_, jump_reject_streak_, reacquire_consistent_frames_,
              last_rel_.x(), last_rel_.y(), last_rel_.z(),
              out_m.x(), out_m.y(), out_m.z());
          }
          last_pub_time_ = t_now;
          return;
        }
      }

      // normal tracking frame
      jump_reject_streak_ = 0;
      jump_candidate_buf_.clear();
      last_rel_ = out_m;
      have_last_rel_ = true;
      publish_delta_(out_m, *msg, t_now);
      break;
    }

    } // switch
  }

  void zero_srv_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    enabled_ = true;
    reset_tracking_state_();

    if (publish_mode_ == PublishMode::RELATIVE_AFTER_ZERO) {
      zero_set_ = false;
      zero_collecting_ = false;
      zero_buf_.clear();
      res->success = true;
      res->message = "zero reset; capturing stable mean...";
      RCLCPP_WARN(get_logger(), "[ZERO] service accepted: relative_after_zero mode, restart zero capture + tracking reset");
    } else {
      zero_set_ = false;
      zero_collecting_ = false;
      zero_buf_.clear();
      res->success = true;
      res->message = "absolute mode active; zero ignored, publishing absolute delta";
      RCLCPP_WARN(get_logger(), "[ZERO] service accepted in absolute mode: tracking reset, publish absolute delta");
    }
  }

  void reset_tracking_srv_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_WARN(get_logger(),
      "[RESET_TRACKING] state=%s -> INIT (clearing all tracking state)",
      track_state_name_().c_str());
    reset_tracking_state_();
    res->success = true;
    res->message = "tracking reset to INIT";
  }

  void enable_srv_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    enabled_ = static_cast<bool>(req->data);

    if (!enabled_) {
      zero_set_ = false;
      zero_collecting_ = false;
      zero_buf_.clear();
      reset_tracking_state_();
    }

    res->success = true;
    res->message = enabled_ ? "enabled" : "disabled";
    RCLCPP_WARN(
      get_logger(),
      "[ENABLE] %s (mode=%s, enforce_age=%d, track=%s)",
      enabled_ ? "enabled" : "disabled",
      publish_mode_name_().c_str(),
      static_cast<int>(enforce_pose_max_age_),
      track_state_name_().c_str());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_zero_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr housekeeping_timer_;

  Eigen::Isometry3d T_camera_to_car_;
  Eigen::Matrix3d constant_R_;
  Eigen::Vector3d vector_v_;

  std::string out_frame_id_;
  double deadband_m_{0.001};
  double max_pub_hz_{30.0};

  bool enabled_{false};
  bool reject_near_zero_pose_{true};

  double pose_max_age_sec_{0.25};
  bool enforce_pose_max_age_{false};

  bool require_nonzero_stamp_{false};
  std::string output_stamp_source_{"now"};
  bool fallback_to_local_stamp_on_clock_offset_{true};

  double future_stamp_tolerance_sec_{1.0};
  double clock_offset_warn_sec_{5.0};
  double max_backward_jump_sec_{0.5};

  std::string publish_mode_str_{"absolute"};
  PublishMode publish_mode_{PublishMode::ABSOLUTE};

  int zero_frames_{10};
  double zero_std_max_m_{0.005};
  double zero_max_wait_sec_{2.0};

  bool zero_set_{false};
  bool zero_collecting_{false};
  rclcpp::Time zero_start_time_;
  std::deque<Eigen::Vector3d> zero_buf_;
  Eigen::Vector3d zero_m_{Eigen::Vector3d::Zero()};

  double abs_max_m_{2.0};
  double max_jump_m_{0.15};

  int acquire_frames_{5};
  double acquire_std_max_m_{0.030};
  int reacquire_consistent_frames_{5};
  double duplicate_position_eps_{1e-5};

  TrackState track_state_{TrackState::INIT};
  std::deque<Eigen::Vector3d> acquire_buf_;
  std::deque<Eigen::Vector3d> jump_candidate_buf_;
  int jump_reject_streak_{0};

  bool have_last_raw_delta_{false};
  Eigen::Vector3d last_raw_delta_{Eigen::Vector3d::Zero()};
  int duplicate_count_{0};
  int total_valid_count_{0};

  bool have_last_rel_{false};
  Eigen::Vector3d last_rel_{Eigen::Vector3d::Zero()};

  bool have_last_input_stamp_{false};
  bool use_local_stamp_for_current_msg_{false};
  rclcpp::Time last_input_stamp_{0, 0, RCL_SYSTEM_TIME};

  rclcpp::Time last_pub_time_;
  rclcpp::Time last_rx_time_;

  int diag_counter_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QrDeltaPublisher>());
  rclcpp::shutdown();
  return 0;
}
