// ─────────────────────────────────────────────────────────────────
// visp_servo_node — Stop-sign approach (v4: translation-only, sim-aware)
//
// 4-state machine:
//
//   SEARCHING ──[lock-on: dets≥N AND |ex|<lockon_ex]──▶ APPROACHING
//       ▲                                                  │
//       │ (reacquire timeout, w/ last-known bearing)       │
//       │                                                  │
//   REACQUIRE ◀──[detection drops in APPROACHING]──────────┤
//       │                                                  │
//       │ [detection returns]                              │
//       └─────────────────────────────────────────────▶    │
//                                                          ▼
//                                   [close + centered]──▶ REACHED (sticky)
//
// Filters applied at publish time (not states):
//   - SIM_STALL override   : /sim/heartbeat not advancing → cmd = 0
//   - Floor clamp          : drone_z < safety_min_altitude → vz ≥ 0
//   - Velocity ramp        : smooth all axes at vel_ramp_rate per tick
//
// Coordinate convention (Simulink-confirmed):
//   linear.x  = forward velocity  (+ = forward,  - = back)
//   linear.y  = lateral velocity  (+ = LEFT,     - = RIGHT)
//   linear.z  = vertical velocity (+ = UP,       - = DOWN)
//   angular.y = pitch RATE        (+ = NOSE DOWN)
//   angular.z = yaw   RATE        (+ = YAW LEFT)
//
// Yaw is used ONLY during SEARCHING. APPROACHING/REACQUIRE never command wz.
// Centering during approach is achieved via vy (lateral) and vz (vertical) only.
// ─────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <deque>
#include <string>
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>

using DetectionArray = yolo_msgs::msg::DetectionArray;
using Detection      = yolo_msgs::msg::Detection;
using Image          = sensor_msgs::msg::Image;
using Twist          = geometry_msgs::msg::Twist;
using Float64        = std_msgs::msg::Float64;
using Float64MA      = std_msgs::msg::Float64MultiArray;

// ─────────────────────────────────────────────────────────────────
class VispServoNode : public rclcpp::Node {
public:
    VispServoNode() : Node("visp_servo_node") {
        declare_parameters();
        load_parameters();
        setup_ros();

        std::string cls_dump;
        for (size_t i = 0; i < target_classes_.size(); ++i) {
            if (i) cls_dump += ", ";
            cls_dump += "\"" + target_classes_[i] + "\"";
        }
        RCLCPP_INFO(get_logger(),
            "visp_servo_node v4 — translation-only — targets=[%s] (conf>=%.2f)",
            cls_dump.c_str(), min_confidence_);
        RCLCPP_INFO(get_logger(),
            "Gains: k_fwd=%.2f k_lat=%.2f k_vz=%.2f k_pitch_return=%.2f",
            k_fwd_, k_lat_, k_vz_, k_pitch_return_);
        RCLCPP_INFO(get_logger(),
            "Caps: lin=%.2fm/s ang=%.2frad/s ramp=%.2fm/s/tick",
            max_linear_, max_angular_, vel_ramp_rate_);
        RCLCPP_INFO(get_logger(),
            "Search: yaw=±%.0f° k=%.2f strafe=%.1fm/s for %.1fs",
            search_yaw_target_deg_, k_search_yaw_, search_strafe_speed_, search_strafe_dur_sec_);
    }

private:
    // ── State machine ─────────────────────────────────────────
    enum class State { SEARCHING, APPROACHING, REACQUIRE, REACHED };

    static const char* state_name(State s) {
        switch (s) {
            case State::SEARCHING:   return "SEARCHING";
            case State::APPROACHING: return "APPROACHING";
            case State::REACQUIRE:   return "REACQUIRE";
            case State::REACHED:     return "REACHED";
        }
        return "UNKNOWN";
    }

    // Sub-states of SEARCHING — deterministic sequence
    enum class SearchStep { YAW_RIGHT_60, YAW_LEFT_60, YAW_CENTER, STRAFE_RIGHT };
    static const char* search_step_name(SearchStep s) {
        switch (s) {
            case SearchStep::YAW_RIGHT_60:  return "YAW_RIGHT_60";
            case SearchStep::YAW_LEFT_60:   return "YAW_LEFT_60";
            case SearchStep::YAW_CENTER:    return "YAW_CENTER";
            case SearchStep::STRAFE_RIGHT:  return "STRAFE_RIGHT";
        }
        return "?";
    }

    // ── Parameters ───────────────────────────────────────────
    std::string              target_class_;
    std::vector<std::string> target_classes_;
    double min_confidence_;
    int    image_width_, image_height_;
    bool   debug_image_enabled_;

    // Control gains
    double k_fwd_, k_lat_, k_vz_;
    double vert_deadband_;
    double k_pitch_return_, pitch_zero_tol_;

    // Limits
    double max_linear_, max_angular_;
    double vel_ramp_rate_;

    // Approach targets
    double target_bbox_ratio_;
    double hold_bbox_ratio_;
    double hold_center_tol_;
    int    reach_consec_ticks_;

    // Lock-on
    int    lockon_consec_;
    double lockon_ex_tol_;          // |ex_norm| must be below this to lock
    double k_lockon_bias_;          // edge-detection pulls search yaw target

    // Search
    double search_yaw_target_deg_;
    double k_search_yaw_;
    double search_yaw_arrive_tol_rad_;
    double search_settle_sec_;
    double search_strafe_speed_;
    double search_strafe_dur_sec_;

    // REACQUIRE (adaptive timeout)
    double base_reacquire_sec_;
    double k_persist_;
    double max_reacquire_sec_;

    // Sim liveness watchdog
    int    heartbeat_window_samples_;
    double heartbeat_gap_sec_;
    double heartbeat_dead_sec_;
    double sim_paused_rate_;
    double sim_recovery_debounce_sec_;
    double sim_unhealthy_entry_sec_;

    // Floor safety
    double safety_min_altitude_;
    double safety_pose_max_age_;

    // Misc
    double watchdog_period_sec_;
    double diag_period_sec_;
    int    smoothing_window_;
    double debug_min_draw_conf_;

    // ── Runtime state ────────────────────────────────────────
    State state_ = State::SEARCHING;

    // Search sub-state
    SearchStep    search_step_      = SearchStep::YAW_RIGHT_60;
    double        yaw_target_rad_   = 0.0;
    bool          yaw_target_init_  = false;
    rclcpp::Time  search_arrive_time_;        // when we entered settle
    bool          search_arrived_   = false;  // currently in settle window?
    rclcpp::Time  strafe_start_time_;

    // Detection bookkeeping
    int           consecutive_dets_   = 0;
    rclcpp::Time  approach_start_time_;
    int           reach_ticks_        = 0;
    double        last_bearing_to_sign_rad_ = 0.0; // for handoff to SEARCH after REACQUIRE timeout
    bool          have_last_bearing_  = false;

    // REACQUIRE
    rclcpp::Time  reacquire_start_time_;
    double        active_reacquire_timeout_sec_ = 0.0;

    // Smoothing
    std::deque<double> cx_hist_, cy_hist_, bw_hist_, bh_hist_;
    double prev_cx_ = -1.0, prev_cy_ = -1.0;

    // Pitch (ground truth, only for camera-level pitch return)
    double       current_pitch_rad_   = 0.0;
    rclcpp::Time last_pitch_msg_time_;
    bool         have_pitch_msg_      = false;

    // Drone pose (ground truth — yaw for search + z for floor clamp; never control feedback)
    double drone_x_ = 0.0, drone_y_ = 15.0, drone_z_ = 10.0;
    double drone_yaw_rad_ = 0.0;
    rclcpp::Time last_drone_pose_time_;
    bool   have_drone_pose_ = false;

    // Target pose (instrumentation only — never in control)
    double target_x_ = 0.0, target_y_ = 0.0, target_z_ = 0.0, target_yaw_ = 0.0;
    bool   have_target_pose_ = false;

    // Heartbeat — sim liveness signal
    struct HbSample { double sim_t; rclcpp::Time wall_t; };
    std::deque<HbSample> hb_buffer_;
    rclcpp::Time         last_hb_wall_time_;
    bool                 have_hb_           = false;
    bool                 sim_healthy_       = true;
    double               last_hb_sim_       = 0.0;
    // Streak tracking: when the most recent run of healthy/unhealthy
    // observations began. Used to debounce both directions of the flip.
    bool                 prev_obs_healthy_       = true;
    rclcpp::Time         healthy_streak_start_;
    rclcpp::Time         unhealthy_streak_start_;

    // Previous published command (for ramp)
    Twist prev_cmd_{};

    // Debug image
    cv::Mat                       latest_frame_;
    DetectionArray::SharedPtr     latest_dets_;

    // ── ROS interfaces ───────────────────────────────────────
    rclcpp::Subscription<DetectionArray>::SharedPtr sub_dets_;
    rclcpp::Subscription<Image>::SharedPtr          sub_image_;
    rclcpp::Subscription<Image>::SharedPtr          sub_sim_cam_;
    rclcpp::Subscription<Float64>::SharedPtr        sub_pitch_;
    rclcpp::Subscription<Float64MA>::SharedPtr      sub_drone_pose_;
    rclcpp::Subscription<Float64MA>::SharedPtr      sub_target_pose_;
    rclcpp::Subscription<Float64>::SharedPtr        sub_heartbeat_;

    rclcpp::Publisher<Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<Image>::SharedPtr pub_debug_;

    rclcpp::TimerBase::SharedPtr diag_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Diagnostics
    rclcpp::Time diag_window_start_;
    int sim_cam_count_ = 0;
    int ovcam_count_   = 0;
    int det_count_     = 0;
    int pitch_count_   = 0;
    int pose_count_    = 0;
    int hb_count_      = 0;
    int cmd_count_     = 0;
    rclcpp::Time last_cmd_publish_time_;
    rclcpp::Time last_det_callback_time_;
    bool have_det_callback_ = false;

    rclcpp::Time last_log_time_;

    // ─────────────────────────────────────────────────────────
    void declare_parameters() {
        declare_parameter("target_class",      std::string("stop sign"));
        declare_parameter("min_confidence",     0.30);
        declare_parameter("image_width",        640);
        declare_parameter("image_height",       480);

        declare_parameter("k_fwd",              1.5);
        declare_parameter("k_lat",              0.5);
        declare_parameter("k_vz",               0.5);
        declare_parameter("vert_deadband",      0.08);

        declare_parameter("k_pitch_return",     1.0);
        declare_parameter("pitch_zero_tol",     0.02);

        declare_parameter("max_linear",         1.5);
        declare_parameter("max_angular",        0.8);
        declare_parameter("vel_ramp_rate",      0.45);

        declare_parameter("target_bbox_ratio",  0.75);
        declare_parameter("hold_bbox_ratio",    0.70);
        declare_parameter("hold_center_tol",    0.12);
        declare_parameter("reach_consec_ticks", 8);

        declare_parameter("lockon_consec",      2);
        declare_parameter("lockon_ex_tol",      0.5);
        declare_parameter("k_lockon_bias",      0.5);

        declare_parameter("search_yaw_target_deg",     60.0);
        declare_parameter("k_search_yaw",              1.0);
        declare_parameter("search_yaw_arrive_tol_rad", 0.05);
        declare_parameter("search_settle_sec",         0.5);
        declare_parameter("search_strafe_speed",       0.6);
        declare_parameter("search_strafe_dur_sec",     3.0);

        declare_parameter("base_reacquire_sec",  1.0);
        declare_parameter("k_persist",           0.3);
        declare_parameter("max_reacquire_sec",   5.0);

        declare_parameter("heartbeat_window_samples",   20);
        declare_parameter("heartbeat_gap_sec",          0.2);
        declare_parameter("heartbeat_dead_sec",         1.5);
        declare_parameter("sim_paused_rate",            0.01);
        declare_parameter("sim_recovery_debounce_sec",  0.5);
        declare_parameter("sim_unhealthy_entry_sec",    1.0);

        declare_parameter("safety_min_altitude",  0.5);
        declare_parameter("safety_pose_max_age",  2.0);

        declare_parameter("watchdog_period_sec",  0.20);
        declare_parameter("diag_period_sec",      2.0);

        declare_parameter("smoothing_window",     5);
        declare_parameter("debug_min_draw_conf",  0.15);
        declare_parameter("debug_image_enabled",  true);
    }

    void load_parameters() {
        target_class_ = get_parameter("target_class").as_string();
        target_classes_.clear();
        for (size_t start = 0; start <= target_class_.size(); ) {
            size_t comma = target_class_.find(',', start);
            if (comma == std::string::npos) comma = target_class_.size();
            std::string tok = target_class_.substr(start, comma - start);
            size_t a = tok.find_first_not_of(" \t");
            size_t b = tok.find_last_not_of(" \t");
            if (a != std::string::npos) target_classes_.push_back(tok.substr(a, b - a + 1));
            start = comma + 1;
        }
        min_confidence_  = get_parameter("min_confidence").as_double();
        image_width_          = get_parameter("image_width").as_int();
        image_height_         = get_parameter("image_height").as_int();
        debug_image_enabled_  = get_parameter("debug_image_enabled").as_bool();

        k_fwd_           = get_parameter("k_fwd").as_double();
        k_lat_           = get_parameter("k_lat").as_double();
        k_vz_            = get_parameter("k_vz").as_double();
        vert_deadband_   = get_parameter("vert_deadband").as_double();
        k_pitch_return_  = get_parameter("k_pitch_return").as_double();
        pitch_zero_tol_  = get_parameter("pitch_zero_tol").as_double();

        max_linear_      = get_parameter("max_linear").as_double();
        max_angular_     = get_parameter("max_angular").as_double();
        vel_ramp_rate_   = get_parameter("vel_ramp_rate").as_double();

        target_bbox_ratio_  = get_parameter("target_bbox_ratio").as_double();
        hold_bbox_ratio_    = get_parameter("hold_bbox_ratio").as_double();
        hold_center_tol_    = get_parameter("hold_center_tol").as_double();
        reach_consec_ticks_ = get_parameter("reach_consec_ticks").as_int();

        lockon_consec_      = get_parameter("lockon_consec").as_int();
        lockon_ex_tol_      = get_parameter("lockon_ex_tol").as_double();
        k_lockon_bias_      = get_parameter("k_lockon_bias").as_double();

        search_yaw_target_deg_       = get_parameter("search_yaw_target_deg").as_double();
        k_search_yaw_                = get_parameter("k_search_yaw").as_double();
        search_yaw_arrive_tol_rad_   = get_parameter("search_yaw_arrive_tol_rad").as_double();
        search_settle_sec_           = get_parameter("search_settle_sec").as_double();
        search_strafe_speed_         = get_parameter("search_strafe_speed").as_double();
        search_strafe_dur_sec_       = get_parameter("search_strafe_dur_sec").as_double();

        base_reacquire_sec_  = get_parameter("base_reacquire_sec").as_double();
        k_persist_           = get_parameter("k_persist").as_double();
        max_reacquire_sec_   = get_parameter("max_reacquire_sec").as_double();

        heartbeat_window_samples_  = get_parameter("heartbeat_window_samples").as_int();
        heartbeat_gap_sec_         = get_parameter("heartbeat_gap_sec").as_double();
        heartbeat_dead_sec_        = get_parameter("heartbeat_dead_sec").as_double();
        sim_paused_rate_           = get_parameter("sim_paused_rate").as_double();
        sim_recovery_debounce_sec_ = get_parameter("sim_recovery_debounce_sec").as_double();
        sim_unhealthy_entry_sec_   = get_parameter("sim_unhealthy_entry_sec").as_double();

        safety_min_altitude_  = get_parameter("safety_min_altitude").as_double();
        safety_pose_max_age_  = get_parameter("safety_pose_max_age").as_double();

        watchdog_period_sec_  = get_parameter("watchdog_period_sec").as_double();
        diag_period_sec_      = get_parameter("diag_period_sec").as_double();

        smoothing_window_     = get_parameter("smoothing_window").as_int();
        debug_min_draw_conf_  = get_parameter("debug_min_draw_conf").as_double();
    }

    // ─────────────────────────────────────────────────────────
    void setup_ros() {
        auto qos_be  = rclcpp::QoS(1).best_effort();
        auto qos_rel = rclcpp::QoS(2).reliable();
        auto qos_tl  = rclcpp::QoS(1).reliable().transient_local();

        sub_dets_ = create_subscription<DetectionArray>(
            "/yolo/detections", qos_be,
            std::bind(&VispServoNode::on_detections, this, std::placeholders::_1));

        if (debug_image_enabled_)
            sub_image_ = create_subscription<Image>(
                "/ovcam/image_raw", qos_rel,
                std::bind(&VispServoNode::on_image, this, std::placeholders::_1));

        sub_sim_cam_ = create_subscription<Image>(
            "/sim/camera/image_raw", qos_be,
            [this](const Image::SharedPtr) { sim_cam_count_++; });

        sub_pitch_ = create_subscription<Float64>(
            "/sim/pitch_angle", rclcpp::QoS(10),
            std::bind(&VispServoNode::on_pitch, this, std::placeholders::_1));

        sub_drone_pose_ = create_subscription<Float64MA>(
            "/sim/drone_pose", rclcpp::QoS(10),
            std::bind(&VispServoNode::on_drone_pose, this, std::placeholders::_1));

        sub_target_pose_ = create_subscription<Float64MA>(
            "/sim/target_pose", qos_tl,
            std::bind(&VispServoNode::on_target_pose, this, std::placeholders::_1));

        sub_heartbeat_ = create_subscription<Float64>(
            "/sim/heartbeat", rclcpp::QoS(10),
            std::bind(&VispServoNode::on_heartbeat, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<Twist>("/cmd_vel", 10);
        if (debug_image_enabled_)
            pub_debug_ = create_publisher<Image>("/visp/debug_image", qos_be);

        auto t0 = now();
        last_log_time_          = t0;
        last_pitch_msg_time_    = t0;
        last_cmd_publish_time_  = t0;
        last_det_callback_time_ = t0;
        last_drone_pose_time_   = t0;
        last_hb_wall_time_      = t0;
        healthy_streak_start_   = t0;
        unhealthy_streak_start_ = t0;
        diag_window_start_      = t0;

        watchdog_timer_ = create_wall_timer(
            std::chrono::duration<double>(watchdog_period_sec_),
            std::bind(&VispServoNode::watchdog_tick, this));

        diag_timer_ = create_wall_timer(
            std::chrono::duration<double>(diag_period_sec_),
            std::bind(&VispServoNode::diag_tick, this));

        reset_search();
    }

    // ─────────────────────────────────────────────────────────
    // Callbacks
    // ─────────────────────────────────────────────────────────
    void on_image(const Image::SharedPtr msg) {
        if (!debug_image_enabled_) return;
        ovcam_count_++;
        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, "mono8");
            cv::cvtColor(cv_ptr->image, latest_frame_, cv::COLOR_GRAY2BGR);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "cv_bridge error: %s", e.what());
            return;
        }
        publish_debug_image();
    }

    void on_drone_pose(const Float64MA::SharedPtr msg) {
        if (msg->data.size() < 5) return;
        drone_x_       = msg->data[0];
        drone_y_       = msg->data[1];
        drone_z_       = msg->data[2];
        // pitch at [3] — already provided more cleanly via /sim/pitch_angle
        drone_yaw_rad_ = msg->data[4];
        last_drone_pose_time_ = now();
        have_drone_pose_      = true;
        pose_count_++;
    }

    void on_target_pose(const Float64MA::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        target_x_   = msg->data[0];
        target_y_   = msg->data[1];
        target_z_   = msg->data[2];
        target_yaw_ = msg->data[3];
        have_target_pose_ = true;
    }

    void on_pitch(const Float64::SharedPtr msg) {
        current_pitch_rad_   = msg->data;
        last_pitch_msg_time_ = now();
        have_pitch_msg_      = true;
        pitch_count_++;
    }

    void on_heartbeat(const Float64::SharedPtr msg) {
        auto t_now = now();

        // Detect simulation restart: sim_time jumped backward by a meaningful
        // amount. Happens when the user stops + restarts Simulink. We reset
        // controller state so the drone re-engages from SEARCHING instead of
        // sitting in REACHED (or APPROACHING with stale geometry).
        if (have_hb_ && msg->data < last_hb_sim_ - 0.5) {
            RCLCPP_WARN(get_logger(),
                "Sim restart detected: sim_time %.2f -> %.2f — resetting controller",
                last_hb_sim_, msg->data);
            handle_sim_restart();
        }

        // Gap detection — if wall gap from previous heartbeat > threshold, drop
        // the window and start fresh (avoids computing sim_rate across a gap).
        if (have_hb_) {
            double wall_gap = (t_now - last_hb_wall_time_).seconds();
            if (wall_gap > heartbeat_gap_sec_) {
                hb_buffer_.clear();
            }
        }
        hb_buffer_.push_back({msg->data, t_now});
        while (static_cast<int>(hb_buffer_.size()) > heartbeat_window_samples_) {
            hb_buffer_.pop_front();
        }
        last_hb_wall_time_ = t_now;
        last_hb_sim_       = msg->data;
        have_hb_           = true;
        hb_count_++;
    }

    void handle_sim_restart() {
        // Force state machine back to SEARCHING from any state (REACHED,
        // APPROACHING, REACQUIRE). Clear all motion-history so the drone
        // doesn't lurch from stale ramp anchors.
        state_ = State::SEARCHING;
        consecutive_dets_   = 0;
        reach_ticks_        = 0;
        prev_cx_ = prev_cy_ = -1.0;
        cx_hist_.clear();
        cy_hist_.clear();
        bw_hist_.clear();
        bh_hist_.clear();
        prev_cmd_           = Twist{};
        have_last_bearing_  = false;

        // Heartbeat buffer holds samples from the OLD run — discard.
        hb_buffer_.clear();
        sim_healthy_            = true;
        prev_obs_healthy_       = true;
        healthy_streak_start_   = now();
        unhealthy_streak_start_ = now();

        // Re-anchor search from current (post-reset) drone yaw. Note: drone_yaw_
        // may briefly be stale for one tick until /sim/drone_pose republishes;
        // that's harmless because the search yaw target is updated lazily.
        reset_search();
    }

    // ─────────────────────────────────────────────────────────
    // Sim liveness — returns true if Simulink is actively stepping.
    // ─────────────────────────────────────────────────────────
    enum class SimHealth { HEALTHY, PAUSED, DEAD };

    SimHealth compute_sim_health() {
        if (!have_hb_) {
            // Until we've seen the first heartbeat, treat as healthy (grace period
            // at startup). The diag_tick will warn loudly if it never arrives.
            return SimHealth::HEALTHY;
        }
        double wall_age = (now() - last_hb_wall_time_).seconds();
        if (wall_age > heartbeat_dead_sec_) return SimHealth::DEAD;

        if (static_cast<int>(hb_buffer_.size()) < 3) {
            // Just recovered from a gap — not enough samples to judge rate yet.
            return SimHealth::HEALTHY;
        }
        double d_sim  = hb_buffer_.back().sim_t      - hb_buffer_.front().sim_t;
        double d_wall = (hb_buffer_.back().wall_t    - hb_buffer_.front().wall_t).seconds();
        if (d_wall < 1e-3) return SimHealth::HEALTHY;
        double rate = d_sim / d_wall;
        if (rate < sim_paused_rate_) return SimHealth::PAUSED;
        return SimHealth::HEALTHY;
    }

    void update_sim_health() {
        SimHealth h = compute_sim_health();
        bool obs_healthy = (h == SimHealth::HEALTHY);
        auto t = now();

        // Track when each streak (healthy / unhealthy observations) began.
        if (obs_healthy != prev_obs_healthy_) {
            if (obs_healthy) healthy_streak_start_   = t;
            else             unhealthy_streak_start_ = t;
            prev_obs_healthy_ = obs_healthy;
        }

        // Bidirectional debounce — only flip after sustaining the new
        // observation for its debounce window. Avoids flicker when sim_rate
        // sits near the paused threshold.
        if (obs_healthy && !sim_healthy_) {
            if ((t - healthy_streak_start_).seconds() >= sim_recovery_debounce_sec_) {
                sim_healthy_ = true;
                RCLCPP_WARN(get_logger(),
                    "Sim health RECOVERED (sim_time=%.3f) — resuming control",
                    last_hb_sim_);
            }
        } else if (!obs_healthy && sim_healthy_) {
            if ((t - unhealthy_streak_start_).seconds() >= sim_unhealthy_entry_sec_) {
                sim_healthy_ = false;
                RCLCPP_ERROR(get_logger(),
                    "Sim health LOST: %s (sim_time=%.3f, wall_age=%.2fs) — cmd_vel held at 0",
                    h == SimHealth::DEAD ? "HEARTBEAT_DEAD" : "SIM_PAUSED",
                    last_hb_sim_, (t - last_hb_wall_time_).seconds());
            }
        }
    }

    // ─────────────────────────────────────────────────────────
    // Pitch — read-only signal for camera-level return.
    // ─────────────────────────────────────────────────────────
    double pitch_for_control() {
        if (have_pitch_msg_) {
            double age = (now() - last_pitch_msg_time_).seconds();
            if (age > 2.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "/sim/pitch_angle stale (%.1fs) — using %.3f rad",
                    age, current_pitch_rad_);
            }
        }
        return current_pitch_rad_;
    }

    double pitch_return_cmd() {
        double pitch = pitch_for_control();
        if (std::abs(pitch) < pitch_zero_tol_) return 0.0;
        return -k_pitch_return_ * pitch;
    }

    // ─────────────────────────────────────────────────────────
    // Main detection callback — drives state machine + control.
    // ─────────────────────────────────────────────────────────
    void on_detections(const DetectionArray::SharedPtr msg) {
        det_count_++;
        latest_dets_ = msg;

        if (have_det_callback_) {
            double gap = (now() - last_det_callback_time_).seconds();
            if (gap > 0.5) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "YOLO callback gap %.2fs — pipeline stalled", gap);
            }
        }
        last_det_callback_time_ = now();
        have_det_callback_      = true;

        // Find best target-class detection.
        const Detection* best = nullptr;
        double best_conf = 0.0;
        for (const auto& det : msg->detections) {
            if (std::find(target_classes_.begin(), target_classes_.end(), det.class_name)
                    != target_classes_.end() &&
                det.confidence >= min_confidence_ &&
                det.confidence > best_conf) {
                best = &det;
                best_conf = det.confidence;
            }
        }

        if (!best) {
            consecutive_dets_ = 0;
            prev_cx_ = prev_cy_ = -1.0;
            handle_no_detection();
            return;
        }

        // Position-continuity guard.
        if (prev_cx_ >= 0.0) {
            double jump_x = std::abs(best->center_x - prev_cx_) / image_width_;
            double jump_y = std::abs(best->center_y - prev_cy_) / image_height_;
            if (jump_x > 0.30 || jump_y > 0.30) {
                consecutive_dets_ = 0;
                prev_cx_ = best->center_x;
                prev_cy_ = best->center_y;
                handle_no_detection();
                return;
            }
        }
        prev_cx_ = best->center_x;
        prev_cy_ = best->center_y;

        // Smooth bbox.
        push_smooth(cx_hist_, best->center_x);
        push_smooth(cy_hist_, best->center_y);
        push_smooth(bw_hist_, best->size_width);
        push_smooth(bh_hist_, best->size_height);

        double cx = avg(cx_hist_);
        double cy = avg(cy_hist_);
        double bh = avg(bh_hist_);
        double bbox_ratio = bh / static_cast<double>(image_height_);

        double ex_norm = (cx - image_width_  / 2.0) / (image_width_  / 2.0);
        double ey_norm = (cy - image_height_ / 2.0) / (image_height_ / 2.0);

        // Cache last-known bearing for SEARCH handoff after REACQUIRE timeout.
        // bearing is in body frame; convert to world by adding current drone yaw.
        // Half-FOV is roughly 30° (60° HFOV) so ex_norm=1 maps to ~30° offset.
        double bearing_offset_rad = ex_norm * (30.0 * M_PI / 180.0);
        last_bearing_to_sign_rad_ = wrap_to_pi(drone_yaw_rad_ + bearing_offset_rad);
        have_last_bearing_ = true;

        consecutive_dets_++;
        update_state_on_detection(bbox_ratio, ex_norm, ey_norm, best_conf);

        // Build + publish command for current state.
        Twist cmd = build_command(ex_norm, ey_norm, bbox_ratio, bearing_offset_rad);
        publish_cmd_vel(cmd);

        throttled_log(cx, cy, bh, bbox_ratio, ex_norm, ey_norm, best_conf, cmd);
    }

    // ─────────────────────────────────────────────────────────
    // State transitions when we have a detection
    // ─────────────────────────────────────────────────────────
    void update_state_on_detection(double bbox_ratio, double ex_norm, double ey_norm,
                                   double /*conf*/) {
        State prev = state_;

        bool centered = std::abs(ex_norm) < hold_center_tol_ &&
                        std::abs(ey_norm) < hold_center_tol_;
        bool close    = bbox_ratio >= hold_bbox_ratio_;

        switch (state_) {
        case State::SEARCHING: {
            bool lock_ok = consecutive_dets_ >= lockon_consec_ &&
                           std::abs(ex_norm) < lockon_ex_tol_;
            if (lock_ok) {
                state_ = State::APPROACHING;
                approach_start_time_ = now();
                reach_ticks_ = 0;
            } else {
                // Detection seen but off-axis — pull search yaw toward it.
                // Reset the per-step settle so we move to the new target.
                if (consecutive_dets_ >= 1 && std::abs(ex_norm) >= lockon_ex_tol_) {
                    double yaw_bias = k_lockon_bias_ * ex_norm * (30.0 * M_PI / 180.0);
                    yaw_target_rad_ = wrap_to_pi(drone_yaw_rad_ + yaw_bias);
                    search_arrived_ = false;
                }
            }
            break;
        }
        case State::REACQUIRE:
            // Detection back — resume APPROACHING immediately, no relock.
            state_ = State::APPROACHING;
            // Don't reset approach_start_time_ — preserve persistence credit
            reach_ticks_ = 0;
            break;
        case State::APPROACHING:
            if (close && centered) {
                reach_ticks_++;
                if (reach_ticks_ >= reach_consec_ticks_) {
                    state_ = State::REACHED;
                }
            } else {
                reach_ticks_ = 0;
            }
            break;
        case State::REACHED:
            // Sticky terminal.
            break;
        }

        if (state_ != prev) {
            RCLCPP_INFO(get_logger(),
                "State: %s -> %s  (consec=%d ratio=%.2f ex=%+.2f ey=%+.2f)",
                state_name(prev), state_name(state_),
                consecutive_dets_, bbox_ratio, ex_norm, ey_norm);
        }
    }

    // ─────────────────────────────────────────────────────────
    // No-detection path — drives APPROACHING → REACQUIRE → SEARCHING.
    // ─────────────────────────────────────────────────────────
    void handle_no_detection() {
        Twist cmd{};

        switch (state_) {
        case State::REACHED:
            cmd.angular.y = pitch_return_cmd();
            break;

        case State::SEARCHING:
            cmd = build_search_command();
            break;

        case State::APPROACHING: {
            // Compute adaptive timeout based on how long we've been approaching.
            double approach_dur = (now() - approach_start_time_).seconds();
            active_reacquire_timeout_sec_ =
                std::clamp(base_reacquire_sec_ + k_persist_ * approach_dur,
                           base_reacquire_sec_, max_reacquire_sec_);
            state_ = State::REACQUIRE;
            reacquire_start_time_ = now();
            RCLCPP_WARN(get_logger(),
                "APPROACHING -> REACQUIRE (approach_dur=%.1fs, timeout=%.1fs)",
                approach_dur, active_reacquire_timeout_sec_);
            cmd = build_reacquire_command();
            break;
        }

        case State::REACQUIRE: {
            double rdur = (now() - reacquire_start_time_).seconds();
            if (rdur >= active_reacquire_timeout_sec_) {
                state_ = State::SEARCHING;
                seed_search_from_last_bearing();
                RCLCPP_WARN(get_logger(),
                    "REACQUIRE -> SEARCHING (timed out after %.1fs)", rdur);
                cmd = build_search_command();
            } else {
                cmd = build_reacquire_command();
            }
            break;
        }
        }

        publish_cmd_vel(cmd);
    }

    // ─────────────────────────────────────────────────────────
    // Build command for the current state with a fresh detection.
    // ─────────────────────────────────────────────────────────
    Twist build_command(double ex_norm, double ey_norm, double bbox_ratio,
                        double /*bearing_offset_rad*/) {
        Twist cmd{};

        switch (state_) {
        case State::REACHED:
            cmd.angular.y = pitch_return_cmd();
            break;

        case State::APPROACHING: {
            // Translation only — yaw locked at zero. No alignment gate.
            // vx — pure distance closure. Naturally tapers as bbox grows.
            double vx = k_fwd_ * (target_bbox_ratio_ - bbox_ratio);
            if (vx < 0.0) vx = 0.0;   // don't reverse — REACHED handles overshoot

            // vy — lateral centering (Simulink: +y = LEFT, so vy = -k*ex moves
            // RIGHT when sign is right of center → toward sign).
            double vy = -k_lat_ * ex_norm;

            // vz — vertical centering. Sign below center (ey > 0) → descend (vz < 0).
            double vz = 0.0;
            if (std::abs(ey_norm) > vert_deadband_) {
                vz = -k_vz_ * ey_norm;
            }
            if (vz > 0.0) vz = 0.0;   // never climb during approach

            cmd.linear.x  = vx;
            cmd.linear.y  = vy;
            cmd.linear.z  = vz;
            cmd.angular.y = pitch_return_cmd();
            cmd.angular.z = 0.0;
            break;
        }

        case State::REACQUIRE:
            // Should not reach here — REACQUIRE always enters via handle_no_detection.
            cmd.angular.y = pitch_return_cmd();
            break;

        case State::SEARCHING:
            cmd = build_search_command();
            break;
        }

        return cmd;
    }

    // ─────────────────────────────────────────────────────────
    // REACQUIRE: hold position, pitch return only. No motion.
    // ─────────────────────────────────────────────────────────
    Twist build_reacquire_command() {
        Twist cmd{};
        cmd.angular.y = pitch_return_cmd();
        return cmd;
    }

    // ─────────────────────────────────────────────────────────
    // SEARCHING: deterministic yaw-to-angle sequence using ground-truth yaw.
    //   STEP_RIGHT_60  → yaw target = -60°
    //   STEP_LEFT_60   → yaw target = +60°
    //   STEP_CENTER    → yaw target =   0°
    //   STEP_STRAFE_R  → vy = -strafe_speed for strafe_dur seconds
    //   then repeat
    //
    // Note: lock-on path can override yaw_target via k_lockon_bias before
    // we reach the planned target — that's intentional. Edge-of-frame
    // detections pull us to face them.
    // ─────────────────────────────────────────────────────────
    Twist build_search_command() {
        Twist cmd{};
        cmd.angular.y = pitch_return_cmd();

        if (!have_drone_pose_) {
            // No ground-truth yaw available — fall back to slow CCW yaw at 0.2 rad/s.
            cmd.angular.z = 0.2;
            return cmd;
        }

        switch (search_step_) {
        case SearchStep::YAW_RIGHT_60:
        case SearchStep::YAW_LEFT_60:
        case SearchStep::YAW_CENTER: {
            double err = wrap_to_pi(yaw_target_rad_ - drone_yaw_rad_);
            if (std::abs(err) < search_yaw_arrive_tol_rad_) {
                if (!search_arrived_) {
                    search_arrived_     = true;
                    search_arrive_time_ = now();
                }
                cmd.angular.z = 0.0;
                double settle = (now() - search_arrive_time_).seconds();
                if (settle >= search_settle_sec_) {
                    advance_search_step();
                }
            } else {
                search_arrived_ = false;
                cmd.angular.z = clamp_vel(k_search_yaw_ * err, max_angular_);
            }
            break;
        }
        case SearchStep::STRAFE_RIGHT: {
            double elapsed = (now() - strafe_start_time_).seconds();
            if (elapsed >= search_strafe_dur_sec_) {
                advance_search_step();
                break;
            }
            cmd.linear.y  = -search_strafe_speed_;  // RIGHT in Simulink convention
            cmd.angular.z = 0.0;
            break;
        }
        }

        return cmd;
    }

    void advance_search_step() {
        switch (search_step_) {
        case SearchStep::YAW_RIGHT_60:
            search_step_    = SearchStep::YAW_LEFT_60;
            yaw_target_rad_ = wrap_to_pi(drone_yaw_at_search_start_ + deg2rad(+search_yaw_target_deg_));
            search_arrived_ = false;
            break;
        case SearchStep::YAW_LEFT_60:
            search_step_    = SearchStep::YAW_CENTER;
            yaw_target_rad_ = drone_yaw_at_search_start_;
            search_arrived_ = false;
            break;
        case SearchStep::YAW_CENTER:
            search_step_       = SearchStep::STRAFE_RIGHT;
            strafe_start_time_ = now();
            break;
        case SearchStep::STRAFE_RIGHT:
            // Strafed right; new "center" yaw is current yaw (which hasn't
            // changed during strafe). Re-anchor and restart cycle.
            drone_yaw_at_search_start_ = drone_yaw_rad_;
            search_step_    = SearchStep::YAW_RIGHT_60;
            yaw_target_rad_ = wrap_to_pi(drone_yaw_at_search_start_ + deg2rad(-search_yaw_target_deg_));
            search_arrived_ = false;
            break;
        }
        RCLCPP_INFO(get_logger(), "[SEARCH] -> %s (yaw_target=%.2frad)",
                    search_step_name(search_step_), yaw_target_rad_);
    }

    double drone_yaw_at_search_start_ = 0.0;

    void reset_search() {
        drone_yaw_at_search_start_ = drone_yaw_rad_;
        search_step_    = SearchStep::YAW_RIGHT_60;
        yaw_target_rad_ = wrap_to_pi(drone_yaw_at_search_start_ + deg2rad(-search_yaw_target_deg_));
        search_arrived_ = false;
        yaw_target_init_ = true;
        RCLCPP_INFO(get_logger(),
            "[SEARCH] reset — anchor_yaw=%.2frad, first target=%.2frad",
            drone_yaw_at_search_start_, yaw_target_rad_);
    }

    void seed_search_from_last_bearing() {
        if (have_last_bearing_) {
            // First target after fallback: look where we last saw the sign.
            drone_yaw_at_search_start_ = drone_yaw_rad_;
            search_step_    = SearchStep::YAW_CENTER;   // will advance to STRAFE → restart cycle
            yaw_target_rad_ = last_bearing_to_sign_rad_;
            search_arrived_ = false;
            RCLCPP_INFO(get_logger(),
                "[SEARCH] seeded with last-known bearing %.2frad", yaw_target_rad_);
        } else {
            reset_search();
        }
    }

    // ─────────────────────────────────────────────────────────
    // Publish-time filters: sim stall, floor clamp, ramp.
    // ─────────────────────────────────────────────────────────
    void publish_cmd_vel(Twist cmd) {
        update_sim_health();

        // Filter 1: sim-stall override — zero everything if Simulink isn't stepping.
        // Also clear ramp state so we don't lurch on resume.
        if (!sim_healthy_) {
            cmd = Twist{};
            prev_cmd_ = Twist{};   // reset ramp anchor to zero
        }

        // Filter 2: clamp velocities to limits.
        cmd.linear.x  = clamp_vel(cmd.linear.x,  max_linear_);
        cmd.linear.y  = clamp_vel(cmd.linear.y,  max_linear_);
        cmd.linear.z  = clamp_vel(cmd.linear.z,  max_linear_);
        cmd.angular.y = clamp_vel(cmd.angular.y, max_angular_);
        cmd.angular.z = clamp_vel(cmd.angular.z, max_angular_);

        // Filter 3: floor clamp — prevent further descent below safety floor.
        if (have_drone_pose_) {
            double pose_age = (now() - last_drone_pose_time_).seconds();
            if (pose_age < safety_pose_max_age_ &&
                drone_z_ < safety_min_altitude_ &&
                cmd.linear.z < 0.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Floor clamp: z=%.2f below %.2f — vz suppressed",
                    drone_z_, safety_min_altitude_);
                cmd.linear.z = 0.0;
            }
        }

        // Filter 4: velocity ramp (smooth all axes).
        ramp_velocity(cmd);

        pub_cmd_->publish(cmd);
        prev_cmd_              = cmd;
        last_cmd_publish_time_ = now();
        cmd_count_++;
    }

    void ramp_velocity(Twist& cmd) {
        auto ramp = [this](double cur, double prev) {
            double delta = cur - prev;
            if (std::abs(delta) > vel_ramp_rate_) {
                return prev + std::copysign(vel_ramp_rate_, delta);
            }
            return cur;
        };
        cmd.linear.x  = ramp(cmd.linear.x,  prev_cmd_.linear.x);
        cmd.linear.y  = ramp(cmd.linear.y,  prev_cmd_.linear.y);
        cmd.linear.z  = ramp(cmd.linear.z,  prev_cmd_.linear.z);
        cmd.angular.y = ramp(cmd.angular.y, prev_cmd_.angular.y);
        cmd.angular.z = ramp(cmd.angular.z, prev_cmd_.angular.z);
    }

    // ─────────────────────────────────────────────────────────
    // Watchdog: if detection callback hasn't fired recently, drive
    // through handle_no_detection so the state machine keeps making progress.
    // ─────────────────────────────────────────────────────────
    void watchdog_tick() {
        double det_age = (now() - last_det_callback_time_).seconds();
        if (det_age > 2.0 * watchdog_period_sec_) {
            // YOLO is silent. Drive the no-detection path so SEARCH advances
            // and REACQUIRE timer runs even with zero detections.
            consecutive_dets_ = 0;
            handle_no_detection();
        }
    }

    // ─────────────────────────────────────────────────────────
    // Diagnostics: every diag_period_sec, log topic rates + state.
    // ─────────────────────────────────────────────────────────
    void diag_tick() {
        auto t = now();
        double dt = (t - diag_window_start_).seconds();
        if (dt < 0.5) return;

        double sim_cam_hz = sim_cam_count_ / dt;
        double det_hz     = det_count_     / dt;
        double pitch_hz   = pitch_count_   / dt;
        double pose_hz    = pose_count_    / dt;
        double hb_hz      = hb_count_      / dt;
        double cmd_hz     = cmd_count_     / dt;

        SimHealth sh = compute_sim_health();
        const char* sh_str = (sh == SimHealth::HEALTHY) ? "OK" :
                             (sh == SimHealth::PAUSED) ? "PAUSED" : "DEAD";
        double sim_rate_meas = 0.0;
        if (hb_buffer_.size() >= 2) {
            double d_sim  = hb_buffer_.back().sim_t - hb_buffer_.front().sim_t;
            double d_wall = (hb_buffer_.back().wall_t - hb_buffer_.front().wall_t).seconds();
            if (d_wall > 1e-3) sim_rate_meas = d_sim / d_wall;
        }

        double tgt_dist = 0.0, tgt_bearing_err_deg = 0.0;
        if (have_drone_pose_ && have_target_pose_) {
            double dx = target_x_ - drone_x_;
            double dy = target_y_ - drone_y_;
            tgt_dist = std::sqrt(dx*dx + dy*dy);
            double bearing = std::atan2(dy, dx);
            tgt_bearing_err_deg = wrap_to_pi(bearing - drone_yaw_rad_) * 180.0 / M_PI;
        }

        if (debug_image_enabled_) {
            double ovcam_hz = ovcam_count_ / dt;
            RCLCPP_INFO(get_logger(),
                "[diag] state=%s | /sim/cam=%.1f /ovcam=%.1f /yolo=%.1f /pitch=%.1f /pose=%.1f "
                "/hb=%.1f /cmd=%.1f | sim=%s rate=%.2f sim_t=%.2f | xyz=(%.1f,%.1f,%.1f) "
                "yaw=%.2f | tgt: dist=%.1fm bearing_err=%+.1f°",
                state_name(state_), sim_cam_hz, ovcam_hz, det_hz, pitch_hz, pose_hz,
                hb_hz, cmd_hz, sh_str, sim_rate_meas, last_hb_sim_,
                drone_x_, drone_y_, drone_z_, drone_yaw_rad_,
                tgt_dist, tgt_bearing_err_deg);
        } else {
            RCLCPP_INFO(get_logger(),
                "[diag] state=%s | /sim/cam=%.1f /ovcam=N/A(debug off) /yolo=%.1f /pitch=%.1f /pose=%.1f "
                "/hb=%.1f /cmd=%.1f | sim=%s rate=%.2f sim_t=%.2f | xyz=(%.1f,%.1f,%.1f) "
                "yaw=%.2f | tgt: dist=%.1fm bearing_err=%+.1f°",
                state_name(state_), sim_cam_hz, det_hz, pitch_hz, pose_hz,
                hb_hz, cmd_hz, sh_str, sim_rate_meas, last_hb_sim_,
                drone_x_, drone_y_, drone_z_, drone_yaw_rad_,
                tgt_dist, tgt_bearing_err_deg);
        }

        sim_cam_count_ = ovcam_count_ = det_count_ = pitch_count_ = 0;
        pose_count_ = hb_count_ = cmd_count_ = 0;
        diag_window_start_ = t;
    }

    // ─────────────────────────────────────────────────────────
    // Utilities
    // ─────────────────────────────────────────────────────────
    void push_smooth(std::deque<double>& hist, double val) {
        hist.push_back(val);
        while (static_cast<int>(hist.size()) > smoothing_window_)
            hist.pop_front();
    }

    double avg(const std::deque<double>& hist) const {
        if (hist.empty()) return 0.0;
        double sum = 0.0;
        for (double v : hist) sum += v;
        return sum / static_cast<double>(hist.size());
    }

    double clamp_vel(double v, double lim) const {
        return std::clamp(v, -lim, lim);
    }

    static double wrap_to_pi(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double deg2rad(double d) { return d * M_PI / 180.0; }

    // ─────────────────────────────────────────────────────────
    void throttled_log(double cx, double cy, double bh, double bbox_ratio,
                       double ex_norm, double ey_norm, double conf, const Twist& cmd) {
        auto t = now();
        if ((t - last_log_time_).seconds() < 0.5) return;
        last_log_time_ = t;

        RCLCPP_INFO(get_logger(),
            "[%s] conf=%.2f bbox_ratio=%.3f ex=%+.3f ey=%+.3f | "
            "cmd: vx=%+.3f vy=%+.3f vz=%+.3f wy=%+.3f wz=%+.3f",
            state_name(state_), conf, bbox_ratio, ex_norm, ey_norm,
            cmd.linear.x, cmd.linear.y, cmd.linear.z,
            cmd.angular.y, cmd.angular.z);
        (void)cx; (void)cy; (void)bh;
    }

    // ─────────────────────────────────────────────────────────
    // Debug overlay — runs at camera rate, draws state + detections.
    // ─────────────────────────────────────────────────────────
    void publish_debug_image() {
        if (latest_frame_.empty()) return;

        cv::Mat debug = latest_frame_.clone();
        int ic_x = image_width_ / 2, ic_y = image_height_ / 2;

        // Centering tolerance box.
        int box_w = static_cast<int>(hold_center_tol_ * image_width_);
        int box_h = static_cast<int>(hold_center_tol_ * image_height_);
        cv::rectangle(debug,
            {ic_x - box_w, ic_y - box_h}, {ic_x + box_w, ic_y + box_h},
            {0, 180, 180}, 1);

        // bbox_ratio rails.
        auto draw_rail = [&](double r, cv::Scalar color, const char* tag) {
            int half_h = static_cast<int>(r * image_height_ / 2.0);
            int y_top = ic_y - half_h, y_bot = ic_y + half_h;
            cv::line(debug, {0, y_top}, {image_width_, y_top}, color, 1);
            cv::line(debug, {0, y_bot}, {image_width_, y_bot}, color, 1);
            cv::putText(debug, tag, {5, y_top - 2}, cv::FONT_HERSHEY_SIMPLEX,
                        0.35, color, 1);
        };
        draw_rail(target_bbox_ratio_, {0, 160, 255}, "target");
        draw_rail(hold_bbox_ratio_,   {0, 100, 255}, "hold");

        // Draw all detections.
        const Detection* target_det = nullptr;
        double target_conf = 0.0;
        if (latest_dets_) {
            for (const auto& d : latest_dets_->detections) {
                bool is_target = std::find(target_classes_.begin(),
                    target_classes_.end(), d.class_name) != target_classes_.end();
                if (is_target && d.confidence > target_conf) {
                    target_det = &d;
                    target_conf = d.confidence;
                }
                if (!is_target && d.confidence >= debug_min_draw_conf_) {
                    int x1 = static_cast<int>(d.center_x - d.size_width  / 2.0);
                    int y1 = static_cast<int>(d.center_y - d.size_height / 2.0);
                    int x2 = static_cast<int>(d.center_x + d.size_width  / 2.0);
                    int y2 = static_cast<int>(d.center_y + d.size_height / 2.0);
                    cv::rectangle(debug, {x1, y1}, {x2, y2}, {160, 160, 160}, 1);
                    char lbl[64];
                    std::snprintf(lbl, sizeof(lbl), "%s %.2f",
                        d.class_name.c_str(), d.confidence);
                    cv::putText(debug, lbl, {x1, std::max(10, y1 - 3)},
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, {180, 180, 180}, 1);
                }
            }
        }

        if (target_det) {
            double cx = target_det->center_x, cy = target_det->center_y;
            double bw = target_det->size_width, bh = target_det->size_height;
            int x1 = static_cast<int>(cx - bw/2), y1 = static_cast<int>(cy - bh/2);
            int x2 = static_cast<int>(cx + bw/2), y2 = static_cast<int>(cy + bh/2);
            cv::rectangle(debug, {x1, y1}, {x2, y2}, {0, 255, 0}, 2);
            cv::circle(debug, {static_cast<int>(cx), static_cast<int>(cy)},
                4, {0, 0, 255}, -1);
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "%s %.2f",
                target_det->class_name.c_str(), target_det->confidence);
            cv::putText(debug, lbl, {x1, std::max(10, y1 - 3)},
                cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 0}, 1);
        }

        // Crosshair.
        cv::line(debug, {ic_x - 15, ic_y}, {ic_x + 15, ic_y}, {255, 255, 255}, 1);
        cv::line(debug, {ic_x, ic_y - 15}, {ic_x, ic_y + 15}, {255, 255, 255}, 1);

        // Header — state + sim health.
        SimHealth sh = compute_sim_health();
        const char* sh_str = (sh == SimHealth::HEALTHY) ? "OK" :
                             (sh == SimHealth::PAUSED) ? "PAUSED" : "DEAD";
        cv::Scalar sh_color = sim_healthy_ ? cv::Scalar(0, 255, 0)
                                           : cv::Scalar(0, 0, 255);
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s  sim=%s  pitch=%+.2f",
            state_name(state_), sh_str, current_pitch_rad_);
        cv::putText(debug, buf, {10, 25}, cv::FONT_HERSHEY_SIMPLEX,
            0.55, sh_color, 2);

        // Command line.
        std::snprintf(buf, sizeof(buf),
            "vx=%+.2f vy=%+.2f vz=%+.2f  wy=%+.2f wz=%+.2f",
            prev_cmd_.linear.x, prev_cmd_.linear.y, prev_cmd_.linear.z,
            prev_cmd_.angular.y, prev_cmd_.angular.z);
        cv::putText(debug, buf, {10, 50}, cv::FONT_HERSHEY_SIMPLEX,
            0.45, {255, 255, 255}, 1);

        // Search-step display.
        if (state_ == State::SEARCHING) {
            double err = wrap_to_pi(yaw_target_rad_ - drone_yaw_rad_);
            std::snprintf(buf, sizeof(buf), "search: %s tgt=%.2f err=%+.2frad",
                search_step_name(search_step_), yaw_target_rad_, err);
            cv::putText(debug, buf, {10, 75}, cv::FONT_HERSHEY_SIMPLEX,
                0.45, {0, 200, 255}, 1);
        } else if (state_ == State::REACQUIRE) {
            double rdur = (now() - reacquire_start_time_).seconds();
            std::snprintf(buf, sizeof(buf), "reacquire: %.1f/%.1fs",
                rdur, active_reacquire_timeout_sec_);
            cv::putText(debug, buf, {10, 75}, cv::FONT_HERSHEY_SIMPLEX,
                0.45, {0, 200, 255}, 1);
        }

        // Bottom — ground-truth pose + target instrumentation.
        if (have_drone_pose_) {
            std::snprintf(buf, sizeof(buf), "gt: xyz=(%.1f,%.1f,%.1f) yaw=%.2f",
                drone_x_, drone_y_, drone_z_, drone_yaw_rad_);
            cv::putText(debug, buf, {10, image_height_ - 25},
                cv::FONT_HERSHEY_SIMPLEX, 0.40, {200, 200, 200}, 1);
        }
        if (have_drone_pose_ && have_target_pose_) {
            double dx = target_x_ - drone_x_;
            double dy = target_y_ - drone_y_;
            double dist = std::sqrt(dx*dx + dy*dy);
            double bearing_err = wrap_to_pi(std::atan2(dy, dx) - drone_yaw_rad_)
                                 * 180.0 / M_PI;
            std::snprintf(buf, sizeof(buf),
                "tgt: (%.1f,%.1f,%.1f) dist=%.1fm bearing_err=%+.1f°",
                target_x_, target_y_, target_z_, dist, bearing_err);
            cv::putText(debug, buf, {10, image_height_ - 8},
                cv::FONT_HERSHEY_SIMPLEX, 0.40, {200, 220, 200}, 1);
        }

        auto out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug)
            .toImageMsg();
        out->header.stamp = now();
        out->header.frame_id = "camera";
        pub_debug_->publish(*out);
    }
};

// ─────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VispServoNode>());
    rclcpp::shutdown();
    return 0;
}
