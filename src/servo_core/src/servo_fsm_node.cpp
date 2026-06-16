// ─────────────────────────────────────────────────────────────────
// servo_core/servo_fsm_node.cpp
//
// Shared FSM + safety filters + /bench/state instrumentation. Ported
// verbatim from the proven HIL controller (TS2) with ONE change: the
// APPROACHING velocity is delegated to the injected IServoController.
// Everything else — search, reacquire, heartbeat health, clamp, floor,
// ramp — is identical for every test subject, so a benchmark difference
// is attributable to the servoing law alone.
// ─────────────────────────────────────────────────────────────────
#include "servo_core/servo_fsm_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace servo_core {

const char* ServoFsmNode::state_name(State s) {
    switch (s) {
        case State::SEARCHING:   return "SEARCHING";
        case State::APPROACHING: return "APPROACHING";
        case State::REACQUIRE:   return "REACQUIRE";
        case State::REACHED:     return "REACHED";
    }
    return "UNKNOWN";
}

const char* ServoFsmNode::search_step_name(SearchStep s) {
    switch (s) {
        case SearchStep::YAW_RIGHT_60:  return "YAW_RIGHT_60";
        case SearchStep::YAW_LEFT_60:   return "YAW_LEFT_60";
        case SearchStep::YAW_CENTER:    return "YAW_CENTER";
        case SearchStep::STRAFE_RIGHT:  return "STRAFE_RIGHT";
    }
    return "?";
}

ServoFsmNode::ServoFsmNode(const std::string& node_name)
    : rclcpp::Node(node_name) {
    declare_parameters();
    load_parameters();
    setup_ros();
    RCLCPP_INFO(get_logger(),
        "%s started — FSM core (targets[0]='%s', conf>=%.2f). Awaiting controller.",
        node_name.c_str(),
        target_classes_.empty() ? "?" : target_classes_[0].c_str(),
        min_confidence_);
}

void ServoFsmNode::set_controller(std::unique_ptr<IServoController> controller) {
    controller_ = std::move(controller);
    controller_name_ = controller_ ? controller_->name() : "unset";

    // Hand the controller the fixed geometry so it can build any
    // one-time/desired quantities (e.g. IBVS desired features).
    ServoInputs cfg;
    cfg.image_width        = image_width_;
    cfg.image_height       = image_height_;
    cfg.fx                 = cam_fx_;
    cfg.fy                 = cam_fy_;
    cfg.cx0                = cam_cx_;
    cfg.cy0                = cam_cy_;
    cfg.target_bbox_ratio  = target_bbox_ratio_;
    cfg.known_target_height = known_target_height_;
    if (controller_) controller_->init(cfg);

    RCLCPP_INFO(get_logger(), "Controller bound: '%s'", controller_name_.c_str());
}

// ─────────────────────────────────────────────────────────────────
void ServoFsmNode::declare_parameters() {
    declare_parameter("target_class",      std::string("stop sign"));
    declare_parameter("min_confidence",     0.30);
    declare_parameter("image_width",        640);
    declare_parameter("image_height",       480);

    // Sim camera intrinsics (hil_ros_init_LT: fx=fy=554, cx=320, cy=240).
    declare_parameter("cam_fx", 554.0);
    declare_parameter("cam_fy", 554.0);
    declare_parameter("cam_cx", 320.0);
    declare_parameter("cam_cy", 240.0);
    declare_parameter("known_target_height", 1.5);

    declare_parameter("k_pitch_return",     1.0);
    declare_parameter("pitch_zero_tol",     0.02);

    declare_parameter("max_linear",         3.0);
    declare_parameter("max_angular",        0.8);
    declare_parameter("vel_ramp_rate",      1.5);

    declare_parameter("target_bbox_ratio",  0.55);
    declare_parameter("hold_bbox_ratio",    0.50);
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
    declare_parameter("fresh_sim_threshold_sec",    2.0);

    declare_parameter("safety_min_altitude",  0.5);
    declare_parameter("safety_pose_max_age",  2.0);

    declare_parameter("watchdog_period_sec",  0.20);
    declare_parameter("diag_period_sec",      2.0);
    declare_parameter("smoothing_window",     5);
}

void ServoFsmNode::load_parameters() {
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
    min_confidence_ = get_parameter("min_confidence").as_double();
    image_width_    = get_parameter("image_width").as_int();
    image_height_   = get_parameter("image_height").as_int();

    cam_fx_ = get_parameter("cam_fx").as_double();
    cam_fy_ = get_parameter("cam_fy").as_double();
    cam_cx_ = get_parameter("cam_cx").as_double();
    cam_cy_ = get_parameter("cam_cy").as_double();
    known_target_height_ = get_parameter("known_target_height").as_double();

    k_pitch_return_ = get_parameter("k_pitch_return").as_double();
    pitch_zero_tol_ = get_parameter("pitch_zero_tol").as_double();

    max_linear_    = get_parameter("max_linear").as_double();
    max_angular_   = get_parameter("max_angular").as_double();
    vel_ramp_rate_ = get_parameter("vel_ramp_rate").as_double();

    target_bbox_ratio_  = get_parameter("target_bbox_ratio").as_double();
    hold_bbox_ratio_    = get_parameter("hold_bbox_ratio").as_double();
    hold_center_tol_    = get_parameter("hold_center_tol").as_double();
    reach_consec_ticks_ = get_parameter("reach_consec_ticks").as_int();

    lockon_consec_ = get_parameter("lockon_consec").as_int();
    lockon_ex_tol_ = get_parameter("lockon_ex_tol").as_double();
    k_lockon_bias_ = get_parameter("k_lockon_bias").as_double();

    search_yaw_target_deg_     = get_parameter("search_yaw_target_deg").as_double();
    k_search_yaw_              = get_parameter("k_search_yaw").as_double();
    search_yaw_arrive_tol_rad_ = get_parameter("search_yaw_arrive_tol_rad").as_double();
    search_settle_sec_         = get_parameter("search_settle_sec").as_double();
    search_strafe_speed_       = get_parameter("search_strafe_speed").as_double();
    search_strafe_dur_sec_     = get_parameter("search_strafe_dur_sec").as_double();

    base_reacquire_sec_ = get_parameter("base_reacquire_sec").as_double();
    k_persist_          = get_parameter("k_persist").as_double();
    max_reacquire_sec_  = get_parameter("max_reacquire_sec").as_double();

    heartbeat_window_samples_  = get_parameter("heartbeat_window_samples").as_int();
    heartbeat_gap_sec_         = get_parameter("heartbeat_gap_sec").as_double();
    heartbeat_dead_sec_        = get_parameter("heartbeat_dead_sec").as_double();
    sim_paused_rate_           = get_parameter("sim_paused_rate").as_double();
    sim_recovery_debounce_sec_ = get_parameter("sim_recovery_debounce_sec").as_double();
    sim_unhealthy_entry_sec_   = get_parameter("sim_unhealthy_entry_sec").as_double();
    fresh_sim_threshold_sec_   = get_parameter("fresh_sim_threshold_sec").as_double();

    safety_min_altitude_ = get_parameter("safety_min_altitude").as_double();
    safety_pose_max_age_ = get_parameter("safety_pose_max_age").as_double();

    watchdog_period_sec_ = get_parameter("watchdog_period_sec").as_double();
    diag_period_sec_     = get_parameter("diag_period_sec").as_double();
    smoothing_window_    = get_parameter("smoothing_window").as_int();
}

void ServoFsmNode::setup_ros() {
    auto qos_be  = rclcpp::QoS(1).best_effort();
    auto qos_tl  = rclcpp::QoS(1).reliable().transient_local();

    sub_dets_ = create_subscription<DetectionArray>(
        "/yolo/detections", qos_be,
        std::bind(&ServoFsmNode::on_detections, this, std::placeholders::_1));

    sub_pitch_ = create_subscription<Float64>(
        "/sim/pitch_angle", rclcpp::QoS(10),
        std::bind(&ServoFsmNode::on_pitch, this, std::placeholders::_1));

    sub_drone_pose_ = create_subscription<Float64MA>(
        "/sim/drone_pose", rclcpp::QoS(10),
        std::bind(&ServoFsmNode::on_drone_pose, this, std::placeholders::_1));

    sub_target_pose_ = create_subscription<Float64MA>(
        "/sim/target_pose", qos_tl,
        std::bind(&ServoFsmNode::on_target_pose, this, std::placeholders::_1));

    sub_heartbeat_ = create_subscription<Float64>(
        "/sim/heartbeat", rclcpp::QoS(10),
        std::bind(&ServoFsmNode::on_heartbeat, this, std::placeholders::_1));

    pub_cmd_         = create_publisher<Twist>("/cmd_vel", 10);
    pub_bench_state_ = create_publisher<String>("/bench/state", rclcpp::QoS(10));

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
        std::bind(&ServoFsmNode::watchdog_tick, this));
    diag_timer_ = create_wall_timer(
        std::chrono::duration<double>(diag_period_sec_),
        std::bind(&ServoFsmNode::diag_tick, this));

    reset_search();
}

// ─────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────
void ServoFsmNode::on_drone_pose(const Float64MA::SharedPtr msg) {
    if (msg->data.size() < 5) return;
    drone_x_       = msg->data[0];
    drone_y_       = msg->data[1];
    drone_z_       = msg->data[2];
    drone_yaw_rad_ = msg->data[4];
    last_drone_pose_time_ = now();
    have_drone_pose_      = true;
    pose_count_++;
}

void ServoFsmNode::on_target_pose(const Float64MA::SharedPtr msg) {
    if (msg->data.size() < 4) return;
    target_x_   = msg->data[0];
    target_y_   = msg->data[1];
    target_z_   = msg->data[2];
    target_yaw_ = msg->data[3];
    have_target_pose_ = true;
}

void ServoFsmNode::on_pitch(const Float64::SharedPtr msg) {
    current_pitch_rad_   = msg->data;
    last_pitch_msg_time_ = now();
    have_pitch_msg_      = true;
    pitch_count_++;
}

void ServoFsmNode::on_heartbeat(const Float64::SharedPtr msg) {
    auto t_now = now();
    if (have_hb_ && msg->data < last_hb_sim_ - 0.5) {
        RCLCPP_WARN(get_logger(),
            "Sim restart detected: sim_time %.2f -> %.2f — resetting controller",
            last_hb_sim_, msg->data);
        handle_sim_restart();
    }
    if (!have_fresh_sim_ && msg->data < fresh_sim_threshold_sec_) {
        have_fresh_sim_ = true;
        RCLCPP_INFO(get_logger(),
            "Fresh sim confirmed (sim_t=%.2f < %.1fs threshold) — FSM enabled",
            msg->data, fresh_sim_threshold_sec_);
    }
    if (have_hb_) {
        double wall_gap = (t_now - last_hb_wall_time_).seconds();
        if (wall_gap > heartbeat_gap_sec_) hb_buffer_.clear();
    }
    hb_buffer_.push_back({msg->data, t_now});
    while (static_cast<int>(hb_buffer_.size()) > heartbeat_window_samples_)
        hb_buffer_.pop_front();
    last_hb_wall_time_ = t_now;
    last_hb_sim_       = msg->data;
    have_hb_           = true;
    hb_count_++;
}

void ServoFsmNode::handle_sim_restart() {
    state_ = State::SEARCHING;
    consecutive_dets_  = 0;
    reach_ticks_       = 0;
    prev_cx_ = prev_cy_ = -1.0;
    cx_hist_.clear(); cy_hist_.clear(); bw_hist_.clear(); bh_hist_.clear();
    prev_cmd_          = Twist{};
    have_last_bearing_ = false;
    hb_buffer_.clear();
    sim_healthy_            = true;
    prev_obs_healthy_       = true;
    healthy_streak_start_   = now();
    unhealthy_streak_start_ = now();
    have_fresh_sim_         = true;
    reset_search();
    publish_bench_state(true);
}

// ─────────────────────────────────────────────────────────────────
ServoFsmNode::SimHealth ServoFsmNode::compute_sim_health() {
    if (!have_hb_) return SimHealth::HEALTHY;
    double wall_age = (now() - last_hb_wall_time_).seconds();
    if (wall_age > heartbeat_dead_sec_) return SimHealth::DEAD;
    if (static_cast<int>(hb_buffer_.size()) < 3) return SimHealth::HEALTHY;
    double d_sim  = hb_buffer_.back().sim_t   - hb_buffer_.front().sim_t;
    double d_wall = (hb_buffer_.back().wall_t - hb_buffer_.front().wall_t).seconds();
    if (d_wall < 1e-3) return SimHealth::HEALTHY;
    double rate = d_sim / d_wall;
    if (rate < sim_paused_rate_) return SimHealth::PAUSED;
    return SimHealth::HEALTHY;
}

void ServoFsmNode::update_sim_health() {
    SimHealth h = compute_sim_health();
    bool obs_healthy = (h == SimHealth::HEALTHY);
    auto t = now();
    if (obs_healthy != prev_obs_healthy_) {
        if (obs_healthy) healthy_streak_start_   = t;
        else             unhealthy_streak_start_ = t;
        prev_obs_healthy_ = obs_healthy;
    }
    if (obs_healthy && !sim_healthy_) {
        if ((t - healthy_streak_start_).seconds() >= sim_recovery_debounce_sec_) {
            sim_healthy_ = true;
            RCLCPP_WARN(get_logger(), "Sim health RECOVERED (sim_time=%.3f)", last_hb_sim_);
        }
    } else if (!obs_healthy && sim_healthy_) {
        if ((t - unhealthy_streak_start_).seconds() >= sim_unhealthy_entry_sec_) {
            sim_healthy_ = false;
            RCLCPP_ERROR(get_logger(),
                "Sim health LOST: %s — cmd_vel held at 0",
                h == SimHealth::DEAD ? "HEARTBEAT_DEAD" : "SIM_PAUSED");
        }
    }
}

// ─────────────────────────────────────────────────────────────────
double ServoFsmNode::pitch_for_control() {
    if (have_pitch_msg_) {
        double age = (now() - last_pitch_msg_time_).seconds();
        if (age > 2.0)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "/sim/pitch_angle stale (%.1fs)", age);
    }
    return current_pitch_rad_;
}

double ServoFsmNode::pitch_return_cmd() {
    double pitch = pitch_for_control();
    if (std::abs(pitch) < pitch_zero_tol_) return 0.0;
    return -k_pitch_return_ * pitch;
}

// ─────────────────────────────────────────────────────────────────
ServoInputs ServoFsmNode::make_inputs(double ex_norm, double ey_norm,
                                      double cx, double cy, double bw, double bh,
                                      double bbox_ratio) const {
    ServoInputs in;
    in.ex_norm = ex_norm;
    in.ey_norm = ey_norm;
    in.cx = cx; in.cy = cy; in.bw = bw; in.bh = bh;
    in.bbox_ratio = bbox_ratio;
    // Bbox-based depth, identical for every controller (no SLAM dependency).
    in.Z = (bh >= 1.0) ? (cam_fy_ * known_target_height_ / bh) : 50.0;
    in.target_bbox_ratio  = target_bbox_ratio_;
    in.image_width  = image_width_;
    in.image_height = image_height_;
    in.fx = cam_fx_; in.fy = cam_fy_; in.cx0 = cam_cx_; in.cy0 = cam_cy_;
    in.known_target_height = known_target_height_;
    return in;
}

// ─────────────────────────────────────────────────────────────────
void ServoFsmNode::on_detections(const DetectionArray::SharedPtr msg) {
    det_count_++;

    if (have_det_callback_) {
        double gap = (now() - last_det_callback_time_).seconds();
        if (gap > 0.5)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "detection gap %.2fs — pipeline stalled", gap);
    }
    last_det_callback_time_ = now();
    have_det_callback_      = true;

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

    push_smooth(cx_hist_, best->center_x);
    push_smooth(cy_hist_, best->center_y);
    push_smooth(bw_hist_, best->size_width);
    push_smooth(bh_hist_, best->size_height);

    double cx = avg(cx_hist_);
    double cy = avg(cy_hist_);
    double bw = avg(bw_hist_);
    double bh = avg(bh_hist_);
    double bbox_ratio = bh / static_cast<double>(image_height_);

    // Stash the freshest smoothed bbox so build_command's APPROACHING path
    // can construct ServoInputs without threading args through every branch.
    sm_cx_ = cx; sm_cy_ = cy; sm_bw_ = bw; sm_bh_ = bh;

    double ex_norm = (cx - image_width_  / 2.0) / (image_width_  / 2.0);
    double ey_norm = (cy - image_height_ / 2.0) / (image_height_ / 2.0);

    double bearing_offset_rad = ex_norm * (30.0 * M_PI / 180.0);
    last_bearing_to_sign_rad_ = wrap_to_pi(drone_yaw_rad_ + bearing_offset_rad);
    have_last_bearing_ = true;

    consecutive_dets_++;
    update_state_on_detection(bbox_ratio, ex_norm, ey_norm, best_conf);

    Twist cmd = build_command(ex_norm, ey_norm, bbox_ratio);
    publish_cmd_vel(cmd);

    if ((now() - last_log_time_).seconds() >= 0.5) {
        last_log_time_ = now();
        RCLCPP_INFO(get_logger(),
            "[%s/%s] conf=%.2f ratio=%.3f ex=%+.3f ey=%+.3f | vx=%+.3f vy=%+.3f vz=%+.3f wz=%+.3f",
            state_name(state_), controller_name_.c_str(), best_conf, bbox_ratio,
            ex_norm, ey_norm, cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
    }
}

void ServoFsmNode::update_state_on_detection(double bbox_ratio, double ex_norm,
                                             double ey_norm, double /*conf*/) {
    if (!have_fresh_sim_) return;  // hold in SEARCHING until sim resets to t≈0
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
        } else if (consecutive_dets_ >= 1 && std::abs(ex_norm) >= lockon_ex_tol_) {
            double yaw_bias = k_lockon_bias_ * ex_norm * (30.0 * M_PI / 180.0);
            yaw_target_rad_ = wrap_to_pi(drone_yaw_rad_ + yaw_bias);
            search_arrived_ = false;
        }
        break;
    }
    case State::REACQUIRE:
        state_ = State::APPROACHING;
        reach_ticks_ = 0;
        break;
    case State::APPROACHING:
        if (close && centered) {
            reach_ticks_++;
            if (reach_ticks_ >= reach_consec_ticks_) state_ = State::REACHED;
        } else {
            reach_ticks_ = 0;
        }
        break;
    case State::REACHED:
        break;
    }

    if (state_ != prev) {
        RCLCPP_INFO(get_logger(), "State: %s -> %s  (consec=%d ratio=%.2f ex=%+.2f ey=%+.2f)",
            state_name(prev), state_name(state_), consecutive_dets_, bbox_ratio, ex_norm, ey_norm);
        publish_bench_state(true);
    }
}

void ServoFsmNode::handle_no_detection() {
    Twist cmd{};
    switch (state_) {
    case State::REACHED:
        cmd.angular.y = pitch_return_cmd();
        break;
    case State::SEARCHING:
        cmd = build_search_command();
        break;
    case State::APPROACHING: {
        double approach_dur = (now() - approach_start_time_).seconds();
        active_reacquire_timeout_sec_ =
            std::clamp(base_reacquire_sec_ + k_persist_ * approach_dur,
                       base_reacquire_sec_, max_reacquire_sec_);
        state_ = State::REACQUIRE;
        reacquire_start_time_ = now();
        RCLCPP_WARN(get_logger(), "APPROACHING -> REACQUIRE (dur=%.1fs, timeout=%.1fs)",
            approach_dur, active_reacquire_timeout_sec_);
        publish_bench_state(true);
        cmd = build_reacquire_command();
        break;
    }
    case State::REACQUIRE: {
        double rdur = (now() - reacquire_start_time_).seconds();
        if (rdur >= active_reacquire_timeout_sec_) {
            state_ = State::SEARCHING;
            seed_search_from_last_bearing();
            RCLCPP_WARN(get_logger(), "REACQUIRE -> SEARCHING (timed out %.1fs)", rdur);
            publish_bench_state(true);
            cmd = build_search_command();
        } else {
            cmd = build_reacquire_command();
        }
        break;
    }
    }
    publish_cmd_vel(cmd);
}

// ─────────────────────────────────────────────────────────────────
// THE one place that differs between test subjects: APPROACHING delegates
// the body-frame velocity to the injected controller.
// ─────────────────────────────────────────────────────────────────
ServoFsmNode::Twist ServoFsmNode::build_command(double ex_norm, double ey_norm,
                                                double bbox_ratio) {
    Twist cmd{};
    switch (state_) {
    case State::REACHED:
        cmd.angular.y = pitch_return_cmd();
        break;
    case State::APPROACHING: {
        if (!controller_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "No controller bound — cmd held at 0");
            break;
        }
        ServoInputs in = make_inputs(ex_norm, ey_norm,
                                     sm_cx_, sm_cy_, sm_bw_, sm_bh_, bbox_ratio);
        ServoVel v = controller_->computeApproach(in);
        cmd.linear.x  = v.vx;
        cmd.linear.y  = v.vy;
        cmd.linear.z  = v.vz;
        cmd.angular.z = v.wz;
        cmd.angular.y = pitch_return_cmd();
        break;
    }
    case State::REACQUIRE:
        cmd.angular.y = pitch_return_cmd();
        break;
    case State::SEARCHING:
        if (consecutive_dets_ >= 1) {
            // Freeze while accumulating lock: keep yawing and we sweep the
            // target back out of FOV before the second detection arrives.
            cmd.angular.y = pitch_return_cmd();
        } else {
            cmd = build_search_command();
        }
        break;
    }
    return cmd;
}

ServoFsmNode::Twist ServoFsmNode::build_reacquire_command() {
    Twist cmd{};
    cmd.angular.y = pitch_return_cmd();
    return cmd;
}

ServoFsmNode::Twist ServoFsmNode::build_search_command() {
    Twist cmd{};
    cmd.angular.y = pitch_return_cmd();
    if (!have_drone_pose_) {
        cmd.angular.z = 0.2;
        return cmd;
    }
    switch (search_step_) {
    case SearchStep::YAW_RIGHT_60:
    case SearchStep::YAW_LEFT_60:
    case SearchStep::YAW_CENTER: {
        double err = wrap_to_pi(yaw_target_rad_ - drone_yaw_rad_);
        if (std::abs(err) < search_yaw_arrive_tol_rad_) {
            if (!search_arrived_) { search_arrived_ = true; search_arrive_time_ = now(); }
            cmd.angular.z = 0.0;
            if ((now() - search_arrive_time_).seconds() >= search_settle_sec_)
                advance_search_step();
        } else {
            search_arrived_ = false;
            cmd.angular.z = clamp_vel(k_search_yaw_ * err, max_angular_);
        }
        break;
    }
    case SearchStep::STRAFE_RIGHT: {
        double elapsed = (now() - strafe_start_time_).seconds();
        if (elapsed >= search_strafe_dur_sec_) { advance_search_step(); break; }
        cmd.linear.y  = -search_strafe_speed_;
        cmd.angular.z = 0.0;
        break;
    }
    }
    return cmd;
}

void ServoFsmNode::advance_search_step() {
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
        drone_yaw_at_search_start_ = drone_yaw_rad_;
        search_step_    = SearchStep::YAW_RIGHT_60;
        yaw_target_rad_ = wrap_to_pi(drone_yaw_at_search_start_ + deg2rad(-search_yaw_target_deg_));
        search_arrived_ = false;
        break;
    }
    RCLCPP_INFO(get_logger(), "[SEARCH] -> %s (yaw_target=%.2f)",
        search_step_name(search_step_), yaw_target_rad_);
}

void ServoFsmNode::reset_search() {
    drone_yaw_at_search_start_ = drone_yaw_rad_;
    search_step_    = SearchStep::YAW_RIGHT_60;
    yaw_target_rad_ = wrap_to_pi(drone_yaw_at_search_start_ + deg2rad(-search_yaw_target_deg_));
    search_arrived_  = false;
    yaw_target_init_ = true;
}

void ServoFsmNode::seed_search_from_last_bearing() {
    if (have_last_bearing_) {
        drone_yaw_at_search_start_ = drone_yaw_rad_;
        search_step_    = SearchStep::YAW_CENTER;
        yaw_target_rad_ = last_bearing_to_sign_rad_;
        search_arrived_ = false;
    } else {
        reset_search();
    }
}

// ─────────────────────────────────────────────────────────────────
// Publish-time filters: sim-stall, clamp, floor, ramp. + /bench/state.
// ─────────────────────────────────────────────────────────────────
void ServoFsmNode::publish_cmd_vel(Twist cmd) {
    update_sim_health();
    if (!sim_healthy_) { cmd = Twist{}; prev_cmd_ = Twist{}; }

    cmd.linear.x  = clamp_vel(cmd.linear.x,  max_linear_);
    cmd.linear.y  = clamp_vel(cmd.linear.y,  max_linear_);
    cmd.linear.z  = clamp_vel(cmd.linear.z,  max_linear_);
    cmd.angular.y = clamp_vel(cmd.angular.y, max_angular_);
    cmd.angular.z = clamp_vel(cmd.angular.z, max_angular_);

    if (have_drone_pose_) {
        double pose_age = (now() - last_drone_pose_time_).seconds();
        if (pose_age < safety_pose_max_age_ &&
            drone_z_ < safety_min_altitude_ &&
            cmd.linear.z < 0.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Floor clamp: z=%.2f below %.2f", drone_z_, safety_min_altitude_);
            cmd.linear.z = 0.0;
        }
    }

    ramp_velocity(cmd);

    pub_cmd_->publish(cmd);
    prev_cmd_              = cmd;
    last_cmd_publish_time_ = now();
    cmd_count_++;

    publish_bench_state(false);
}

void ServoFsmNode::ramp_velocity(Twist& cmd) {
    auto ramp = [this](double cur, double prev) {
        double delta = cur - prev;
        if (std::abs(delta) > vel_ramp_rate_)
            return prev + std::copysign(vel_ramp_rate_, delta);
        return cur;
    };
    cmd.linear.x  = ramp(cmd.linear.x,  prev_cmd_.linear.x);
    cmd.linear.y  = ramp(cmd.linear.y,  prev_cmd_.linear.y);
    cmd.linear.z  = ramp(cmd.linear.z,  prev_cmd_.linear.z);
    cmd.angular.y = ramp(cmd.angular.y, prev_cmd_.angular.y);
    cmd.angular.z = ramp(cmd.angular.z, prev_cmd_.angular.z);
}

// ─────────────────────────────────────────────────────────────────
void ServoFsmNode::watchdog_tick() {
    double det_age = (now() - last_det_callback_time_).seconds();
    if (det_age > 2.0 * watchdog_period_sec_) {
        consecutive_dets_ = 0;
        handle_no_detection();
    }
}

void ServoFsmNode::diag_tick() {
    auto t = now();
    double dt = (t - diag_window_start_).seconds();
    if (dt < 0.5) return;

    double tgt_dist = 0.0, tgt_bearing_err_deg = 0.0;
    if (have_drone_pose_ && have_target_pose_) {
        double dx = target_x_ - drone_x_;
        double dy = target_y_ - drone_y_;
        tgt_dist = std::sqrt(dx*dx + dy*dy);
        tgt_bearing_err_deg = wrap_to_pi(std::atan2(dy, dx) - drone_yaw_rad_) * 180.0 / M_PI;
    }
    SimHealth sh = compute_sim_health();
    const char* sh_str = (sh == SimHealth::HEALTHY) ? "OK" :
                         (sh == SimHealth::PAUSED) ? "PAUSED" : "DEAD";
    RCLCPP_INFO(get_logger(),
        "[diag] state=%s ctrl=%s | /yolo=%.1f /pose=%.1f /hb=%.1f /cmd=%.1f | "
        "sim=%s sim_t=%.2f | xyz=(%.1f,%.1f,%.1f) yaw=%.2f | tgt dist=%.1fm berr=%+.1f deg",
        state_name(state_), controller_name_.c_str(),
        det_count_/dt, pose_count_/dt, hb_count_/dt, cmd_count_/dt,
        sh_str, last_hb_sim_, drone_x_, drone_y_, drone_z_, drone_yaw_rad_,
        tgt_dist, tgt_bearing_err_deg);

    det_count_ = pitch_count_ = pose_count_ = hb_count_ = cmd_count_ = 0;
    diag_window_start_ = t;
    publish_bench_state(true);
}

// ─────────────────────────────────────────────────────────────────
// /bench/state — "<STATE>,<sim_time>,<controller>,<dist_to_target>"
// The analysis script keys t=0 to the first SEARCHING→APPROACHING edge,
// and uses dist_to_target as a convenience (ground truth is also in the bag).
// ─────────────────────────────────────────────────────────────────
void ServoFsmNode::publish_bench_state(bool /*force*/) {
    double dist = -1.0;
    if (have_drone_pose_ && have_target_pose_) {
        double dx = target_x_ - drone_x_;
        double dy = target_y_ - drone_y_;
        dist = std::sqrt(dx*dx + dy*dy);
    }
    String s;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s,%.4f,%s,%.4f",
        state_name(state_), last_hb_sim_, controller_name_.c_str(), dist);
    s.data = buf;
    pub_bench_state_->publish(s);
}

// ─────────────────────────────────────────────────────────────────
void ServoFsmNode::push_smooth(std::deque<double>& hist, double val) {
    hist.push_back(val);
    while (static_cast<int>(hist.size()) > smoothing_window_) hist.pop_front();
}

double ServoFsmNode::avg(const std::deque<double>& hist) const {
    if (hist.empty()) return 0.0;
    double sum = 0.0;
    for (double v : hist) sum += v;
    return sum / static_cast<double>(hist.size());
}

double ServoFsmNode::clamp_vel(double v, double lim) const {
    return std::clamp(v, -lim, lim);
}

double ServoFsmNode::wrap_to_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double ServoFsmNode::deg2rad(double d) { return d * M_PI / 180.0; }

}  // namespace servo_core