// ─────────────────────────────────────────────────────────────────
// servo_core/servo_fsm_node.hpp
//
// Shared FSM + safety-filter + instrumentation node. This is TS2's
// proven state machine (SEARCHING → APPROACHING ↔ REACQUIRE → REACHED)
// with the APPROACHING velocity factored out into an IServoController.
//
// A test-subject executable is just:
//     auto node = std::make_shared<servo_core::ServoFsmNode>("hil_servo_node");
//     node->set_controller(std::make_unique<ProportionalController>(node.get()));
//     rclcpp::spin(node);
//
// Publishes /bench/state (std_msgs/String "<STATE>,<sim_time>,<controller>")
// on every transition and at diag rate — the analysis script keys t=0 to the
// first SEARCHING→APPROACHING edge.
// ─────────────────────────────────────────────────────────────────
#ifndef SERVO_CORE_SERVO_FSM_NODE_HPP
#define SERVO_CORE_SERVO_FSM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "servo_core/servo_controller.hpp"

namespace servo_core {

class ServoFsmNode : public rclcpp::Node {
public:
    explicit ServoFsmNode(const std::string& node_name);

    // Inject the pluggable servoing law. Must be called once, before spin().
    void set_controller(std::unique_ptr<IServoController> controller);

private:
    using DetectionArray = yolo_msgs::msg::DetectionArray;
    using Detection      = yolo_msgs::msg::Detection;
    using Twist          = geometry_msgs::msg::Twist;
    using Float64        = std_msgs::msg::Float64;
    using Float64MA      = std_msgs::msg::Float64MultiArray;
    using String         = std_msgs::msg::String;

    enum class State { SEARCHING, APPROACHING, REACQUIRE, REACHED };
    static const char* state_name(State s);

    enum class SearchStep { FULL_ROTATE, YAW_RIGHT_60, YAW_LEFT_60, YAW_CENTER, STRAFE_RIGHT };
    static const char* search_step_name(SearchStep s);

    // ── setup ──
    void declare_parameters();
    void load_parameters();
    void setup_ros();

    // ── callbacks ──
    void on_detections(const DetectionArray::SharedPtr msg);
    void on_drone_pose(const Float64MA::SharedPtr msg);
    void on_target_pose(const Float64MA::SharedPtr msg);
    void on_pitch(const Float64::SharedPtr msg);
    void on_heartbeat(const Float64::SharedPtr msg);

    // ── FSM ──
    void update_state_on_detection(double bbox_ratio, double ex_norm,
                                   double ey_norm, double conf);
    void handle_no_detection();
    Twist build_command(double ex_norm, double ey_norm, double bbox_ratio);
    Twist build_reacquire_command();
    Twist build_search_command();
    void advance_search_step();
    void reset_search();
    void seed_search_from_last_bearing();
    void handle_sim_restart();

    // ── sim liveness ──
    enum class SimHealth { HEALTHY, PAUSED, DEAD };
    SimHealth compute_sim_health();
    void update_sim_health();

    // ── pitch ──
    double pitch_for_control();
    double pitch_return_cmd();

    // ── publish-time filters ──
    void publish_cmd_vel(Twist cmd);
    void ramp_velocity(Twist& cmd);

    // ── timers ──
    void watchdog_tick();
    void diag_tick();

    // ── instrumentation ──
    void publish_bench_state(bool force);

    // ── utils ──
    void   push_smooth(std::deque<double>& hist, double val);
    double avg(const std::deque<double>& hist) const;
    double clamp_vel(double v, double lim) const;
    static double wrap_to_pi(double a);
    static double deg2rad(double d);

    // Build the ServoInputs handed to the controller for one APPROACHING tick.
    ServoInputs make_inputs(double ex_norm, double ey_norm,
                            double cx, double cy, double bw, double bh,
                            double bbox_ratio) const;

    // ── the pluggable law ──
    std::unique_ptr<IServoController> controller_;
    std::string controller_name_{"unset"};

    // ── parameters (FSM / filters / geometry) ──
    std::string              target_class_;
    std::vector<std::string> target_classes_;
    double min_confidence_;
    int    image_width_, image_height_;

    // Intrinsics + physical target height (for depth + IBVS desired features).
    double cam_fx_, cam_fy_, cam_cx_, cam_cy_;
    double known_target_height_;

    double k_pitch_return_, pitch_zero_tol_;
    double max_linear_, max_angular_;
    double vel_ramp_rate_;

    double target_bbox_ratio_;
    double hold_bbox_ratio_;
    double hold_center_tol_;
    int    reach_consec_ticks_;

    int    lockon_consec_;
    double lockon_ex_tol_;
    double k_lockon_bias_;

    double search_yaw_target_deg_;
    double k_search_yaw_;
    double search_yaw_arrive_tol_rad_;
    double search_settle_sec_;
    double search_strafe_speed_;
    double search_strafe_dur_sec_;
    double search_spin_speed_;
    double search_full_rotate_sec_;

    double base_reacquire_sec_;
    double k_persist_;
    double max_reacquire_sec_;

    int    heartbeat_window_samples_;
    double heartbeat_gap_sec_;
    double heartbeat_dead_sec_;
    double sim_paused_rate_;
    double sim_recovery_debounce_sec_;
    double sim_unhealthy_entry_sec_;

    double safety_min_altitude_;
    double safety_pose_max_age_;

    double watchdog_period_sec_;
    double diag_period_sec_;
    int    smoothing_window_;

    // ── runtime state ──
    State state_ = State::SEARCHING;

    SearchStep   search_step_     = SearchStep::FULL_ROTATE;
    double       yaw_target_rad_  = 0.0;
    bool         yaw_target_init_ = false;
    rclcpp::Time search_arrive_time_;
    bool         search_arrived_  = false;
    rclcpp::Time strafe_start_time_;
    rclcpp::Time spin_start_time_;
    double       drone_yaw_at_search_start_ = 0.0;

    int          consecutive_dets_ = 0;
    rclcpp::Time approach_start_time_;
    int          reach_ticks_ = 0;
    double       last_bearing_to_sign_rad_ = 0.0;
    bool         have_last_bearing_ = false;

    rclcpp::Time reacquire_start_time_;
    double       active_reacquire_timeout_sec_ = 0.0;

    std::deque<double> cx_hist_, cy_hist_, bw_hist_, bh_hist_;
    double prev_cx_ = -1.0, prev_cy_ = -1.0;
    // Freshest smoothed bbox (set each detection, read by build_command).
    double sm_cx_ = 0.0, sm_cy_ = 0.0, sm_bw_ = 0.0, sm_bh_ = 0.0;

    double       current_pitch_rad_ = 0.0;
    rclcpp::Time last_pitch_msg_time_;
    bool         have_pitch_msg_ = false;

    double drone_x_ = 0.0, drone_y_ = 15.0, drone_z_ = 10.0;
    double drone_yaw_rad_ = 0.0;
    rclcpp::Time last_drone_pose_time_;
    bool   have_drone_pose_ = false;

    double target_x_ = 0.0, target_y_ = 0.0, target_z_ = 0.0, target_yaw_ = 0.0;
    bool   have_target_pose_ = false;

    struct HbSample { double sim_t; rclcpp::Time wall_t; };
    std::deque<HbSample> hb_buffer_;
    rclcpp::Time         last_hb_wall_time_;
    bool                 have_hb_ = false;
    bool                 sim_healthy_ = true;
    double               last_hb_sim_ = 0.0;
    bool                 prev_obs_healthy_ = true;
    bool                 have_fresh_sim_ = false;  // must see sim_t<threshold before FSM runs
    double               fresh_sim_threshold_sec_ = 2.0;
    rclcpp::Time         healthy_streak_start_;
    rclcpp::Time         unhealthy_streak_start_;

    Twist prev_cmd_{};

    // ── ROS interfaces ──
    rclcpp::Subscription<DetectionArray>::SharedPtr sub_dets_;
    rclcpp::Subscription<Float64>::SharedPtr        sub_pitch_;
    rclcpp::Subscription<Float64MA>::SharedPtr      sub_drone_pose_;
    rclcpp::Subscription<Float64MA>::SharedPtr      sub_target_pose_;
    rclcpp::Subscription<Float64>::SharedPtr        sub_heartbeat_;

    rclcpp::Publisher<Twist>::SharedPtr  pub_cmd_;
    rclcpp::Publisher<String>::SharedPtr pub_bench_state_;

    rclcpp::TimerBase::SharedPtr diag_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // diagnostics
    rclcpp::Time diag_window_start_;
    int det_count_ = 0, pitch_count_ = 0, pose_count_ = 0, hb_count_ = 0, cmd_count_ = 0;
    rclcpp::Time last_cmd_publish_time_;
    rclcpp::Time last_det_callback_time_;
    bool have_det_callback_ = false;
    rclcpp::Time last_log_time_;
};

}  // namespace servo_core

#endif  // SERVO_CORE_SERVO_FSM_NODE_HPP