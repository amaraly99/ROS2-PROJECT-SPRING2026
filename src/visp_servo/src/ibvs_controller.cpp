// ─────────────────────────────────────────────────────────────────
// visp_servo/ibvs_controller.cpp   (TS1) — thin orchestration.
// ─────────────────────────────────────────────────────────────────
#include "visp_servo/ibvs_controller.hpp"

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>

#include "visp_servo/ibvs_law.hpp"
#include "visp_servo/depth/bbox_depth_source.hpp"
#include "visp_servo/depth/slam_depth_source.hpp"
#include "visp_servo/slam_pose_source.hpp"

namespace visp_servo {

IBVSController::IBVSController(rclcpp::Node* node) : node_(node) {
    lambda_         = node_->declare_parameter<double>("lambda", 0.3);
    k_fwd_          = node_->declare_parameter<double>("k_fwd",   3.0);
    use_slam_depth_   = node_->declare_parameter<bool>("use_slam_depth", false);
    slam_depth_scale_ = node_->declare_parameter<double>("slam_depth_scale", 1.0);
    use_slam_pose_    = node_->declare_parameter<bool>("use_slam_pose", false);
    standoff_m_       = node_->declare_parameter<double>("standoff_m", 3.0);

    // Depth module: bbox or SLAM point-cloud.
    if (use_slam_depth_)
        depth_ = std::make_unique<SlamDepthSource>(node_, slam_depth_scale_);
    else
        depth_ = std::make_unique<BboxDepthSource>();

    // Pose gate for vx switching (Option 1: staleness, Option 2: +tracking state).
    // Subscribes /slam/pose + /slam/tracking_state; if neither is published the
    // gate never opens and vx falls back to bbox-ratio unconditionally.
    if (use_slam_pose_) {
        if (!use_slam_depth_)
            RCLCPP_WARN(node_->get_logger(),
                "use_slam_pose=true but use_slam_depth=false — "
                "Z_cur for vx comes from bbox, not SLAM range. "
                "Set use_slam_depth=true for metric approach control.");
        slam_pose_ = std::make_unique<SlamPoseSource>(node_);
    }

    RCLCPP_INFO(node_->get_logger(),
        "IBVSController (TS1): lambda=%.2f k_fwd=%.2f depth='%s' slam_pose=%s standoff=%.1fm",
        lambda_, k_fwd_, depth_->name(),
        use_slam_pose_ ? "ON" : "OFF", standoff_m_);
}

void IBVSController::init(const servo_core::ServoInputs& cfg) {
    image_width_       = cfg.image_width;
    image_height_      = cfg.image_height;
    target_bbox_ratio_ = cfg.target_bbox_ratio;
    known_height_      = cfg.known_target_height;

    cam_.initPersProjWithoutDistortion(cfg.fx, cfg.fy, cfg.cx0, cfg.cy0);
    depth_->init(cfg);

    servo_.setServo(vpServo::EYEINHAND_CAMERA);
    servo_.setInteractionMatrixType(vpServo::MEAN);
    servo_.setLambda(lambda_);
    servo_.addFeature(s_tl_, s_tl_d_);
    servo_.addFeature(s_tr_, s_tr_d_);
    servo_.addFeature(s_br_, s_br_d_);
    servo_.addFeature(s_bl_, s_bl_d_);

    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(),
        "IBVS ready: setpoint ratio=%.2f at %dx%d", target_bbox_ratio_,
        image_width_, image_height_);
}

servo_core::ServoVel IBVSController::computeApproach(const servo_core::ServoInputs& in) {
    if (!initialized_) return {};

    const double aspect = (in.bh > 1.0) ? (in.bw / in.bh) : 1.0;
    const double Z_cur  = depth_->depth(in);                 // swappable source

    // current features
    Corners cur = box_from_center(in.cx, in.cy, in.bw, in.bh);
    // desired = same size as current, centred → error is CENTERING only (no size error).
    // Size/range is handled by the proportional vx term below, not by IBVS.
    // This prevents size-error dominance from collapsing the yaw/lateral channels in L^+.
    Corners des = centered_box(image_width_, image_height_, in.bbox_ratio, aspect);

    try {
        update_corner_features(cam_, cur, Z_cur, s_tl_,   s_tr_,   s_br_,   s_bl_);
        update_corner_features(cam_, des, Z_cur, s_tl_d_, s_tr_d_, s_br_d_, s_bl_d_);
        vpColVector v = servo_.computeControlLaw();          // v = -lambda*L^+*(s-s*)
        auto vel = camera_twist_to_body(v);
        // vx: SLAM-range approach when gate is open, else bbox-ratio (TS2-style).
        // Option 1 (staleness) + Option 2 (tracking state) are both in is_usable().
        if (slam_pose_ && slam_pose_->is_usable()) {
            vel.vx = k_fwd_ * std::max(0.0, Z_cur - standoff_m_);
        } else {
            vel.vx = k_fwd_ * std::max(0.0, target_bbox_ratio_ - in.bbox_ratio);
        }
        return vel;
    } catch (const vpException& e) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "IBVS computeControlLaw failed: %s — cmd 0", e.what());
        return {};
    }
}

}  // namespace visp_servo