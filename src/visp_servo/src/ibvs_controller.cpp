// ─────────────────────────────────────────────────────────────────
// visp_servo/ibvs_controller.cpp   (TS1) — thin orchestration.
// ─────────────────────────────────────────────────────────────────
#include "visp_servo/ibvs_controller.hpp"

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>

#include "visp_servo/ibvs_law.hpp"
#include "visp_servo/depth/bbox_depth_source.hpp"
#include "visp_servo/depth/slam_depth_source.hpp"

namespace visp_servo {

IBVSController::IBVSController(rclcpp::Node* node) : node_(node) {
    lambda_         = node_->declare_parameter<double>("lambda", 0.5);
    use_slam_depth_ = node_->declare_parameter<bool>("use_slam_depth", false);

    // Pick the depth module. Adding SLAM is exactly this branch + its subs.
    if (use_slam_depth_)
        depth_ = std::make_unique<SlamDepthSource>(node_);
    else
        depth_ = std::make_unique<BboxDepthSource>();

    RCLCPP_INFO(node_->get_logger(),
        "IBVSController (TS1): lambda=%.2f, depth='%s'", lambda_, depth_->name());
}

void IBVSController::init(const servo_core::ServoInputs& cfg) {
    image_width_       = cfg.image_width;
    image_height_      = cfg.image_height;
    target_bbox_ratio_ = cfg.target_bbox_ratio;
    known_height_      = cfg.known_target_height;

    cam_.initPersProjWithoutDistortion(cfg.fx, cfg.fy, cfg.cx0, cfg.cy0);
    depth_->init(cfg);

    servo_.setServo(vpServo::EYEINHAND_CAMERA);
    servo_.setInteractionMatrixType(vpServo::CURRENT);
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
    // desired features (centred, target height, same aspect)
    Corners des = centered_box(image_width_, image_height_, target_bbox_ratio_, aspect);
    double  Z_des = depth_for_box_height(cam_, known_height_, des.y2 - des.y1);

    try {
        update_corner_features(cam_, cur, Z_cur,  s_tl_,   s_tr_,   s_br_,   s_bl_);
        update_corner_features(cam_, des, Z_des,  s_tl_d_, s_tr_d_, s_br_d_, s_bl_d_);
        vpColVector v = servo_.computeControlLaw();          // v = -lambda*L^+*(s-s*)
        return camera_twist_to_body(v);
    } catch (const vpException& e) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "IBVS computeControlLaw failed: %s — cmd 0", e.what());
        return {};
    }
}

}  // namespace visp_servo