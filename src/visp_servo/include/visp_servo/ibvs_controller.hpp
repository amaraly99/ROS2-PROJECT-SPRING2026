// ─────────────────────────────────────────────────────────────────
// visp_servo/ibvs_controller.hpp   (TS1)
//
// Thin orchestration of the IBVS law:  depth module + ibvs_law helpers +
// a persistent vpServo task. computeApproach() reads like a recipe.
//
//   v = -lambda * L^+ * (s - s*)     (inside vpServo::computeControlLaw)
//
// Depth is a SWAPPABLE module (bbox or SLAM) — see depth/. Set
// use_slam_depth:=true to add SLAM via /vo_pose + /point_cloud, no law change.
// ─────────────────────────────────────────────────────────────────
#ifndef VISP_SERVO_IBVS_CONTROLLER_HPP
#define VISP_SERVO_IBVS_CONTROLLER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#include "servo_core/servo_controller.hpp"
#include "visp_servo/depth/depth_source.hpp"

namespace visp_servo {

class IBVSController : public servo_core::IServoController {
public:
    explicit IBVSController(rclcpp::Node* node);

    void init(const servo_core::ServoInputs& cfg) override;
    servo_core::ServoVel computeApproach(const servo_core::ServoInputs& in) override;
    const char* name() const override { return "ibvs"; }

private:
    rclcpp::Node* node_;
    double lambda_;
    bool   use_slam_depth_;

    std::unique_ptr<IDepthSource> depth_;     // swappable: bbox | slam

    vpCameraParameters cam_;
    vpServo            servo_;
    vpFeaturePoint     s_tl_,   s_tr_,   s_br_,   s_bl_;     // current
    vpFeaturePoint     s_tl_d_, s_tr_d_, s_br_d_, s_bl_d_;   // desired
    bool               initialized_ = false;

    double target_bbox_ratio_ = 0.55;
    int    image_width_  = 640;
    int    image_height_ = 480;
    double known_height_ = 1.5;
};

}  // namespace visp_servo

#endif  // VISP_SERVO_IBVS_CONTROLLER_HPP