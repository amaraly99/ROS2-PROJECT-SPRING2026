#pragma once

// ─────────────────────────────────────────────────────────────────
// h_vs_servo/hvs_controller.hpp   (TS3)
//
// Adapts Benhimane & Malis homography-based 2D visual servoing
// (RViMLab/h_vs) to the IServoController interface.
//
// Instead of detecting a calibration pattern in a camera image
// (as in the original h_vs), it computes the projective homography G
// directly from the oracle's bounding box corners — 4 exact point
// correspondences between the desired and current configurations.
//
// Control law (zero interaction matrix, zero depth estimate):
//   H   = K^{-1} G K          (projective → euclidean homography)
//   e_v = (H − I) m*           (translational error, eq. 15)
//   e_w = vex(H − H^T)         (rotational error,    eq. 16)
//   v_c = −λ_v ⊙ e_v           (camera-frame linear velocity)
//   ω_c = −λ_w ⊙ e_w           (camera-frame angular velocity)
//
// Output mapped to Simulink body frame (x=fwd, y=left, z=up):
//   vx = proportional approach (bbox scale error) — homography cannot produce
//        forward velocity for axis-aligned oracle bboxes because getPerspective-
//        Transform returns a pure affine (H[2,2]=1 → e_v[2]=0 always).
//   vy = −v_c[0]  (−Xc → left)
//   vz = −v_c[1]  (−Yc → up)
//   wz = −ω_c[1]  (−ry → yaw-left; ry = rotation around Yc = camera yaw)
// ─────────────────────────────────────────────────────────────────
#ifndef H_VS_SERVO_HVS_CONTROLLER_HPP
#define H_VS_SERVO_HVS_CONTROLLER_HPP

#include <deque>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "servo_core/servo_controller.hpp"
#include "h_vs_servo/homography_2d_vs.h"

namespace h_vs_servo {

class HVSController : public servo_core::IServoController {
public:
    explicit HVSController(rclcpp::Node* node);

    void init(const servo_core::ServoInputs& cfg) override;
    servo_core::ServoVel computeApproach(const servo_core::ServoInputs& in) override;
    const char* name() const override { return "h_vs"; }

private:
    rclcpp::Node* node_;

    Homography2DVisualServo hvs_;

    // Desired bbox corners in pixels — set once at init(), never change.
    std::vector<cv::Point2f> pts_star_;

    // Moving average buffer (same noise-rejection as original h_vs).
    std::deque<Eigen::VectorXd> twist_buffer_;
    int buffer_len_{5};

    bool initialized_{false};

    // Forward approach gain — same proportional law as hil_servo (TS2).
    // The homography law handles lateral (vy) and yaw (wz); vx must come
    // from outside because H[2,2]=1 for axis-aligned oracle bboxes.
    double k_fwd_{3.0};
    double target_bbox_ratio_{0.55};
};

}  // namespace h_vs_servo

#endif  // H_VS_SERVO_HVS_CONTROLLER_HPP