// ─────────────────────────────────────────────────────────────────
// visp_servo/ibvs_law.hpp
//
// Pure IBVS math — small, stateless, ROS-free helpers. Each does ONE job.
// The controller just orchestrates these + the vpServo task.
// ─────────────────────────────────────────────────────────────────
#ifndef VISP_SERVO_IBVS_LAW_HPP
#define VISP_SERVO_IBVS_LAW_HPP

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>
#include <visp3/visual_features/vpFeaturePoint.h>

#include "servo_core/servo_controller.hpp"

namespace visp_servo {

// Pixel-space axis-aligned box corners.
struct Corners { double x1, y1, x2, y2; };

// Box around a centre with given width/height (pixels).
Corners box_from_center(double cx, double cy, double bw, double bh);

// Centred desired box: height = ratio*H, width = height*aspect.
Corners centered_box(int image_w, int image_h, double height_ratio, double aspect);

// Desired-feature depth for a box of the given pixel height.
double depth_for_box_height(const vpCameraParameters& cam,
                            double known_height_m, double box_height_px);

// Update the 4 corner features (TL,TR,BR,BL) from pixel corners at depth Z.
void update_corner_features(const vpCameraParameters& cam,
                            const Corners& c, double Z,
                            vpFeaturePoint& tl, vpFeaturePoint& tr,
                            vpFeaturePoint& br, vpFeaturePoint& bl);

// Map a ViSP camera-frame twist [vx vy vz wx wy wz] to the Simulink body
// frame ServoVel (x=fwd, y=LEFT, z=UP, wz=yaw-left).
servo_core::ServoVel camera_twist_to_body(const vpColVector& v);

}  // namespace visp_servo

#endif  // VISP_SERVO_IBVS_LAW_HPP