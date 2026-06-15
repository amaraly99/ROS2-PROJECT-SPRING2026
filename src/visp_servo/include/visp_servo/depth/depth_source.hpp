// ─────────────────────────────────────────────────────────────────
// visp_servo/depth/depth_source.hpp
//
// Pluggable DEPTH module. The IBVS law needs a range Z per feature; where
// that Z comes from is a swappable concern:
//
//   BboxDepthSource  → Z = fy*H/bh        (default; identical to TS2, fair)
//   SlamDepthSource  → Z from /point_cloud + /vo_pose (fallback: bbox)
//
// Adding SLAM is "a simple subscription": set use_slam_depth:=true and the
// SlamDepthSource constructs the /vo_pose + /point_cloud subscriptions. The
// control law does not change.
// ─────────────────────────────────────────────────────────────────
#ifndef VISP_SERVO_DEPTH_SOURCE_HPP
#define VISP_SERVO_DEPTH_SOURCE_HPP

#include "servo_core/servo_controller.hpp"

namespace visp_servo {

class IDepthSource {
public:
    virtual ~IDepthSource() = default;
    // One-time geometry handoff (intrinsics, image size).
    virtual void init(const servo_core::ServoInputs& cfg) { (void)cfg; }
    // Range to the target (metres) for the current detection.
    virtual double depth(const servo_core::ServoInputs& in) = 0;
    virtual const char* name() const = 0;
};

}  // namespace visp_servo

#endif  // VISP_SERVO_DEPTH_SOURCE_HPP