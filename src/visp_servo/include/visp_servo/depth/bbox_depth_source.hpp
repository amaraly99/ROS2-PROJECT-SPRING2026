// ─────────────────────────────────────────────────────────────────
// visp_servo/depth/bbox_depth_source.hpp
//
// Default depth source: the bbox-based estimate already computed by the
// FSM core (ServoInputs.Z = fy*H/bh). Held identical to TS2 so depth is
// not a confound in the controller comparison.
// ─────────────────────────────────────────────────────────────────
#ifndef VISP_SERVO_BBOX_DEPTH_SOURCE_HPP
#define VISP_SERVO_BBOX_DEPTH_SOURCE_HPP

#include "visp_servo/depth/depth_source.hpp"

namespace visp_servo {

class BboxDepthSource : public IDepthSource {
public:
    double depth(const servo_core::ServoInputs& in) override { return in.Z; }
    const char* name() const override { return "bbox"; }
};

}  // namespace visp_servo

#endif  // VISP_SERVO_BBOX_DEPTH_SOURCE_HPP