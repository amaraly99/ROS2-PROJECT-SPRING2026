// ─────────────────────────────────────────────────────────────────
// servo_core/servo_controller.hpp
//
// The pluggable SERVOING interface. The shared FSM (ServoFsmNode) owns
// the state machine, the safety filters, all ROS plumbing and the
// /bench/state instrumentation. The ONLY thing that differs between
// test subjects is the body-frame velocity computed inside APPROACHING,
// which is delegated to an IServoController.
//
//   TS1 (visp_servo) → IBVSController         v = -lambda * L^+ * (s - s*)
//   TS2 (hil_servo)  → ProportionalController decoupled P-law
//
// Both receive the SAME ServoInputs (identical bbox, identical bbox-based
// depth Z, identical intrinsics) so the comparison isolates the law.
// ─────────────────────────────────────────────────────────────────
#ifndef SERVO_CORE_SERVO_CONTROLLER_HPP
#define SERVO_CORE_SERVO_CONTROLLER_HPP

namespace servo_core {

// Everything a servoing law could need for one APPROACHING tick.
// Image-plane errors are normalized to [-1, 1] (0 = centred).
struct ServoInputs {
    // Normalized centring error (cx-W/2)/(W/2), (cy-H/2)/(H/2).
    double ex_norm{0.0};
    double ey_norm{0.0};

    // Smoothed bounding box in pixels (centre + size).
    double cx{0.0};
    double cy{0.0};
    double bw{0.0};
    double bh{0.0};

    // Apparent-size ratio bh / image_height (proxy for range).
    double bbox_ratio{0.0};

    // Bbox-based depth estimate (metres). Held identical across TS1/TS2
    // so depth is NOT a confound — Z = fy * known_target_height / bh.
    double Z{0.0};

    // Desired apparent-size setpoint (= stop distance). Shared param.
    double target_bbox_ratio{0.55};

    // Geometry the IBVS law needs (the P-law ignores these).
    int    image_width{640};
    int    image_height{480};
    double fx{554.0};
    double fy{554.0};
    double cx0{320.0};
    double cy0{240.0};

    // Known physical target height (m) used for the desired-feature depth.
    double known_target_height{1.5};
};

// Body-frame velocity (Simulink convention):
//   vx forward(+), vy LEFT(+), vz UP(+), wz YAW-LEFT(+).
// The FSM adds the pitch-return (wy) and applies clamp/ramp/floor itself,
// so a controller only returns the four DOFs it actually drives.
struct ServoVel {
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    double wz{0.0};
};

class IServoController {
public:
    virtual ~IServoController() = default;

    // Called once after the node has loaded its parameters. `cfg` carries
    // the intrinsics, image dims and target_bbox_ratio so a controller can
    // build any fixed/desired quantities (e.g. IBVS desired features).
    virtual void init(const ServoInputs& cfg) { (void)cfg; }

    // Called every APPROACHING tick. Returns the body-frame velocity.
    virtual ServoVel computeApproach(const ServoInputs& in) = 0;

    // Short identifier logged at startup and stamped into /bench/state.
    virtual const char* name() const = 0;
};

}  // namespace servo_core

#endif  // SERVO_CORE_SERVO_CONTROLLER_HPP