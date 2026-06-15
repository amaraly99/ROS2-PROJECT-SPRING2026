// ─────────────────────────────────────────────────────────────────
// visp_servo/ibvs_law.cpp — pure IBVS math helpers (one job each).
// ─────────────────────────────────────────────────────────────────
#include "visp_servo/ibvs_law.hpp"

#include <visp3/core/vpPixelMeterConversion.h>

namespace visp_servo {

Corners box_from_center(double cx, double cy, double bw, double bh) {
    return {cx - bw / 2.0, cy - bh / 2.0, cx + bw / 2.0, cy + bh / 2.0};
}

Corners centered_box(int image_w, int image_h, double height_ratio, double aspect) {
    double h = height_ratio * image_h;
    double w = h * aspect;
    double cx = image_w / 2.0, cy = image_h / 2.0;
    return {cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0};
}

double depth_for_box_height(const vpCameraParameters& cam,
                            double known_height_m, double box_height_px) {
    if (box_height_px < 1.0) box_height_px = 1.0;
    return cam.get_py() * known_height_m / box_height_px;
}

void update_corner_features(const vpCameraParameters& cam,
                            const Corners& c, double Z,
                            vpFeaturePoint& tl, vpFeaturePoint& tr,
                            vpFeaturePoint& br, vpFeaturePoint& bl) {
    double xn, yn;
    vpPixelMeterConversion::convertPoint(cam, c.x1, c.y1, xn, yn); tl.buildFrom(xn, yn, Z);
    vpPixelMeterConversion::convertPoint(cam, c.x2, c.y1, xn, yn); tr.buildFrom(xn, yn, Z);
    vpPixelMeterConversion::convertPoint(cam, c.x2, c.y2, xn, yn); br.buildFrom(xn, yn, Z);
    vpPixelMeterConversion::convertPoint(cam, c.x1, c.y2, xn, yn); bl.buildFrom(xn, yn, Z);
}

servo_core::ServoVel camera_twist_to_body(const vpColVector& v) {
    servo_core::ServoVel out;
    out.vx = v[2];     // cam Z → forward
    out.vy = -v[0];    // -cam X → left
    out.vz = -v[1];    // -cam Y → up
    out.wz = -v[4];    // -cam wy → yaw-left
    if (out.vx < 0.0) out.vx = 0.0;   // approach is forward-only
    return out;
}

}  // namespace visp_servo