// ─────────────────────────────────────────────────────────────────
// h_vs_servo/hvs_controller.cpp   (TS3)
// ─────────────────────────────────────────────────────────────────
#include "h_vs_servo/hvs_controller.hpp"

namespace h_vs_servo {

HVSController::HVSController(rclcpp::Node* node) : node_(node) {
    std::vector<double> lv = node_->declare_parameter<std::vector<double>>(
        "lambda_v", {0.5, 0.5, 0.5});
    std::vector<double> lw = node_->declare_parameter<std::vector<double>>(
        "lambda_w", {0.3, 0.3, 0.3});
    buffer_len_  = node_->declare_parameter<int>("twist_buffer_len", 5);
    k_fwd_       = node_->declare_parameter<double>("k_fwd", 3.0);

    Eigen::Vector3d lambda_v(lv[0], lv[1], lv[2]);
    Eigen::Vector3d lambda_w(lw[0], lw[1], lw[2]);

    // K placeholder — replaced in init() once the FSM hands us intrinsics.
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    hvs_ = Homography2DVisualServo(K, lambda_v, lambda_w);

    RCLCPP_INFO(node_->get_logger(),
        "HVSController (TS3): lambda_v=[%.2f,%.2f,%.2f] lambda_w=[%.2f,%.2f,%.2f] "
        "k_fwd=%.2f buf=%d",
        lv[0], lv[1], lv[2], lw[0], lw[1], lw[2], k_fwd_, buffer_len_);
}

void HVSController::init(const servo_core::ServoInputs& cfg) {
    // Build K from intrinsics the FSM passes after parameter load.
    Eigen::Matrix3d K;
    K << cfg.fx,    0.0, cfg.cx0,
          0.0, cfg.fy, cfg.cy0,
          0.0,    0.0,   1.0;
    hvs_.K(K);

    // Desired configuration: bbox centered on principal point, height =
    // target_bbox_ratio * image_height.  Aspect = 1.0 (matches oracle).
    double bh_star = cfg.target_bbox_ratio * cfg.image_height;
    double bw_star = bh_star;
    double cx0 = cfg.cx0, cy0 = cfg.cy0;

    pts_star_ = {
        {(float)(cx0 - bw_star / 2.0), (float)(cy0 - bh_star / 2.0)},  // TL
        {(float)(cx0 + bw_star / 2.0), (float)(cy0 - bh_star / 2.0)},  // TR
        {(float)(cx0 + bw_star / 2.0), (float)(cy0 + bh_star / 2.0)},  // BR
        {(float)(cx0 - bw_star / 2.0), (float)(cy0 + bh_star / 2.0)},  // BL
    };

    target_bbox_ratio_ = cfg.target_bbox_ratio;
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(),
        "HVSController init: desired bbox %.1fpx x %.1fpx centred at (%.0f,%.0f)",
        bw_star, bh_star, cx0, cy0);
}

servo_core::ServoVel HVSController::computeApproach(const servo_core::ServoInputs& in) {
    if (!initialized_) return {};

    // ── 1. Current bbox corners (from oracle, pixel coordinates) ─────
    float hw = (float)(in.bw / 2.0);
    float hh = (float)(in.bh / 2.0);
    std::vector<cv::Point2f> pts_curr = {
        {(float)in.cx - hw, (float)in.cy - hh},  // TL
        {(float)in.cx + hw, (float)in.cy - hh},  // TR
        {(float)in.cx + hw, (float)in.cy + hh},  // BR
        {(float)in.cx - hw, (float)in.cy + hh},  // BL
    };

    // ── 2. Projective homography G: maps desired corners → current ───
    // 4 exact correspondences from a perfect oracle → no RANSAC needed.
    cv::Mat G_cv = cv::getPerspectiveTransform(pts_star_, pts_curr);

    Eigen::Matrix3d G;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            G(i, j) = G_cv.at<double>(i, j);

    // ── 3. Benhimane & Malis control law → camera-frame 6-DOF twist ─
    //   H   = K^{-1} G K
    //   e_v = (H − I) m*          (eq. 15)
    //   e_w = vex(H − H^T)        (eq. 16)
    //   twist = [−λ_v⊙e_v, −λ_w⊙e_w]   (camera frame)
    Eigen::VectorXd twist = hvs_.computeFeedback(G);
    // twist indices: [0]=tx_c [1]=ty_c [2]=tz_c [3]=rx_c [4]=ry_c [5]=rz_c

    // ── 4. Moving average for noise rejection ────────────────────────
    twist_buffer_.push_back(twist);
    if ((int)twist_buffer_.size() > buffer_len_) twist_buffer_.pop_front();

    Eigen::VectorXd avg = Eigen::VectorXd::Zero(6);
    for (const auto& t : twist_buffer_) avg += t / (double)twist_buffer_.size();

    // ── 5. Camera frame → Simulink body frame (x=fwd, y=left, z=up) ─
    //   Camera: Xc=right, Yc=down, Zc=forward
    //   Body:   x=forward, y=left,  z=up
    //
    // vx: The oracle always produces axis-aligned bounding boxes, so
    //   getPerspectiveTransform returns a pure affine G (G[2,0]=G[2,1]=0,
    //   G[2,2]=1). This makes H[2,2]=1 → e_v[2]=0 regardless of range.
    //   The homography law can never produce forward velocity from axis-aligned
    //   bboxes. Use the same proportional approach as hil_servo (TS2) for vx.
    //
    // vy/vz: e_v[0] and e_v[1] correctly encode lateral and vertical offset
    //   of the current bbox center vs the desired center (principal point).
    //   G maps star→curr so H[0,2] = (cx_curr - cx0)/fx and
    //   H[1,2] = (cy_curr - cy0)/fy. Negating gives the right body direction.
    //
    // wz: body yaw = rotation around body z (up) = rotation around camera Yc.
    //   That is e_w[1] = twist[4], not e_w[2] = twist[5] (which is camera roll).
    //   Matches ibvs_law.cpp: out.wz = -v[4].
    servo_core::ServoVel v;
    v.vx = std::max(0.0, k_fwd_ * (target_bbox_ratio_ - in.bbox_ratio));
    v.vy = -avg(0);   // −Xc → body left
    v.vz = -avg(1);   // −Yc → body up
    v.wz = -avg(4);   // −ry (camera Yc rotation = yaw) → body yaw-left
    return v;
}

}  // namespace h_vs_servo