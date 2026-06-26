#include <rclcpp/rclcpp.hpp>
#include "visp_pbvs_servo/pbvs_controller.hpp"

PBVSController::PBVSController(rclcpp::Node* node) : node_(node) {
    lambda_ = node_->declare_parameter<double>("lambda", 0.5);
    sign_width_ = node_->declare_parameter<double>("sign_width", 0.6);
    initialized_ = false;
}

void PBVSController::init(const servo_core::ServoInputs& cfg) {
    sign_height_ = cfg.known_target_height;
    cam_.initPersProjWithoutDistortion(cfg.fx, cfg.fy, cfg.cx0, cfg.cy0);
    servo_.setServo(vpServo::EYEINHAND_CAMERA);
    servo_.setInteractionMatrixType(vpServo::CURRENT);
    servo_.setLambda(lambda_);
    servo_.addFeature(ft_);
    servo_.addFeature(ftu_);

    double standoff = cfg.fy*cfg.known_target_height / (cfg.target_bbox_ratio * cfg.image_height);
    cMo_desired_.buildFrom(0,0,standoff, 0,0,0);
    initialized_ = true;
}

servo_core::ServoVel PBVSController::computeApproach(const servo_core::ServoInputs& in) {
    if (!initialized_) return {};
    std::vector<cv::Point2f> img_pts = {
        {float(in.cx - in.bw/2),  float(in.cy - in.bh/2)},  // top-left
        {float(in.cx + in.bw/2),  float(in.cy - in.bh/2)},  // top-right
        {float(in.cx + in.bw/2),  float(in.cy + in.bh/2)},  // bottom-right
        {float(in.cx - in.bw/2),  float(in.cy + in.bh/2)},  // bottom-left
    };
    std::vector<cv::Point3f> object_pts = {
        {-sign_width_/2,  sign_height_/2, 0},  // top-left
        { sign_width_/2,  sign_height_/2, 0},  // top-right
        { sign_width_/2, -sign_height_/2, 0},  // bottom-right
        {-sign_width_/2, -sign_height_/2, 0},  // bottom-left
    }; 
    cv::Mat K = (cv::Mat_<double>(3,3) << 
    in.fx, 0, in.cx0,
    0,in.fy, in.cy0,
    0,0,1);

cv::Mat rvec, tvec;
bool ok = cv::solvePnP(object_pts, img_pts, K, cv::noArray(), rvec, tvec);
if (!ok) return{};
cv::Mat R;
cv::Rodrigues(rvec, R);
vpRotationMatrix Rvp;
for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
        Rvp[i][j] = R.at<double>(i, j);

vpTranslationVector tvp(
    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)
);

vpHomogeneousMatrix cMo_current(tvp, Rvp);

vpHomogeneousMatrix cdMc = cMo_desired_.inverse() * cMo_current;
ft_.buildFrom(cdMc); //build translation from desired
ftu_.buildFrom(cdMc); //build rotation from desired
vpColVector v = servo_.computeControlLaw();
servo_core::ServoVel vel;
vel.vx =  v[2];   // camera Zc (forward) → body forward
vel.vy = -v[0];   // camera Xc (right)   → body left (negate)
vel.vz = 0.0;     // altitude held by FSM pitch-return, not PBVS
vel.wz = -v[5];   // camera wz           → body yaw-left (negate)
return vel;

}