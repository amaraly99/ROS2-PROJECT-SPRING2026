#ifndef VISP_PBVS_SERVO_PBVS_CONTROLLER_HPP
#define VISP_PBVS_SERVO_PBVS_CONTROLLER_HPP

#include <opencv2/calib3d.hpp> // for PNP solving
#include <visp3/core/vpHomogeneousMatrix.h> // for 4x4 pose matrix
#include <visp3/visual_features/vpFeatureTranslation.h> // for translation error feature
#include <visp3/visual_features/vpFeatureThetaU.h> //rotation error feature (axis-angle)
#include <visp3/vs/vpServo.h> // for the servo task
#include <visp3/core/vpCameraParameters.h> // for camera parameters
#include "servo_core/servo_controller.hpp" //for the base class
#include <rclcpp/rclcpp.hpp> // for ROS2 node


class PBVSController : public servo_core::IServoController {
private:
    rclcpp::Node* node_;
    double lambda_;
    vpCameraParameters cam_;
    vpServo servo_;
    vpHomogeneousMatrix cMo_desired_;
    double sign_width_, sign_height_;
    bool initialized_;
    vpFeatureTranslation ft_{vpFeatureTranslation::cdMc};
    vpFeatureThetaU      ftu_{vpFeatureThetaU::cdRc};


public:
    PBVSController(rclcpp::Node* node);
    void init(const servo_core::ServoInputs& cfg) override;
    servo_core::ServoVel computeApproach(const servo_core::ServoInputs& in) override;
    const char* name() const override {return "pbvs";}
};   


#endif // VISP_PBVS_SERVO_PBVS_CONTROLLER_HPP