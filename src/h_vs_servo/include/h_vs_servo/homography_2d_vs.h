#pragma once

// ─────────────────────────────────────────────────────────────────
// Copied verbatim from RViMLab/h_vs (ROS1 upstream).
// Pure Eigen — zero ROS dependency. Do not modify.
// Reference: Benhimane & Malis, "Homography-based 2D Visual Servoing",
//            ICRA 2006. https://ieeexplore.ieee.org/document/1642061
// ─────────────────────────────────────────────────────────────────

#include <Eigen/Core>
#include <Eigen/LU>

class Homography2DVisualServo {

    public:

        Homography2DVisualServo(Eigen::Matrix3d& K, Eigen::Vector3d& lambda_v, Eigen::Vector3d& lambda_w);
        Homography2DVisualServo() = default;

        // eq. 15 and 16 — with explicit control point p_star
        Eigen::VectorXd computeFeedback(Eigen::Matrix3d& G, Eigen::Vector3d& p_star);
        // eq. 15 and 16 — defaults p_star to principal point (K(0,2), K(1,2))
        Eigen::VectorXd computeFeedback(Eigen::Matrix3d& G);

        Eigen::MatrixXd K() const { return _K; };
        void K(Eigen::Matrix3d& K) { _K = std::move(K); };

    private:

        Eigen::Matrix3d _K;          // camera intrinsics, eq. 3
        Eigen::Vector3d _lambda_v;   // translational gain, eq. 20
        Eigen::Vector3d _lambda_w;   // rotational gain, eq. 20

        // eq. 15 — translational error from euclidean homography H
        Eigen::Vector3d _computeEv(Eigen::Matrix3d& H, Eigen::Vector3d& m_star);
        // eq. 16 — rotational error from antisymmetric part of H
        Eigen::Vector3d _computeEw(Eigen::Matrix3d& H);
};