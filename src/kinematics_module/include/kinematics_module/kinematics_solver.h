#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <vector>
#include <utility>

namespace KinematicsSolver
{
    using SE3 = Sophus::SE3d;
    using SO3 = Sophus::SO3d;
    using Twist = Sophus::Vector6d;
    using Vector3d = Eigen::Vector3d;
    using Rotation3d = Eigen::Matrix3d;

    class KinematicsSolver
    {
    public:
        KinematicsSolver(const KDL::Chain& kdl_chain){
            initial(kdl_chain);
        }
        KinematicsSolver() = default;
        KinematicsSolver& operator=(const KinematicsSolver& other) = delete;
        KinematicsSolver(const KinematicsSolver& other) = delete;

        // Initialize the kinematics solver with the given KDL chain
        bool initial(const KDL::Chain& kdl_chain);

        // Compute the forward kinematics for the given joint angles
        SE3 ForwardKinematics(const std::vector<double>& joint_angles) const;

        ~KinematicsSolver() = default;
    private:
        
        SE3 M_zero_; // initial pose
        std::vector<SE3> M_list_; // transformation matrices
        std::vector<Twist> screws_; // screw axes
        std::vector<std::pair<double, double>> joint_limits_; // joint limits
    };
}