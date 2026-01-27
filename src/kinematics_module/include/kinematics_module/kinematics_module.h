#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_kdl/tf2_kdl.h>
#include <vector>
#include <string>
#include <memory>

namespace KinematicsModule
{
    // IK solver types
    // enum class IK_Solver_Type{
    //     IKFAST,
    //     KDL,
    //     TRAC_IK,
    // };

    // Type aliases
    using SE3 = Sophus::SE3d;
    using SO3 = Sophus::SO3d;
    using Twist = Sophus::Vector6d;

    // KinematicsModule: Forward kinematics and inverse kinematics calculations
    class KinematicsModule : public kinematics::KinematicsBase
    {
    public:
        KinematicsModule();
        
        bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                const std::string& group_name, const std::string& base_frame,
                const std::vector<std::string>& tip_frames, double search_discretization) override;
        
        bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::msg::Pose>& poses) const override;

        bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;
        
        bool getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                const std::vector<double>& ik_seed_state, std::vector<std::vector<double> >& solutions,
                kinematics::KinematicsResult& result, const kinematics::KinematicsQueryOptions& options) const override;
        
        bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

        bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                const std::vector<double>& consistency_limits, std::vector<double>& solution,
                moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;
        
        bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                std::vector<double>& solution, const IKCallbackFn& solution_callback,
                moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;
        
        bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                const std::vector<double>& consistency_limits, std::vector<double>& solution,
                const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;
        
        const std::vector<std::string>& getJointNames() const override;

        const std::vector<std::string>& getLinkNames() const override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::string group_name_;
        std::string base_frame_;
        std::vector<std::string> tip_frames_;
        double search_discretization_;
        const moveit::core::JointModelGroup* joint_model_group_;
        std::vector<std::string> joint_names_;
        // std::vector<double> joint_min_limits_;
        // std::vector<double> joint_max_limits_;
        KDL::Chain kdl_chain_;
    };
}
