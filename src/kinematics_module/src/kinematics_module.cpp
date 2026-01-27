#include "kinematics_module.h"

namespace KinematicsModule
{
    bool KinematicsModule::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
            const std::string& group_name, const std::string& base_frame,
            const std::vector<std::string>& tip_frames, double search_discretization)
    {
        node_ = node;
        group_name_ = group_name;
        base_frame_ = base_frame;
        tip_frames_ = tip_frames;
        search_discretization_ = search_discretization;
        joint_model_group_ = robot_model.getJointModelGroup(group_name_);
        if (!joint_model_group_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint model group %s not found", group_name.c_str());
            return false;
        }
        const urdf::ModelInterfaceSharedPtr urdf_model = robot_model.getURDF();
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl tree");
            return false;
        }

        if (!kdl_tree.getChain(base_frame, tip_frames[0], kdl_chain_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl chain from %s to %s", 
            base_frame.c_str(), tip_frames[0].c_str());
            return false;
        }

        joint_names_ = joint_model_group_->getActiveJointModelNames();
        return true;
    }

}