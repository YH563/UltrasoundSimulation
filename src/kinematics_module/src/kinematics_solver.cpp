#include "kinematics_solver.h"

namespace KinematicsSolver{
    bool KinematicsSolver::initial(const KDL::Chain& kdl_chain){
        M_list_.clear();
        screws_.clear();
        joint_limits_.clear();

        int num_segments = kdl_chain.getNrOfSegments();
        if(num_segments == 0){
            return false;
        }
        // TODO: Add joint limits extraction if needed

        // Traverse all connected segments
        for (int i = 0; i < num_segments; i++){
            const KDL::Segment& segment = kdl_chain.getSegment(i);
            const KDL::Joint& joint = segment.getJoint();
            const KDL::Frame& frame = segment.getFrameToTip();

            KDL::Vector axis = joint.JointAxis();
            if (joint.getType() != KDL::Joint::None)
            {
                Twist screw;
                Vector3d q(frame.p.x(), frame.p.y(), frame.p.z());
                Vector3d w(axis.x(), axis.y(), axis.z());
                screw.head<3>() = -w.cross(q);
                screw.tail<3>() = w;
                screws_.push_back(screw);
            }
            else if (joint.getType() == KDL::Joint::TransAxis){
                Vector3d v(axis.x(), axis.y(), axis.z());
                Twist screw;
                screw.head<3>() = v;
                screw.tail<3>() = Vector3d::Zero();
                screws_.push_back(screw);
            }

            KDL::Frame kdl_frame = frame;
            if (joint.getType() != KDL::Joint::Fixed) {
                // If the joint is not fixed, we need to transform the frame to the joint zero position
                KDL::Frame joint_zero = joint.pose(0.0);
                kdl_frame = joint_zero * frame;
            }
            const auto frame2SE3 = [&kdl_frame]() -> SE3 {
                Vector3d translation(kdl_frame.p.x(), kdl_frame.p.y(), kdl_frame.p.z());
                Rotation3d rotation;
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        rotation(r, c) = kdl_frame.M(r, c);
                    }
                }
                return SE3(rotation, translation);
            };
            M_list_.push_back((i == 0 ? SE3() : M_list_[i - 1]) * frame2SE3());
        }

        M_zero_ = M_list_.back();
        return true;
    }

    SE3 KinematicsSolver::ForwardKinematics(const std::vector<double>& joint_angles) const{
        if (joint_angles.size() != screws_.size()) {
            throw std::runtime_error("Joint angles size does not match the number of screws.");
        }
        SE3 T = SE3();
        for (size_t i = 0; i < joint_angles.size(); ++i) {
            T = T * SE3::exp(screws_[i] * joint_angles[i]);
        }
        return T * M_zero_;
    }
}