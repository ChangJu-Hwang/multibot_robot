#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include <g2o/stuff/misc.h>

#include "multibot_robot/g2o_utils/base_mpc_edges.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"
#include "multibot_robot/g2o_utils/vertex_timediff.hpp"
#include "multibot_robot/g2o_utils/penalties.hpp"
#include "multibot_robot/g2o_utils/g2o_utils.hpp"

namespace Control
{
    namespace MPC
    {
        class EdgeAcceleration : public BaseMPCMultiEdge<2, double>
        {
        public:
            EdgeAcceleration()
            {
                _measurement = NULL;
                this->resize(5);
            }

        public:
            void computeError()
            {
                const VertexPose *pose1 = static_cast<const VertexPose *>(_vertices[0]);
                const VertexPose *pose2 = static_cast<const VertexPose *>(_vertices[1]);
                const VertexPose *pose3 = static_cast<const VertexPose *>(_vertices[2]);
                const VertexTimeDiff *dt1 = static_cast<const VertexTimeDiff *>(_vertices[3]);
                const VertexTimeDiff *dt2 = static_cast<const VertexTimeDiff *>(_vertices[4]);

                // Linear acceleration Error
                const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
                const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

                const double dist1 = diff1.norm();
                const double dist2 = diff2.norm();

                double vel1 = dist1 / dt1->dt();
                double vel2 = dist2 / dt2->dt();

                vel1 *= fast_sigmoid(100 * (diff1.x() * cos(pose1->theta()) + diff1.y() * sin(pose1->theta())));
                vel2 *= fast_sigmoid(100 * (diff2.x() * cos(pose2->theta()) + diff2.y() * sin(pose2->theta())));

                const double acc_lin = (vel2 - vel1) * 2 / (dt1->dt() + dt2->dt());

                _error[0] = penaltyBoundToInterval(acc_lin, max_acc_lin_, 0.0);

                // Linear acceleration Error
                const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
                const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

                const double omega1 = angle_diff1 / dt1->dt();
                const double omega2 = angle_diff2 / dt2->dt();

                const double acc_ang = (omega2 - omega1) * 2 / (dt1->dt() + dt2->dt());

                _error[1] = penaltyBoundToInterval(acc_ang, max_acc_ang_, 0.0);
            }

            void setMaxAcceleration(double _max_acc_lin, double _max_acc_ang)
            {
                max_acc_lin_ = _max_acc_lin;
                max_acc_ang_ = _max_acc_ang;
            }

        private:
            double max_acc_lin_;
            double max_acc_ang_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgeAcceleration

        class EdgeAccelerationStart : public BaseMPCMultiEdge<2, const geometry_msgs::msg::Twist *>
        {
        public:
            EdgeAccelerationStart()
            {
                _measurement = NULL;
                this->resize(3);
            }

        public:
            void computeError()
            {
                const VertexPose *pose_start = static_cast<const VertexPose *>(_vertices[0]);
                const VertexPose *pose_post_start = static_cast<const VertexPose *>(_vertices[1]);
                const VertexTimeDiff *dt = static_cast<const VertexTimeDiff *>(_vertices[2]);

                // Linear acceleration Error
                const Eigen::Vector2d diff = pose_post_start->position() - pose_start->position();
                const double dist = diff.norm();

                const double vel1 = _measurement->linear.x;
                double vel2 = dist / dt->dt();
                vel2 *= fast_sigmoid(100 * (diff.x() * cos(pose_start->theta()) + diff.y() * sin(pose_start->theta())));

                const double acc_lin = (vel2 - vel1) / dt->dt();

                _error[0] = penaltyBoundToInterval(acc_lin, max_acc_lin_, 0.0);

                // Angular acceleration Error
                const double angleDiff = g2o::normalize_theta(pose_post_start->theta() - pose_start->theta());

                const double omega1 = _measurement->angular.z;
                const double omega2 = angleDiff / dt->dt();
                const double acc_rot = (omega2 - omega1) / dt->dt();

                _error[1] = penaltyBoundToInterval(acc_rot, max_acc_ang_, 0.0);
            }

            void setInitialVelocity(const geometry_msgs::msg::Twist &_vel_start)
            {
                _measurement = &_vel_start;
            }

            void setMaxAcceleration(double _max_acc_lin, double _max_acc_ang)
            {
                max_acc_lin_ = _max_acc_lin;
                max_acc_ang_ = _max_acc_ang;
            }

        private:
            double max_acc_lin_;
            double max_acc_ang_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgeAccelerationStart

        class EdgeAccelerationGoal : public BaseMPCMultiEdge<2, const geometry_msgs::msg::Twist *>
        {
        public:
            EdgeAccelerationGoal()
            {
                _measurement = NULL;
                this->resize(3);
            }

        public:
            void computeError()
            {
                const VertexPose *pose_pre_goal = static_cast<const VertexPose *>(_vertices[0]);
                const VertexPose *pose_goal = static_cast<const VertexPose *>(_vertices[1]);
                const VertexTimeDiff *dt = static_cast<const VertexTimeDiff *>(_vertices[2]);

                // Linear acceleration Error
                const Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();
                const double dist = diff.norm();

                double vel1 = dist / dt->dt();
                const double vel2 = _measurement->linear.x;

                vel1 *= fast_sigmoid(100 * (diff.x() * cos(pose_pre_goal->theta()) + diff.y() * sin(pose_pre_goal->theta())));

                const double acc_lin = (vel2 - vel1) / dt->dt();

                _error[0] = penaltyBoundToInterval(acc_lin, max_acc_lin_, 0.0);

                // Angular acceleration Error
                const double angle_diff = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta());

                const double omega1 = angle_diff / dt->dt();
                const double omega2 = _measurement->angular.z;

                const double acc_rot = (omega2 - omega1) / dt->dt();

                _error[1] = penaltyBoundToInterval(acc_rot, max_acc_ang_, 0.0);
            }

            void setInitialVelocity(const geometry_msgs::msg::Twist &_vel_goal)
            {
                _measurement = &_vel_goal;
            }

            void setMaxAcceleration(double _max_acc_lin, double _max_acc_ang)
            {
                max_acc_lin_ = _max_acc_lin;
                max_acc_ang_ = _max_acc_ang;
            }

        private:
            double max_acc_lin_;
            double max_acc_ang_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        }; // class EdgeAccelerationGoal
    }      // namespace MPC
} // namespace Control