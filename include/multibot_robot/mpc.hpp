#pragma once

#include <cassert>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "multibot_robot/distance_calculations.hpp"
#include "multibot_robot/g2o_utils/vertex_pose.hpp"
#include "multibot_robot/g2o_utils/vertex_timediff.hpp"

namespace Control
{
    namespace MPC
    {
        typedef std::vector<VertexPose *> PoseSequence;
        typedef std::vector<VertexTimeDiff *> TimeDiffSequence;

        class ModelPredictiveControl
        {
        public:
            PoseSequence &poses() { return pose_vec_; }
            const PoseSequence &poses() const { return pose_vec_; }

            TimeDiffSequence &timediffs() { return timediff_vec_; }
            const TimeDiffSequence &timediffs() const { return timediff_vec_; }

            double &TimeDiff(int _index)
            {
                assert(_index < sizeTimeDiffs());
                return timediff_vec_.at(_index)->dt();
            }
            const double &TimeDiff(int _index) const
            {
                assert(_index < sizeTimeDiffs());
                return timediff_vec_.at(_index)->dt();
            }
            double &BackTimeDiff() { return timediff_vec_.back()->dt(); }
            const double &BackTimeDiff() const { return timediff_vec_.back()->dt(); }

            VertexTimeDiff *TimeDiffVertex(int _index)
            {
                assert(_index < sizeTimeDiffs());
                return timediff_vec_.at(_index);
            }

            PoseSE2 &Pose(int _index)
            {
                assert(_index < sizePoses());
                return pose_vec_.at(_index)->pose();
            }
            const PoseSE2 &Pose(int _index) const
            {
                assert(_index < sizePoses());
                return pose_vec_.at(_index)->pose();
            }

            PoseSE2 &BackPose() { return pose_vec_.back()->pose(); }
            const PoseSE2 &BackPose() const { return pose_vec_.back()->pose(); }

            VertexPose *PoseVertex(int _index)
            {
                assert(_index < sizePoses());
                return pose_vec_.at(_index);
            }

        public:
            void addPose(const PoseSE2 &_pose, bool _fixed = false);
            void addPose(
                const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta, bool _fixed = false);
            void addPose(double _x, double _y, double _theta, bool _fixed = false);

            void addTimeDiff(double _dt, bool _fixed = false);

            void addPoseAndTimeDiff(const PoseSE2 &_pose, double _dt);
            void addPoseAndTimeDiff(
                const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta, double _dt);
            void addPoseAndTimeDiff(double _x, double _y, double _theta, double _dt);

            void insertPose(int _index, const PoseSE2 &_pose);
            void insertPose(int _index, const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta);
            void insertPose(int _index, double _x, double _y, double _theta);

            void insertTimeDiff(int _index, double _dt);

            void deletePose(int _index);
            void deletePoses(int _index, int _number);

            void deleteTimeDiff(int _index);
            void deleteTimeDiffs(int _index, int _number);

        public:
            void setPoseVertexFixed(int _index, bool _status);
            void setTimeDiffVertexFixed(int _index, bool _status);

        public:
            double estimateDeltaT(
                const PoseSE2 &_start, const PoseSE2 &_end,
                double _max_vel_x, double _max_vel_theta);

        public:
            bool initTrajectoryToGoal(
                const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &_plan,
                double _max_vel_x, double max_vel_theta,
                bool _estimate_orient = false, int _min_samples = 3, bool _guess_backwards_motion = false);

            void updateAndPruneTEB(
                const PoseSE2 &_new_start, const PoseSE2 &_new_goal,
                int _min_samples = 3);

            void autoResize(
                double _dt_ref, double _dt_hysteresis,
                int _min_samples = 3, int _max_samples = 1000,
                bool _fast_mode = false);

            int findClosestTrajectoryPose(
                const Eigen::Ref<const Eigen::Vector2d> &_ref_point,
                double *_distance = NULL, int begin_idx = 0) const;

            int findClosestTrajectoryPose(
                const Eigen::Ref<const Eigen::Vector2d> &_ref_line_start,
                const Eigen::Ref<const Eigen::Vector2d> &_ref_line_end,
                double *_distance = NULL) const;

            int findClosestTrajectoryPose(
                const Point2dContainer &_vertices, double *_distance = NULL) const;

        public:
            double getSumOfAllTimeDiffs() const;
            double getSumOfTimeDiffsUpToIdx(int _index) const;
            double getAccumulatedDistance() const;

        public:
            int sizePoses() const { return (int)pose_vec_.size(); }
            int sizeTimeDiffs() const { return (int)timediff_vec_.size(); }
            bool isInit() const { return not(pose_vec_.empty() or timediff_vec_.empty()); }

        public:
            void clearModelPredictiveControl();

        private:
            PoseSequence pose_vec_;
            TimeDiffSequence timediff_vec_;

        public:
            ModelPredictiveControl() {}
            ~ModelPredictiveControl() { clearModelPredictiveControl(); }
        }; // class ModelPredictiveControl
    }      // namespace MPC
} // namespace Control