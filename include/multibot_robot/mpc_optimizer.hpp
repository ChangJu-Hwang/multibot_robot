#pragma once

#include <vector>
#include <utility>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "multibot_robot/mpc.hpp"

#include "multibot_robot/g2o_utils/edge_acceleration.hpp"
#include "multibot_robot/g2o_utils/edge_kinematics.hpp"
#include "multibot_robot/g2o_utils/edge_position_optimal.hpp"
#include "multibot_robot/g2o_utils/edge_prefer_rotdir.hpp"
#include "multibot_robot/g2o_utils/edge_time_optimal.hpp"
#include "multibot_robot/g2o_utils/edge_velocity.hpp"
#include "multibot_robot/g2o_utils/edge_waypoint.hpp"
#include "multibot_robot/g2o_utils/g2o_utils.hpp"

namespace Control
{
    namespace MPC
    {
        class MPC_Optimizer
        {
        private:
            typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> WayPointContainer;

            typedef g2o::BlockSolverX MPCBlockSolver;
            typedef g2o::LinearSolverCSparse<MPCBlockSolver::PoseMatrixType> MPCLinearSolver;
            typedef g2o::BlockSolverX MPCBlockSolver;

        public:
            bool plan(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &_initial_plan,
                      const geometry_msgs::msg::Twist *_start_vel = NULL,
                      bool _free_goal_vel = false);

            bool getVelocityCommand(geometry_msgs::msg::Twist &_cmd_vel, int _look_ahead_poses) const;

            void clearPlanner()
            {
                clearGraph();
                mpc_->clearModelPredictiveControl();
            }

            void setPreferredTurningDir(RotType _dir) { prefer_rotdir_ = _dir; }

        private:
            void initialize(
                const double _max_vel_lin, const double _max_vel_ang,
                const double _max_acc_lin, const double _max_acc_ang,
                const double _dt_ref, const double _dt_hysteresis,
                const double _min_samples, const double _max_samples,
                const WayPointContainer *_waypoints);
            std::shared_ptr<g2o::SparseOptimizer> initOptimizer();
            void registerG2OTypes();

            bool buildGraph();
            void clearGraph();

            bool optimizeMPC(int _iterations_innerloop, int _iterations_outerloop);
            bool optimizeGraph(int _no_iterations, bool _clear_after);

            void extractVelocity(
                const PoseSE2 &_pose1, const PoseSE2 &_pose2,
                double _dt, geometry_msgs::msg::Twist &_cmd_vel) const;

        private:
            void setVelocityStart(const geometry_msgs::msg::Twist &_vel_start);
            void setVelocityGoal(const geometry_msgs::msg::Twist &_vel_goal);
            void setVelocityGoalFree() { vel_goal_.first = false; }

        private:
            void AddMPCVertices();

            void AddEdgesAcceleration();

            void AddEdgesKinematics();

            void AddEdgesPositionOptimal();

            void AddEdgesPreferRotDir();

            void AddEdgesTimeOptimal();

            void AddEdgesVelocity();

            void AddEdgesWayPoints();

        private:
            std::shared_ptr<g2o::SparseOptimizer> optimizer_;
            std::shared_ptr<ModelPredictiveControl> mpc_;
            double cost_;

            int min_samples_;
            int max_samples_;

            RotType prefer_rotdir_;
            const WayPointContainer *way_points_;

            std::pair<bool, geometry_msgs::msg::Twist> vel_start_;
            std::pair<bool, geometry_msgs::msg::Twist> vel_goal_;

            double max_vel_lin_, max_vel_ang_;
            double max_acc_lin_, max_acc_ang_;

            double dt_ref_;
            double dt_hysteresis_;

            bool initialized_;
            bool optimized_;

        public:
            MPC_Optimizer()
                : prefer_rotdir_(RotType::none), way_points_(NULL), initialized_(false), optimized_(false) {}
            MPC_Optimizer(
                const double _max_vel_lin, const double _max_vel_ang,
                const double _max_acc_lin, const double _max_acc_ang,
                const double _dt_ref, const double _dt_hysteresis,
                const double _min_samples, const double _max_samples,
                const WayPointContainer *_way_points = NULL);
            ~MPC_Optimizer();
        }; // class Optimizer
    }      // namespace MPC
} // namespace Control