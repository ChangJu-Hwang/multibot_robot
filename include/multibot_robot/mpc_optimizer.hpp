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

            bool getVelocityCommand(
                double &_v, double &_omega, int _look_ahead_poses);

            void clearPlanner();

            void setPreferredTurningDir(RotType dir);

            bool isTrajectoryFeasible();

        private:
            void initialize(const WayPointContainer *_waypoints);
            std::shared_ptr<g2o::SparseOptimizer> initOptimizer();
            void registerG2OTypes();

            void clearGraph();
            
            bool optimizeMPC();

            bool getVelocityCommand(geometry_msgs::msg::Twist &_cmd_vel, int _look_ahead_poses) const;

        private:
        
            std::shared_ptr<g2o::SparseOptimizer> optimizer_;
            double cost_;

            RotType prefer_rotdir_;
            const WayPointContainer *way_points_;

            std::pair<bool, geometry_msgs::msg::Twist> vel_start_;
            std::pair<bool, geometry_msgs::msg::Twist> vel_goal_;

            bool initialized_;
            bool optimized_;

        public:
            MPC_Optimizer()
                : prefer_rotdir_(RotType::none), way_points_(NULL), initialized_(false), optimized_(false) {}
            MPC_Optimizer(const WayPointContainer *_way_points = NULL);
            ~MPC_Optimizer();
        }; // class Optimizer
    }      // namespace MPC
} // namespace Control