#include "multibot_robot/mpc_optimizer.hpp"

using namespace Control;
using namespace MPC;

void MPC_Optimizer::initialize(const WayPointContainer *_way_points)
{
    optimizer_ = initOptimizer();
    cost_ = std::numeric_limits<double>::infinity();

    prefer_rotdir_ = RotType::none;
    way_points_ = _way_points;

    vel_start_.first = true;
    vel_start_.second.linear.x = 0.0;
    vel_start_.second.angular.z = 0.0;

    vel_goal_.first = true;
    vel_goal_.second.linear.x = 0.0;
    vel_goal_.second.angular.z = 0.0;

    initialized_ = true;
}

std::shared_ptr<g2o::SparseOptimizer> MPC_Optimizer::initOptimizer()
{
    static std::once_flag flag;
    std::call_once(flag, [this]()
                   { this->registerG2OTypes(); });

    std::unique_ptr<MPCLinearSolver> linearSolver = std::make_unique<MPCLinearSolver>();
    linearSolver->setBlockOrdering(true);

    std::unique_ptr<MPCBlockSolver> blockSolver = std::make_unique<MPCBlockSolver>(std::move(linearSolver));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();

    optimizer->setAlgorithm(solver);

    optimizer->initMultiThreading();

    return optimizer;
}

void MPC_Optimizer::registerG2OTypes()
{
    g2o::Factory *factory = g2o::Factory::instance();

    factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
    factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

    factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
    factory->registerType("EDGE_POSITION_OPTIMAL", new g2o::HyperGraphElementCreator<EdgePositionOptimal>);
    factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
    factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
    factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
    factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
    factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
    factory->registerType("EDGE_WAYPOINT", new g2o::HyperGraphElementCreator<EdgeWaypoint>);
    factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);

    return;
}

bool MPC_Optimizer::getVelocityCommand(geometry_msgs::msg::Twist &_cmd_vel, int _look_ahead_poses) const
{
    
}

void MPC_Optimizer::clearGraph()
{
    if (optimizer_)
    {
        optimizer_->vertices().clear();
        optimizer_->clear();
    }
}

MPC_Optimizer::MPC_Optimizer(const WayPointContainer *_way_points = NULL)
{
    initialize(_way_points);
}

MPC_Optimizer::~MPC_Optimizer()
{
    clearGraph();
}