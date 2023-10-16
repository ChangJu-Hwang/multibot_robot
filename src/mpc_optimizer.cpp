#include "multibot_robot/mpc_optimizer.hpp"

using namespace Control;
using namespace MPC;

bool MPC_Optimizer::plan(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &_initial_plan,
                         const geometry_msgs::msg::Twist *_start_vel = NULL,
                         bool _free_goal_vel = false)
{
    if (not(mpc_->isInit()))
        mpc_->initTrajectoryToGoal(_initial_plan, max_vel_lin_, max_vel_ang_, true,
                                   min_samples_, false);
    else
    {
        PoseSE2 start(_initial_plan.front());
        PoseSE2 goal(_initial_plan.back());
        if (mpc_->sizePoses() > 0 and
            (goal.position() - mpc_->BackPose().position()).norm() < 1.0 and
            std::fabs(g2o::normalize_theta(goal.theta() - mpc_->BackPose().theta())) < 0.5 * M_PI)
        {
            mpc_->updateAndPruneTEB(start, goal, min_samples_);
        }
        else
        {
            mpc_->clearModelPredictiveControl();
            mpc_->initTrajectoryToGoal(_initial_plan, max_vel_lin_, max_vel_ang_, true,
                                       min_samples_, false);
        }
    }

    if (_start_vel)
        setVelocityStart(*_start_vel);
    if (_free_goal_vel)
        setVelocityGoalFree();
    else
        vel_goal_.first = true;

    return optimizeMPC(5, 4);
}

bool MPC_Optimizer::getVelocityCommand(geometry_msgs::msg::Twist &_cmd_vel, int _look_ahead_poses) const
{
    if (mpc_->sizePoses() < 2)
    {
        _cmd_vel.linear.x = 0.0;
        _cmd_vel.angular.z = 0.0;
        return false;
    }

    _look_ahead_poses = std::max(1, std::min(_look_ahead_poses, mpc_->sizePoses() - 1));

    double dt = 0.0;
    for (int index = 0; index < _look_ahead_poses; ++index)
    {
        dt += mpc_->TimeDiff(index);

        if (dt >= dt_ref_ * _look_ahead_poses)
        {
            _look_ahead_poses = index + 1;
            break;
        }
    }

    if (dt <= 0)
    {
        _cmd_vel.linear.x = 0.0;
        _cmd_vel.angular.z = 0.0;
        return false;
    }

    extractVelocity(
        mpc_->Pose(0), mpc_->Pose(_look_ahead_poses),
        dt, _cmd_vel);

    return true;
}

void MPC_Optimizer::initialize(
    const double _max_vel_lin, const double _max_vel_ang,
    const double _max_acc_lin, const double _max_acc_ang,
    const double _dt_ref, const double _dt_hysteresis,
    const double _min_samples, const double _max_samples,
    const WayPointContainer *_way_points)
{
    optimizer_ = initOptimizer();
    cost_ = std::numeric_limits<double>::infinity();

    min_samples_ = _min_samples;

    prefer_rotdir_ = RotType::none;
    way_points_ = _way_points;

    max_vel_lin_ = _max_vel_lin;
    max_vel_ang_ = _max_vel_ang;

    max_acc_lin_ = _max_acc_lin;
    max_acc_ang_ = _max_acc_ang;

    dt_ref_ = _dt_ref;

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

bool MPC_Optimizer::buildGraph()
{
    if (not(optimizer_->edges().empty() and optimizer_->vertices().empty()))
        return false;

    AddMPCVertices();

    AddEdgesWayPoints();

    AddEdgesVelocity();

    AddEdgesAcceleration();

    AddEdgesTimeOptimal();

    AddEdgesPositionOptimal();

    AddEdgesKinematics();

    AddEdgesPreferRotDir();

    return true;
}

void MPC_Optimizer::clearGraph()
{
    if (optimizer_)
    {
        optimizer_->vertices().clear();
        optimizer_->clear();
    }
}

bool MPC_Optimizer::optimizeMPC(int _iterations_innerloop, int _iterations_outerloop)
{
    bool success = false;
    optimized_ = false;

    bool fast_mode = false;

    for (int i = 0; i < _iterations_outerloop; ++i)
    {
        // Todo: Parameterize
        mpc_->autoResize(dt_ref_, dt_hysteresis_, min_samples_, max_samples_, fast_mode);

        success = buildGraph();
        if (not(success))
        {
            clearGraph();
            return false;
        }
        success = optimizeGraph(_iterations_innerloop, false);
        if (not(success))
        {
            clearGraph();
            return false;
        }
        optimized_ = true;

        clearGraph();
    }

    return true;
}

bool MPC_Optimizer::optimizeGraph(int _no_iterations, bool _clear_after)
{
    if (max_vel_lin_ < 0.01)
    {
        if (_clear_after)
            clearGraph();

        return false;
    }

    if (not(mpc_->isInit()) or mpc_->sizePoses() < min_samples_)
    {
        if (_clear_after)
            clearGraph();

        return false;
    }

    optimizer_->setVerbose(false);
    optimizer_->initializeOptimization();

    int iter = optimizer_->optimize(_no_iterations);

    if (not(iter))
        return false;

    if (_clear_after)
        clearGraph();

    return true;
}

void MPC_Optimizer::setVelocityStart(const geometry_msgs::msg::Twist &_vel_start)
{
    vel_start_.first = true;
    vel_start_.second = _vel_start;
}

void MPC_Optimizer::setVelocityGoal(const geometry_msgs::msg::Twist &_vel_goal)
{
    vel_goal_.first = true;
    vel_goal_.second = _vel_goal;
}

void MPC_Optimizer::extractVelocity(
    const PoseSE2 &_pose1, const PoseSE2 &_pose2,
    double _dt, geometry_msgs::msg::Twist &_cmd_vel) const
{
    if (_dt == 0)
    {
        _cmd_vel.linear.x = 0.0;
        _cmd_vel.angular.z = 0.0;
        return;
    }

    Eigen::Vector2d deltaS = _pose2.position() - _pose1.position();
    Eigen::Vector2d confldir(cos(_pose1.theta()), sin(_pose1.theta()));
    double dir = deltaS.dot(confldir);

    const double orientDiff = g2o::normalize_theta(_pose2.theta() - _pose1.theta());

    _cmd_vel.linear.x = (double)g2o::sign(dir) * deltaS.norm() / _dt;
    _cmd_vel.angular.z = orientDiff / _dt;
}

void MPC_Optimizer::AddMPCVertices()
{
    unsigned int id_counter = 0;

    for (int i = 0; i < mpc_->sizePoses(); ++i)
    {
        mpc_->PoseVertex(i)->setId(id_counter);
        optimizer_->addVertex(mpc_->PoseVertex(i));

        if (mpc_->sizeTimeDiffs() != 0 and
            i < mpc_->sizeTimeDiffs())
        {
            mpc_->TimeDiffVertex(i)->setId(id_counter++);
            optimizer_->addVertex(mpc_->TimeDiffVertex(i));
        }
    }
}

void MPC_Optimizer::AddEdgesAcceleration()
{
    int sizePoses = mpc_->sizePoses();

    Eigen::Matrix<double, 2, 2> information_acceleration;
    information_acceleration.fill(0.0);
    // Todo: Parameterize weight
    information_acceleration(0, 0) = 1.0;
    information_acceleration(1, 1) = 1.0;

    if (vel_start_.first)
    {
        EdgeAccelerationStart *acceleration_edge = new EdgeAccelerationStart;
        acceleration_edge->setVertex(0, mpc_->PoseVertex(0));
        acceleration_edge->setVertex(1, mpc_->PoseVertex(1));
        acceleration_edge->setVertex(2, mpc_->TimeDiffVertex(0));
        acceleration_edge->setInitialVelocity(vel_start_.second);
        acceleration_edge->setMaxAcceleration(max_acc_lin_, max_acc_ang_);
        acceleration_edge->setInformation(information_acceleration);

        optimizer_->addEdge(acceleration_edge);
    }

    for (int i = 0; i < sizePoses - 2; ++i)
    {
        EdgeAcceleration *acceleration_edge = new EdgeAcceleration;
        acceleration_edge->setVertex(0, mpc_->PoseVertex(i));
        acceleration_edge->setVertex(1, mpc_->PoseVertex(i + 1));
        acceleration_edge->setVertex(2, mpc_->PoseVertex(i + 2));
        acceleration_edge->setVertex(3, mpc_->TimeDiffVertex(i));
        acceleration_edge->setVertex(4, mpc_->TimeDiffVertex(i + 1));
        acceleration_edge->setInformation(information_acceleration);
        acceleration_edge->setMaxAcceleration(max_acc_lin_, max_acc_ang_);

        optimizer_->addEdge(acceleration_edge);
    }

    if (vel_goal_.first)
    {
        EdgeAccelerationGoal *acceleration_edge = new EdgeAccelerationGoal;
        acceleration_edge->setVertex(0, mpc_->PoseVertex(sizePoses - 2));
        acceleration_edge->setVertex(1, mpc_->PoseVertex(sizePoses - 1));
        acceleration_edge->setVertex(2, mpc_->TimeDiffVertex(mpc_->sizeTimeDiffs() - 1));
        acceleration_edge->setGoalVelocity(vel_goal_.second);
        acceleration_edge->setInformation(information_acceleration);
        acceleration_edge->setMaxAcceleration(max_acc_lin_, max_acc_ang_);

        optimizer_->addEdge(acceleration_edge);
    }
}

void MPC_Optimizer::AddEdgesKinematics()
{
    Eigen::Matrix<double, 2, 2> information_kinematics;
    // Todo: Parameterize weight
    information_kinematics(0, 0) = 1000;
    information_kinematics(0, 1) = 1;

    for (int i = 0; i < mpc_->sizePoses() - 1; ++i)
    {
        EdgeKinematicsDiffDrive *kinematics_edge = new EdgeKinematicsDiffDrive;
        kinematics_edge->setVertex(0, mpc_->PoseVertex(i));
        kinematics_edge->setVertex(1, mpc_->PoseVertex(i + 1));
        kinematics_edge->setInformation(information_kinematics);

        optimizer_->addEdge(kinematics_edge);
    }
}

void MPC_Optimizer::AddEdgesPositionOptimal()
{
    // Todo: Parameterize weight
    // ! In TEB, default weight_shortest_path is zero
    Eigen::Matrix<double, 1, 1> information_position_optimal;
    information_position_optimal.fill(1.0);

    for (int i = 0; i < mpc_->sizePoses() - 1; ++i)
    {
        EdgePositionOptimal *position_optimal_edge = new EdgePositionOptimal;
        position_optimal_edge->setVertex(0, mpc_->PoseVertex(i));
        position_optimal_edge->setVertex(1, mpc_->PoseVertex(i + 1));
        position_optimal_edge->setInformation(information_position_optimal);

        optimizer_->addEdge(position_optimal_edge);
    }
}

void MPC_Optimizer::AddEdgesPreferRotDir()
{
    if (prefer_rotdir_ == RotType::none)
        return;

    if (not(prefer_rotdir_ == RotType::right or prefer_rotdir_ == RotType::left))
        return;

    Eigen::Matrix<double, 1, 1> information_rotdir;
    // Todo: Parameterize weight
    information_rotdir.fill(50.0);

    for (int i = 0; i < mpc_->sizePoses() - 1 and i < 3; ++i)
    {
        EdgePreferRotDir *rotdir_edge = new EdgePreferRotDir;
        rotdir_edge->setVertex(0, mpc_->PoseVertex(i));
        rotdir_edge->setVertex(1, mpc_->PoseVertex(i + 1));
        rotdir_edge->setInformation(information_rotdir);

        if (prefer_rotdir_ == RotType::left)
            rotdir_edge->preferLeft();
        else if (prefer_rotdir_ == RotType::right)
            rotdir_edge->preferRight();

        optimizer_->addEdge(rotdir_edge);
    }
}

void MPC_Optimizer::AddEdgesTimeOptimal()
{
    Eigen::Matrix<double, 1, 1> information_time_optimal;
    information_time_optimal.fill(1.0);

    for (int i = 0; i < mpc_->sizeTimeDiffs(); ++i)
    {
        EdgeTimeOptimal *time_optimal_edge = new EdgeTimeOptimal;
        time_optimal_edge->setVertex(0, mpc_->TimeDiffVertex(i));
        time_optimal_edge->setInformation(information_time_optimal);

        optimizer_->addEdge(time_optimal_edge);
    }
}

void MPC_Optimizer::AddEdgesVelocity()
{
    int sizePoses = mpc_->sizePoses();
    Eigen::Matrix<double, 2, 2> information_velocity;
    information_velocity(0, 0) = 2.0;
    information_velocity(1, 1) = 1.0;

    for (int i = 0; i < sizePoses - 1; ++i)
    {
        EdgeVelocity *velocity_edge = new EdgeVelocity;
        velocity_edge->setVertex(0, mpc_->PoseVertex(i));
        velocity_edge->setVertex(1, mpc_->PoseVertex(i + 1));
        velocity_edge->setVertex(2, mpc_->TimeDiffVertex(i));
        ;
        velocity_edge->setInformation(information_velocity);
        velocity_edge->setMaxVelocity(max_vel_lin_, max_vel_ang_);

        optimizer_->addEdge(velocity_edge);
    }
}

void MPC_Optimizer::AddEdgesWayPoints()
{
    if (way_points_ == NULL or way_points_->empty())
        return;

    int start_pose_idx = 0;

    int siezPoses = mpc_->sizePoses();
    if (siezPoses < 3)
        return;

    for (WayPointContainer::const_iterator wp_it = way_points_->begin(); wp_it != way_points_->end(); ++wp_it)
    {
        int index = mpc_->findClosestTrajectoryPose(*wp_it, NULL, start_pose_idx);

        if (index > siezPoses - 2)
            index = siezPoses - 2;

        if (index < 1)
            continue;

        Eigen::Matrix<double, 1, 1> information_way_points;
        information_way_points.fill(1.0);

        EdgeWaypoint *edge_waypoint = new EdgeWaypoint;
        edge_waypoint->setVertex(0, mpc_->PoseVertex(index));
        edge_waypoint->setInformation(information_way_points);
        edge_waypoint->setWaypoint(&(*wp_it));

        optimizer_->addEdge(edge_waypoint);
    }
}

MPC_Optimizer::MPC_Optimizer(
    const double _max_vel_lin, const double _max_vel_ang,
    const double _max_acc_lin, const double _max_acc_ang,
    const double _dt_ref, const double _dt_hysteresis,
    const double _min_samples, const double _max_samples,
    const WayPointContainer *_way_points = NULL)

{
    initialize(
        _max_vel_lin, _max_vel_ang,
        _max_acc_lin, _max_acc_ang,
        _dt_ref, _dt_hysteresis,
        _min_samples, _max_samples,
        _way_points);
}

MPC_Optimizer::~MPC_Optimizer()
{
    clearGraph();
}