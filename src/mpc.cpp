#include "multibot_robot/mpc.hpp"

using namespace Control;
using namespace MPC;

void ModelPredictiveControl::addPose(const PoseSE2 &_pose, bool _fixed)
{
    VertexPose *pose_vertex = new VertexPose(_pose, _fixed);
    pose_vec_.push_back(pose_vertex);
    return;
}

void ModelPredictiveControl::addPose(
    const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta, bool _fixed)
{
    VertexPose *pose_vertex = new VertexPose(_position, _theta, _fixed);
    pose_vec_.push_back(pose_vertex);
    return;
}

void ModelPredictiveControl::addPose(double _x, double _y, double _theta, bool _fixed)
{
    VertexPose *pose_vertex = new VertexPose(_x, _y, _theta, _fixed);
    pose_vec_.push_back(pose_vertex);
    return;
}

void ModelPredictiveControl::addTimeDiff(double _dt, bool _fixed)
{
    assert(_dt > 0.0);
    VertexTimeDiff *timediff_vertex = new VertexTimeDiff(_dt, _fixed);
    timediff_vec_.push_back(timediff_vertex);
    return;
}

void ModelPredictiveControl::addPoseAndTimeDiff(const PoseSE2 &_pose, double _dt)
{
    if (sizePoses() != sizeTimeDiffs())
    {
        addPose(_pose, false);
        addTimeDiff(_dt, false);
    }
    return;
}

void ModelPredictiveControl::addPoseAndTimeDiff(
    const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta, double _dt)
{
    if (sizePoses() != sizeTimeDiffs())
    {
        addPose(_position, _theta, false);
        addTimeDiff(_dt, false);
    }
    return;
}

void ModelPredictiveControl::addPoseAndTimeDiff(double _x, double _y, double _theta, double _dt)
{
    if (sizePoses() != sizeTimeDiffs())
    {
        addPose(_x, _y, _theta, _dt);
        addTimeDiff(_dt, false);
    }
    return;
}

void ModelPredictiveControl::insertPose(int _index, const PoseSE2 &_pose)
{
    VertexPose *pose_vertex = new VertexPose(_pose);
    pose_vec_.insert(pose_vec_.begin() + _index, pose_vertex);
    return;
}

void ModelPredictiveControl::insertPose(int _index, const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta)
{
    VertexPose *pose_vertex = new VertexPose(_position, _theta);
    pose_vec_.insert(pose_vec_.begin() + _index, pose_vertex);
    return;
}

void ModelPredictiveControl::insertPose(int _index, double _x, double _y, double _theta)
{
    VertexPose *pose_vertex = new VertexPose(_x, _y, _theta);
    pose_vec_.insert(pose_vec_.begin() + _index, pose_vertex);
    return;
}

void ModelPredictiveControl::insertTimeDiff(int _index, double _dt)
{
    VertexTimeDiff *timediff_vertex = new VertexTimeDiff(_dt);
    timediff_vec_.insert(timediff_vec_.begin() + _index, timediff_vertex);
    return;
}

void ModelPredictiveControl::deletePose(int _index)
{
    assert(_index < sizePoses());
    delete pose_vec_.at(_index);
    pose_vec_.erase(pose_vec_.begin() + _index);
    return;
}

void ModelPredictiveControl::deletePoses(int _index, int _number)
{
    assert(_index + _number <= (int)sizePoses());
    for (int i = _index; i < _index + _number; ++i)
        delete pose_vec_.at(i);
    pose_vec_.erase(pose_vec_.begin() + _index, pose_vec_.begin() + _index + _number);
}

void ModelPredictiveControl::deleteTimeDiff(int _index)
{
    assert(_index < sizeTimeDiffs());
    delete timediff_vec_.at(_index);
    timediff_vec_.erase(timediff_vec_.begin() + _index);
}

void ModelPredictiveControl::deleteTimeDiffs(int _index, int _number)
{
    assert(_index + _number <= (int)sizeTimeDiffs());
    for (int i = _index; i < _index + _number; ++i)
        delete timediff_vec_.at(i);
    timediff_vec_.erase(timediff_vec_.begin() + _index, timediff_vec_.begin() + _index + _number);
}

void ModelPredictiveControl::setPoseVertexFixed(int _index, bool _status)
{
    assert(_index < sizePoses());
    pose_vec_.at(_index)->setFixed(_status);
}

void ModelPredictiveControl::setTimeDiffVertexFixed(int _index, bool _status)
{
    assert(_index < sizeTimeDiffs());
    timediff_vec_.at(_index)->setFixed(_status);
}

double ModelPredictiveControl::estimateDeltaT(
    const PoseSE2 &_start, const PoseSE2 &_end,
    double _max_vel_x, double _max_vel_theta)
{
    double dt_constant_motion = 0.1;

    if (_max_vel_x > 0)
    {
        double trans_dist = (_end.position() - _start.position()).norm();
        dt_constant_motion = trans_dist / _max_vel_x;
    }
    if (_max_vel_theta > 0)
    {
        double ros_dist = std::fabs(g2o::normalize_theta(_end.theta() - _start.theta()));
        dt_constant_motion = std::max(dt_constant_motion, ros_dist / _max_vel_theta);
    }

    return dt_constant_motion;
}

bool ModelPredictiveControl::initTrajectoryToGoal(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &_plan,
    double _max_vel_x, double _max_vel_theta,
    bool _estimate_orient, int _min_samples, bool _guess_backwards_motion)
{
    if (isInit())
        return false;

    PoseSE2 start(_plan.front());
    PoseSE2 goal(_plan.back());

    addPose(start);
    setPoseVertexFixed(0, true);

    bool backwards = false;
    if (_guess_backwards_motion and
        (goal.position() - start.position()).dot(start.orientationUnitVec()) < 0)
        backwards = true;

    for (int i = 1; i < (int)_plan.size() - 1; ++i)
    {
        PoseSE2 intermediate_pose(_plan[i]);
        double dt = estimateDeltaT(BackPose(), intermediate_pose, _max_vel_x, _max_vel_theta);
        addPoseAndTimeDiff(intermediate_pose, dt);
    }

    while (sizePoses() < _min_samples - 1)
    {
        PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
        double dt = estimateDeltaT(BackPose(), intermediate_pose, _max_vel_x, _max_vel_theta);
        addPoseAndTimeDiff(intermediate_pose, dt);
    }

    double dt = estimateDeltaT(BackPose(), goal, _max_vel_x, _max_vel_theta);
    addPoseAndTimeDiff(goal, dt);
    setPoseVertexFixed(sizePoses() - 1, true);

    return true;
}

void ModelPredictiveControl::updateAndPruneTEB(
    const PoseSE2 &_new_start, const PoseSE2 &_new_goal,
    int _min_samples)
{
    if (sizePoses() > 0)
    {
        double dist_cache = (_new_start.position() - Pose(0).position()).norm();
        double dist;
        int lookahead = std::min<int>(sizePoses() - _min_samples, 10);

        int nearest_idx = 0;
        for (int i = 1; i <= lookahead; ++i)
        {
            dist = (_new_start.position() - Pose(i).position()).norm();
            if (dist < dist_cache)
            {
                dist_cache = dist;
                nearest_idx = i;
            }
            else
                break;
        }

        if (nearest_idx > 0)
        {
            deletePoses(1, nearest_idx);
            deleteTimeDiffs(1, nearest_idx);
        }

        Pose(0) = _new_start;
    }

    if (sizePoses() > 0)
        BackPose() = _new_goal;
}

void ModelPredictiveControl::autoResize(
    double _dt_ref, double _dt_hysteresis,
    int _min_samples, int _max_samples,
    bool _fast_mode)
{
    assert(sizeTimeDiffs() == 0 or sizeTimeDiffs() + 1 == sizePoses());
    bool modified = true;

    for (int rep = 0; rep < 100 and modified; ++rep)
    {
        modified = false;

        for (int i = 0; i < sizeTimeDiffs(); ++i)
        {
            if (TimeDiff(i) > _dt_ref + _dt_hysteresis and
                sizeTimeDiffs() < _max_samples)
            {
                double newTime = 0.5 * TimeDiff(i);

                TimeDiff(i) = newTime;
                insertPose(i + 1, PoseSE2::average(Pose(i), Pose(i + 1)));
                insertTimeDiff(i + 1, newTime);

                modified = true;
            }
            else if (TimeDiff(i) < _dt_ref - _dt_hysteresis and
                     sizeTimeDiffs() > _min_samples)
            {
                if (i < ((int)sizeTimeDiffs() - 1))
                {
                    TimeDiff(i + 1) += TimeDiff(i);
                    deleteTimeDiff(i);
                    deletePose(i + 1);
                }
                else
                {
                    TimeDiff(i - 1) += TimeDiff(i);
                    deleteTimeDiff(i);
                    deletePose(i);
                }

                modified = true;
            }
        }

        if (_fast_mode)
            break;
    }
}

int ModelPredictiveControl::findClosestTrajectoryPose(
    const Eigen::Ref<const Eigen::Vector2d> &_ref_point,
    double *_distance, int begin_idx) const
{
    int n = sizePoses();
    if (begin_idx < 0 or begin_idx >= n)
        return -1;

    double min_dist_sq = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (int i = begin_idx; i < n; i++)
    {
        double dist_sq = (_ref_point - Pose(i).position()).squaredNorm();
        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            min_idx = i;
        }
    }

    if (_distance)
        *_distance = std::sqrt(min_dist_sq);

    return min_idx;
}

int ModelPredictiveControl::findClosestTrajectoryPose(
    const Eigen::Ref<const Eigen::Vector2d> &_ref_line_start,
    const Eigen::Ref<const Eigen::Vector2d> &_ref_line_end,
    double *_distance = NULL) const
{
    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (int i = 0; i < sizePoses(); i++)
    {
        Eigen::Vector2d point = Pose(i).position();
        double dist = distance_point_to_segment_2d(point, _ref_line_start, _ref_line_end);
        if (dist < min_dist)
        {
            min_dist = dist;
            min_idx = i;
        }
    }

    if (_distance)
        *_distance = min_dist;

    return min_idx;
}

int ModelPredictiveControl::findClosestTrajectoryPose(
    const Point2dContainer &_vertices, double *_distance = NULL) const
{
    if (_vertices.empty())
        return 0;
    else if (_vertices.size() == 1)
        return findClosestTrajectoryPose(_vertices.front());
    else if (_vertices.size() == 2)
        return findClosestTrajectoryPose(_vertices.front(), _vertices.back());

    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (int i = 0; i < sizePoses(); i++)
    {
        Eigen::Vector2d point = Pose(i).position();

        double dist_to_polygon = std::numeric_limits<double>::max();
        for (int j = 0; j < (int)_vertices.size() - 1; ++j)
            dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(point, _vertices[j], _vertices[j + 1]));
        dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(point, _vertices.back(), _vertices.front()));

        if (dist_to_polygon < min_dist)
        {
            min_dist = dist_to_polygon;
            min_idx = i;
        }
    }

    if (_distance)
        *_distance = min_dist;

    return min_idx;
}

double ModelPredictiveControl::getSumOfAllTimeDiffs() const
{
    double time = 0.0;

    for (const auto &dt : timediff_vec_)
        time += dt->dt();

    return time;
}

double ModelPredictiveControl::getSumOfTimeDiffsUpToIdx(int _index) const
{
    assert(_index <= sizeTimeDiffs());

    double time = 0.0;

    for (int i = 0; i < _index; ++i)
        time = timediff_vec_.at(i)->dt();

    return time;
}

double ModelPredictiveControl::getAccumulatedDistance() const
{
    double dist = 0;

    for (int i = 1; i < sizePoses(); ++i)
        dist += (Pose(i).position() - Pose(i - 1).position()).norm();

    return dist;
}

void ModelPredictiveControl::clearModelPredictiveControl()
{
    for (VertexPose *pose : pose_vec_)
        delete pose;
    pose_vec_.clear();

    for (VertexTimeDiff *dt : timediff_vec_)
        delete dt;
    timediff_vec_.clear();
    return;
}