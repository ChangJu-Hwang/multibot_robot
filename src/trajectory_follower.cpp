#include "multibot_robot/trajectory_follower.hpp"

using namespace Control;

Trajectory_Follower::TrajSegment::TrajSegment(const LocalTraj &_localTraj)
{
    start_ = Position::Pose(_localTraj.start);
    goal_ = Position::Pose(_localTraj.goal);

    departure_time_ = _localTraj.departure_time;
    arrival_time_ = _localTraj.arrival_time;

    Position::Coordinates start_unit_vector;
    start_unit_vector.x_ = cos(start_.component_.theta);
    start_unit_vector.y_ = sin(start_.component_.theta);
    Position::Coordinates goal_unit_vector;
    goal_unit_vector.x_ = cos(goal_.component_.theta);
    goal_unit_vector.y_ = sin(goal_.component_.theta);

    int angleSign = Position::crossProduct(start_unit_vector, goal_unit_vector) > 0 ? 1 : -1;

    distance_ = Position::getDistance(start_, goal_);
    angle_ = angleSign * Position::getAngleDiff(start_, goal_);
}

void Trajectory_Follower::receiveTraj(
    const std::shared_ptr<Traj::Request> _request)
{
    localTrajIdx_ = 0;

    time_ = -1 * _request->start_time;
    traj_.clear();
    for (const auto &localTraj : _request->traj)
    {
        TrajSegment trajSegment(localTraj);
        trajSegment.rotational_duration_ = Motion::TotalMoveTimeComputer(
            trajSegment.angle_, max_ang_vel_, max_ang_acc_);
        trajSegment.translational_duration_ = Motion::TotalMoveTimeComputer(
            trajSegment.distance_, max_lin_vel_, max_lin_acc_);

        traj_.push_back(trajSegment);
    }
}

Position::Pose Trajectory_Follower::PoseComputer(
    const TrajSegment &_trajSegment,
    const double _time)
{
    try
    {
        if (_trajSegment.departure_time_ > _time + 1e-8)
            throw _time;
    }
    catch (double _wrong_time)
    {
        return _trajSegment.start_;
    }

    if (_time > _trajSegment.arrival_time_ + 1e-8)
        return _trajSegment.goal_;

    Position::Pose pose = _trajSegment.start_;
    // Rotate
    if (_trajSegment.rotational_duration_ + 1e-8 > _time - _trajSegment.departure_time_)
    {
        pose.component_.theta = pose.component_.theta +
                                (_trajSegment.angle_ > 0 ? 1 : -1) *
                                    Motion::DisplacementComputer(
                                        _trajSegment.angle_, max_ang_vel_, max_ang_acc_,
                                        _time - _trajSegment.departure_time_);
        
        while (std::fabs(pose.component_.theta) - M_PI > 1e-8 or
               pose.component_.theta + 1e-8 > M_PI)
        {
            int sign = pose.component_.theta > 0 ? 1 : -1;
            pose.component_.theta = pose.component_.theta - sign * 2 * M_PI;
        }
    }
    // Move
    else
    {
        double displacement = Motion::DisplacementComputer(
            _trajSegment.distance_, max_lin_vel_, max_lin_acc_,
            _time - _trajSegment.departure_time_ - _trajSegment.rotational_duration_);

        double theta = _trajSegment.goal_.component_.theta;

        pose.component_.x = pose.component_.x + displacement * cos(theta);
        pose.component_.y = pose.component_.y + displacement * sin(theta);
        pose.component_.theta = theta;
    }

    return pose;
}