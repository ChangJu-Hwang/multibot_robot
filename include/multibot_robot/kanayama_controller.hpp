#pragma once

#include "multibot_robot/trajectory_follower.hpp"

using namespace Instance;

namespace Control
{
    class Kanayama_Controller : public Trajectory_Follower
    {
    public:
        bool control(
            const Position::Pose &_curPose) override;

    private:
        void init_varibales(const AgentInstance::Agent &_robot);
        void init_parameters();

    private:
        // Kanayama Controller Parameter
        double Kx_, Ky_, Ktheta_;

    public:
        Kanayama_Controller(
            std::shared_ptr<rclcpp::Node> _nh,
            const AgentInstance::Agent &_robot);
        ~Kanayama_Controller();
    }; // class Kanayama_Controller
} // namespace Control