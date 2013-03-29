/// NOTE: Code taken from IRL-1's code base at IntRoLab.
/// Author: Francois Ferland, francois.ferland@usherbrooke.ca
///

#include "turn_toward.hpp"

using namespace autonav;

TurnToward::TurnToward(const ros::NodeHandle& np)
{
    np.param("robot_frame", robot_frame_, std::string("base_link"));
    np.param("turn_k_theta", k_theta_, 0.1);
    np.param("turn_max_vel", max_vel_, 0.3);

}

bool TurnToward::calcVel(
    const geometry_msgs::PoseStamped& pose, 
    double& vel,
    bool eps_test) const
{
    tf::Stamped<tf::Pose> p, p_t;
    tf::poseStampedMsgToTF(pose, p);

    // NOTE: This mostly happens if the pose is too old, just discard the 
    // error.
    try {
        tf_.transformPose(robot_frame_, p, p_t);
    } catch (tf::TransformException e) {
        ROS_DEBUG_THROTTLE(1.0, "TF error: %s", e.what());
        return false;
    }

    const tf::Vector3& pos = p_t.getOrigin();
    return calcVel(pos, vel, eps_test);
}

bool TurnToward::calcVel(
    const geometry_msgs::PoseStamped& pose,
    geometry_msgs::Twist& cmd_vel,
    bool eps_test) const
{
    if (calcVel(pose, cmd_vel.angular.z, eps_test))
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        return true;
    }
    else
        return false;
}

bool TurnToward::calcVel(const tf::Point& pos, double& vel, bool eps_test) const
{
    double e_yaw = atan2(pos.y(), pos.x());

    if (eps_test && fabs(e_yaw) < epsilon_)
        return false;

    vel = k_theta_ * e_yaw;
    double abs_vel = fabs(vel);
    if (abs_vel > max_vel_)
        vel = max_vel_ * abs_vel / fabs(vel);

    return true;
}
