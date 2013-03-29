/// NOTE: Code taken from IRL-1's code base at IntRoLab.
/// Author: Francois Ferland, francois.ferland@usherbrooke.ca
///

#ifndef TURN_TOWARD_HPP
#define TURN_TOWARD_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace autonav
{
    /// \brief A simple behavior tool that generates a command to turn a mobile
    /// robot's base on itself toward a given pose.
    ///
    /// Parameters:
    ///  - robot_frame: Frame id of the mobile base, used to transform given
    ///    poses in the robot's frame.
    ///    Default: "base_link".
    ///  - turn_k_theta: Control coeffient on theta (yaw) angle error.
    ///    Default: 0.1.
    ///  - turn_max_vel: Maximum angular velocity to apply, in rad/s.
    ///    Default: 0.3.
    ///  - turn_epsilon: Minimum angular error (dead angle) before producing a
    ///    command, in rad.
    ///    Default: 0.3.
    ///
    class TurnToward
    {
    public:
        /// \brief Constructor.
        ///
        /// \param np Private node handle to get parameters from.
        TurnToward(const ros::NodeHandle& np = ros::NodeHandle("~"));

        /// \brief Calculate an angular velocity based on the given pose.
        ///
        /// \param td The resulting angular velocity.
        /// \param eps_test If the epsilon test should be run or not.
        /// \return true if a velocity command was produced.
        bool calcVel(const geometry_msgs::PoseStamped& pose, double& vel,
            bool eps_test = true) const;

        /// \brief Same as calcVel, but act on a Twist message instead.
        ///
        /// Will clear the linear velocity from the given message if a valid
        /// angular velocity could be calculated.
        ///
        /// \param eps_test If the epsilon test should be run or not.
        bool calcVel(
            const geometry_msgs::PoseStamped& pose,
            geometry_msgs::Twist& cmd_vel,
            bool eps_test = true) const;

        /// \brief Calculate an angular velocity based on a point already
        /// transformed in the robot's frame.
        ///
        /// Used by the two other versions of the method.
        /// Does not produce a velocity value if the direction is under the
        /// configured epsilon (and eps_test is true).
        /// 
        /// \param eps_test Indicates if we should test for epsilon or not.
        /// \return True if a valid velocity has been calculated.
        bool calcVel(const tf::Point& pt, double& vel, bool eps_test = true) 
            const;

    private:
        tf::TransformListener tf_;

        std::string robot_frame_;
        double k_theta_;
        double max_vel_;
        double epsilon_;

    };

}

#endif

