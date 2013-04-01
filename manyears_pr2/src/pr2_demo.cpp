/// \file pr2_demo.cpp Quick demo app for ManyEars on the PR2.
///

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include "turn_toward.hpp"

namespace manyears_pr2
{
    /// \brief Main class for the ManyEars PR2 demo.
    ///
    /// Input topics:
    ///
    ///  - source_pose: geometry_msgs/PoseStamped output from ManyEars, should
    ///    correspond to the loudest source.
    ///
    /// Output topics:
    ///
    ///  - cmd_vel: Base rotation to face the latest sound source.
    ///  - ~last_pose: For diagnostic purposes, output the pose transformed in
    ///    the fixed frame.
    ///
    /// Parameters:
    /// 
    ///  - pointing_frame: Name of the pointing frame for PointHeadAction.
    ///    Will always orient X toward the detected source.
    ///    Default: "head_tilt_link".
    ///  - fixed_frame: Used to transform detected poses for base control.
    ///    Default: "odom".
    ///  - min_duration: Minimum duration to get to a given pose.
    ///    Default: 0.5 s.
    ///  - max_vel: Maximum velocity in rad/s.
    ///    Default: 1.0 rad/s.
    ///  - timeout: Maximum age of a source pose for base control.
    ///    Default: 2.0 s.
    ///  - period: Control period for velocity commands.
    ///    Default: 0.1 s.
    ///
    class PR2Demo
    {
    public:
        PR2Demo(ros::NodeHandle& n, ros::NodeHandle& np):
            last_update_(0)
        {
            double p; // Used for period parameters.

            np.param("pointing_frame", pointing_frame_, 
                std::string("head_tilt_link"));
            np.param("fixed_frame", fixed_frame_,
                std::string("odom"));
            np.param("min_duration", p, 0.5);
            min_duration_ = ros::Duration(p);

            np.param("max_vel", max_vel_, 1.0);

            np.param("timeout", p, 5.0);
            timeout_ = ros::Duration(p);

            np.param("period", p, 0.1);
            timer_ = n.createTimer(ros::Duration(p), &PR2Demo::timerCB, this);

            sub_source_pose_ = n.subscribe("source_pose", 1, 
                &PR2Demo::poseCB, this);
            pub_cmd_vel_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            pub_last_pose_ = np.advertise<geometry_msgs::PoseStamped>(
                "last_pose", 1);

            ph_client_.reset(new PointHeadClient(
                std::string("/head_traj_controller/point_head_action"), 
                true));
            while (!ph_client_->waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the point_head_action server...");
            }

        }

        void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            // First, transform the pose in a fixed frame and save it.
            try 
            {
                tf_.waitForTransform(fixed_frame_, msg->header.frame_id, 
                    msg->header.stamp, min_duration_);
                tf_.transformPose(fixed_frame_, msg->header.stamp, *msg, 
                    fixed_frame_, last_pose_);
            } 
            catch (tf::TransformException e)
            {
                ROS_DEBUG_THROTTLE(1.0, "TF Exception: %s", e.what());
            }

            last_update_ = msg->header.stamp;
        }

        void timerCB(const ros::TimerEvent&)
        {
            geometry_zeros::Twist zero;
            zero.linear.x = zero.linear.y = zero.angular.z = 0.0;

            if (last_update_.isZero() || 
                ((ros::Time::now() - last_update_) > timeout_))
            {
                // Invalid / obsolete goal, publish a zero twist to avoid
                // applying older commands indefinitely.
                pub_cmd_vel_.publish(zero);
                return;
            }

            // Force the last pose time to the latest common time between the
            // fixed frame and the source frame.
            tf_.getLatestCommonTime(pointing_frame_, fixed_frame_, 
                last_pose_.header.stamp, NULL);


            // Just send the last valid sound pose as an action goal
            // for the head.
            // No filtering performed.
            pr2_controllers_msgs::PointHeadGoal goal;
            goal.target.header.frame_id = last_pose_.header.frame_id;
            goal.target.point = last_pose_.pose.position;
            goal.pointing_frame = pointing_frame_;
            goal.pointing_axis.x = 1.0;
            goal.pointing_axis.y = 0.0;
            goal.pointing_axis.z = 0.0;
            goal.min_duration = min_duration_;
            goal.max_velocity = max_vel_;
            ph_client_->sendGoal(goal);

            // Send velocity commmands (and diagnostic output) with turn_toward.
            if (pub_last_pose_.getNumSubscribers())
                pub_last_pose_.publish(last_pose_);

            geometry_msgs::Twist cmd;
            // calcVel returns true only if a valid command could be obtained.
            // Send zero otherwise.
            if (turn_.calcVel(last_pose_, cmd))
                pub_cmd_vel_.publish(cmd);
            else
                pub_cmd_vel_.publish(zero);
        }

    private:
        autonav::TurnToward turn_;

        ros::Subscriber sub_source_pose_;
        ros::Publisher pub_cmd_vel_;
        ros::Publisher pub_last_pose_;

        ros::Timer timer_;
        tf::TransformListener tf_;

        typedef actionlib::SimpleActionClient<
            pr2_controllers_msgs::PointHeadAction> PointHeadClient;
        boost::scoped_ptr<PointHeadClient> ph_client_;

        std::string pointing_frame_;
        std::string fixed_frame_;
        ros::Duration min_duration_;
        double max_vel_;
        ros::Duration timeout_;

        geometry_msgs::PoseStamped last_pose_;
        ros::Time last_update_;
    };

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pr2_demo");

    ros::NodeHandle n, np("~");

    manyears_pr2::PR2Demo node(n, np);
    ros::spin();

    return 0;
}

