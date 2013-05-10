#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

extern "C"
{
    #include "libavcodec/avcodec.h"
    #include "libavformat/avio.h"
    #include "libavformat/avformat.h"
    #include "libswscale/swscale.h"
}

class x264TestSubscriber
{
public:

    x264TestSubscriber()
    {
        ROS_INFO("x264TestSubscriber()");
        nh_ = ros::NodeHandle("~");
        it_ = new image_transport::ImageTransport(nh_);
        nh_.param<std::string>("topic_name", topicName_, "/x264_test_publisher/image");
        sub_ =  it_->subscribe(topicName_, 5, &x264TestSubscriber::imageCallback, this);
    }


    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("imageCallback(const sensor_msgs::ImageConstPtr& msg width: %i, height: %i)",msg->width, msg->height);
    }



protected:

    ros::NodeHandle nh_;
    image_transport::Subscriber sub_;
    image_transport::ImageTransport *it_;
    std::string topicName_;

};



int main(int argc, char** argv)
{
    av_register_all();
    ros::init(argc, argv, "x264_test_subscriber");
    x264TestSubscriber node;
    ros::spin();
    return 0;
}
