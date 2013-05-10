#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "x264_test_publisher");
  ros::NodeHandle nh("~");

  int refreshRate;
  nh.param<int>("refresh_rate", refreshRate, 15);

  std::string topicName;
  nh.param<std::string>("topic_name", topicName, "image");

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(topicName, 10);

  
  sensor_msgs::ImagePtr msg(new sensor_msgs::Image);  
  msg->width = 1024;
  msg->height = 768;
  msg->step = msg->width*3;
  msg->data.resize(msg->step * msg->height);
  msg->encoding = "8UC3";
  

  
  
  ros::Rate loop_rate(refreshRate);
  while (nh.ok()) 
  {
	  
	//Put random pixels in image...
	for (unsigned int i = 0; i < msg->step * msg->height;i++)
	{
	  	msg->data[i] = rand();
	}
    //ROS_INFO("Publishing image");
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
