#include <pluginlib/class_list_macros.h>
#include "x264_transport/x264_publisher.h"
#include "x264_transport/x264_subscriber.h"

PLUGINLIB_DECLARE_CLASS(image_transport, x264_pub, x264_transport::x264Publisher, image_transport::PublisherPlugin)

PLUGINLIB_DECLARE_CLASS(image_transport, x264_sub, x264_transport::x264Subscriber, image_transport::SubscriberPlugin)
