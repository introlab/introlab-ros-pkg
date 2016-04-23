#include <pluginlib/class_list_macros.h>
#include "x264_image_transport/x264_publisher.h"
#include "x264_image_transport/x264_subscriber.h"

PLUGINLIB_EXPORT_CLASS( x264_image_transport::x264Publisher, image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS( x264_image_transport::x264Subscriber, image_transport::SubscriberPlugin)
