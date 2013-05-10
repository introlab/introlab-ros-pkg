#include <image_transport/simple_publisher_plugin.h>
#include <dynamic_reconfigure/server.h>
#include <x264_image_transport/x264PublisherConfig.h>
#include <x264_image_transport/x264Packet.h>

extern "C"
{
	#include "x264.h"
}


namespace x264_image_transport {

	class x264Publisher : public image_transport::SimplePublisherPlugin<x264_image_transport::x264Packet>
	{
	public:
	  x264Publisher();
	
	  ~x264Publisher();
	  
	  // Return the system unique string representing the theora transport type
	  virtual std::string getTransportName() const { return "x264"; }
	
	protected:
	
	  // Overridden to tweak arguments and set up reconfigure server
	  virtual void advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
	                             const image_transport::SubscriberStatusCallback  &user_connect_cb,
	                             const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
	                             const ros::VoidPtr &tracked_object, bool latch);
	  
	  // Callback to send header packets to new clients
	  virtual void connectCallback(const ros::SingleSubscriberPublisher& pub);
	
	  // Main publish function
	  virtual void publish(const sensor_msgs::Image& message,
	                       const PublishFn& publish_fn) const;
	
	  // Dynamic reconfigure support
	  typedef x264_image_transport::x264PublisherConfig Config;
	  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	
	  void configCb(Config& config, uint32_t level);
	
      void initialize_codec(int width,int height,int fps) const;
	
	  // Some data is preserved across calls to publish(), but from the user's perspective publish() is
	  // "logically const"
	  mutable std::vector<x264_image_transport::x264Packet> stream_header_;
	  
	  //x264 encoder stuff
	  mutable x264_param_t x264_codec_param;
	  mutable x264_t *x264_encoder;
	  mutable x264_picture_t x264_pic_in;
	  mutable x264_picture_t x264_pic_out;
	  
	  //initialized flag
	  mutable bool initialized_;
	  
};

} //namespace x264_image_transport
