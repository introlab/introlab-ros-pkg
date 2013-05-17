#include <image_transport/simple_publisher_plugin.h>
#include <dynamic_reconfigure/server.h>
#include <x264_image_transport/x264Packet.h>

#ifndef __APPLE__
    #include <x264_image_transport/x264PublisherConfig.h>
#endif


extern "C"
{
	#include "libavcodec/avcodec.h"
    #include "libavformat/avio.h"
    #include "libavformat/avformat.h"
    #include "libswscale/swscale.h"
    #include "libavutil/opt.h"
    #include "libavutil/imgutils.h"
    #include "libavutil/samplefmt.h"
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
#ifndef __APPLE__	  
	  typedef x264_image_transport::x264PublisherConfig Config;
	  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	  boost::shared_ptr<ReconfigureServer> reconfigure_server_;	
	  void configCb(Config& config, uint32_t level);
#endif	
	
      void initialize_codec(int width,int height,int fps) const;
	
	  // Some data is preserved across calls to publish(), but from the user's perspective publish() is
	  // "logically const"
	  mutable std::vector<x264_image_transport::x264Packet> stream_header_;
	   
      /** ENCODER VAR **/
      mutable AVFormatContext *encFmtCtx_;
      mutable AVCodecContext  *encCdcCtx_;
      mutable AVFrame *encFrame_;
      mutable AVPacket encodedPacket_;

      /** SOFTWARE SCALE CONTEXT **/
      mutable SwsContext *sws_ctx_;

	  //initialized flag
	  mutable bool initialized_;
	  
};

} //namespace x264_image_transport
