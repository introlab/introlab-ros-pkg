#include <image_transport/simple_subscriber_plugin.h>
#include <x264_image_transport/x264Packet.h>

#ifndef __APPLE__
    //Dynamic reconfigure not yet working on iOS
    #include <x264_image_transport/x264SubscriberConfig.h>
    #include <dynamic_reconfigure/server.h>
#endif

extern "C"
{
	#include <libavcodec/avcodec.h>
	#include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
}

namespace x264_image_transport {

	class x264Subscriber : public image_transport::SimpleSubscriberPlugin<x264_image_transport::x264Packet>
	{
	public:
	      x264Subscriber();
	      virtual ~x264Subscriber();
	      virtual std::string getTransportName() const { return "x264"; }
	
	protected:
	
        // Overridden to bump queue_size, otherwise we might lose headers
        // Overridden to tweak arguments and set up reconfigure server
        virtual void subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
        const Callback &callback, const ros::VoidPtr &tracked_object,
        const image_transport::TransportHints &transport_hints);

        // The function that does the actual decompression and calls a user supplied callback with the resulting image
        virtual void internalCallback(const x264_image_transport::x264PacketConstPtr &msg, const Callback& user_cb);


        // Dynamic reconfigure support
#ifndef __APPLE__       
        typedef x264_image_transport::x264SubscriberConfig Config;
        typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
        boost::shared_ptr<ReconfigureServer> reconfigure_server_;
        void configCb(Config& config, uint32_t level);
#endif

        void initialize_codec(int width, int height);
        void convert_rgb(AVCodecContext *codec, AVFrame *inFrame, AVFrame *outFrame);

        sensor_msgs::ImagePtr latest_image_;
        bool initialized_;


        AVFormatContext *m_pFormatCtx;
        AVCodecContext  *m_pCodecCtx;
        AVCodec         *m_pCodec;
        AVFrame         *m_pFrame;
        AVFrame         *m_pFrameRGB;
        AVPacket        m_packet;
        SwsContext      *m_img_convert_ctx;
        AVInputFormat* m_inputFormat;
        // AVFormatParameters m_formatParams;
        uint8_t         *m_buffer;
	};

} //namespace x264_image_transport
