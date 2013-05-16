#include "x264_image_transport/x264_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <cstdio> //for memcpy


extern "C"
{
	#include <libswscale/swscale.h>
}

namespace x264_image_transport {

	namespace enc = sensor_msgs::image_encodings;
	
	
	x264Publisher::x264Publisher()
	: m_encFmtCtx(NULL),
	  m_encCdcCtx(NULL),
	  m_encFrame(NULL),
	  sws_ctx(NULL),
	  initialized_(false)
	{
		

		
	}
	
	x264Publisher::~x264Publisher()
	{
		ROS_INFO("x264Publisher::~x264Publisher()");
		

         if(m_encCdcCtx)
         {
             avcodec_close(m_encCdcCtx);
             m_encCdcCtx = 0;
         }
         if(m_encFmtCtx)
         {
             avformat_close_input(&m_encFmtCtx);
             m_encFmtCtx = 0;
         }
         if(m_encFrame)
         {
             av_free(m_encFrame);
             m_encFrame = 0;
         }

         if(sws_ctx)
         {
             sws_freeContext(sws_ctx);
             sws_ctx = 0;
         }
	}
	
	void x264Publisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
	                                    const image_transport::SubscriberStatusCallback  &user_connect_cb,
	                                    const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
	                                    const ros::VoidPtr &tracked_object, bool latch)
	{
		
	  //TODO UNDERSTAND THOSE PARAMETERS...	
		
		
	  // queue_size doesn't account for the 3 header packets, so we correct (with a little extra) here.
	  queue_size += 4;
	  // Latching doesn't make a lot of sense with this transport. Could try to save the last keyframe,
	  // but do you then send all following delta frames too?
	  latch = false;
	  typedef image_transport::SimplePublisherPlugin<x264_image_transport::x264Packet> Base;
	  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);
	
	  // Set up reconfigure server for this topic
	  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
	  ReconfigureServer::CallbackType f = boost::bind(&x264Publisher::configCb, this, _1, _2);
	  reconfigure_server_->setCallback(f);
	}
	
	void x264Publisher::configCb(Config& config, uint32_t level)
	{
		//TODO HANDLE CONFIGURATION...
		ROS_INFO("Configuration not handled yet");
	}
	
	
	void x264Publisher::initialize_codec(int width, int height, int fps) const
	{
	
	    // must be called before using avcodec lib
        avcodec_register_all();
        //Register codecs, devices and formats
        av_register_all();
        //Initialize network, this is new from april 2013, it will initialize the RTP
        avformat_network_init();
	
	    //Codec
	    AVCodec *codec = 0;
	    
	    codec = avcodec_find_encoder(CODEC_ID_H264);	    
	    if (!codec)
	    {
	        ROS_ERROR("Unable to find H264 encoder, ffmpeg version too old ?");
	        return;
	    }
	    
	    //Context
	    m_encCdcCtx = avcodec_alloc_context3(codec);
        if (!m_encCdcCtx)
        {
           ROS_ERROR("Unable to allocate encoder context");
           return;
        }
	    
	    
	    //Setup some parameter
        /* put sample parameters */
        m_encCdcCtx->bit_rate = 512000; //Seems a good starting point
        m_encCdcCtx->qmax = 51; //Allow big degradation
        /* resolution must be a multiple of two */
        m_encCdcCtx->width = width;
        m_encCdcCtx->height = height;
        /* frames per second */
        //WARNING FPS = 1/TIME_BASE---------------------
        AVRational fr;
        fr.num = fps;
        fr.den = 1;
        m_encCdcCtx->time_base.num = fr.den;
        m_encCdcCtx->time_base.den = fr.num;
        
        //Theory : High gop_size (more b-frame and p-frame) = High CPU, Low Bandwidth
        //       : Low gop_size (more intra-frame)  = Low CPU, High Bandwidth
        m_encCdcCtx->gop_size = (fr.num/fr.den)/2; /* emit one group of picture (which has an intra frame) every  frameRate/2 */
        m_encCdcCtx->max_b_frames=2;
        m_encCdcCtx->pix_fmt = PIX_FMT_YUV420P;
        
        av_opt_set(m_encCdcCtx->priv_data, "profile", "main", AV_OPT_SEARCH_CHILDREN);
        av_opt_set(m_encCdcCtx->priv_data, "tune", "zerolatency", AV_OPT_SEARCH_CHILDREN);
        av_opt_set(m_encCdcCtx->priv_data, "preset", "ultrafast", AV_OPT_SEARCH_CHILDREN);
        
	    /* open the encoder codec */
        if (avcodec_open2(m_encCdcCtx, codec, NULL) < 0)
        {
            ROS_ERROR("Could not open the encoder");
            return;
        }

        /** allocate and AVFRame for encoder **/
        m_encFrame = avcodec_alloc_frame();
        if (!m_encFrame)
        {
            ROS_ERROR("Cannot allocate frame");
            return;
        }

        // Prepare the software scale context
        // Will convert from RGB24 to YUV420P        
        sws_ctx = sws_getContext(width, height, PIX_FMT_RGB24, //src
                                m_encCdcCtx->width, m_encCdcCtx->height, m_encCdcCtx->pix_fmt, //dest
                                SWS_FAST_BILINEAR, NULL, NULL, NULL);

        //Allocate picture region
        avpicture_alloc((AVPicture *)m_encFrame, PIX_FMT_YUV420P, width, height);

        //Initialize packet
        av_init_packet(&m_encodedPacket);
        m_encodedPacket.data = NULL;    // packet data will be allocated by the encoder
        m_encodedPacket.size = 0;
			
		initialized_ = true;
		ROS_INFO("x264Publisher::initialize_codec(): codec initialized (width: %i, height: %i, fps: %i)",width,height,fps);
	}
	
	
	void x264Publisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
	{		
	    ROS_INFO("x264Publisher::connectCallback");	 
	    //This will always occur after the first publish...	 
	}
	
	
	
	void x264Publisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
	{
		//ROS_INFO("x264Publisher::publish");		
        int width = message.width;
        int height = message.height;
        int fps = 15;
        int srcstride = message.step;
		
		if (!initialized_)
		{
		      initialize_codec(width,height,fps);  
		}

        //Something went wrong
        if (!initialized_)
            return;
        
        //Pointer to RGB DATA    
        unsigned char* ptr[1];		
		ptr[0] = (unsigned char*) &message.data[0];
           
        //Let's convert the image to something ffmpeg/x264 understands		
   		//Get scaling context...    
        sws_scale(sws_ctx,ptr,&srcstride,0,height,m_encFrame->data, m_encFrame->linesize);
 
        //int got_output = 0;
        int ret = 0;
        uint8_t buffer[height * srcstride];
        
        //Soon to be used...
        //ret = avcodec_encode_video2(m_encCdcCtx, &m_encodedPacket, m_encFrame, &got_output);
        ret = avcodec_encode_video(m_encCdcCtx, buffer, height * srcstride, m_encFrame);
 
        if (ret > 0)
        {
                // OK, Let's send our packets...
		    	x264_image_transport::x264Packet packet;
		    	
		    	//Set data
		    	packet.data.resize(ret);
		    	
		    	//Set width & height
		    	packet.img_width = width; 
		    	packet.img_height = height;
		    	
		    	//copy NAL data
		    	memcpy(&packet.data[0],buffer,ret);
		    	
		    	//publish
		    	publish_fn(packet);
		    	
		    	//Not yet used...
		    	av_free_packet(&m_encodedPacket);
                av_init_packet(&m_encodedPacket);        
        }		
	}
} //namespace x264_image_transport
