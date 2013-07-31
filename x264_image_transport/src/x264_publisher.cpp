#include "x264_image_transport/x264_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <cstdio> //for memcpy

namespace x264_image_transport {

	namespace enc = sensor_msgs::image_encodings;
	
	
	x264Publisher::x264Publisher()
	: encFmtCtx_(NULL),
	  encCdcCtx_(NULL),
	  encFrame_(NULL),
	  sws_ctx_(NULL),
	  initialized_(false)
	{
		

		
	}
	
	x264Publisher::~x264Publisher()
	{
		ROS_INFO("x264Publisher::~x264Publisher()");
		

         if(encCdcCtx_)
         {
             avcodec_close(encCdcCtx_);
             encCdcCtx_ = 0;
         }
         if(encFmtCtx_)
         {
             avformat_close_input(&encFmtCtx_);
             encFmtCtx_ = 0;
         }
         if(encFrame_)
         {
             av_free(encFrame_);
             encFrame_ = 0;
         }

         if(sws_ctx_)
         {
             sws_freeContext(sws_ctx_);
             sws_ctx_ = 0;
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
#ifndef __APPLE__	  
	  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
	  ReconfigureServer::CallbackType f = boost::bind(&x264Publisher::configCb, this, _1, _2);
	  reconfigure_server_->setCallback(f);
#endif	  
	}

#ifndef __APPLE__	
	void x264Publisher::configCb(Config& config, uint32_t level)
	{
		//TODO HANDLE CONFIGURATION...
		ROS_INFO("Configuration not handled yet");
	}
#endif	
	
	void x264Publisher::initialize_codec(int width, int height, int fps, const std::string& encoding) const
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
	    encCdcCtx_ = avcodec_alloc_context3(codec);
        if (!encCdcCtx_)
        {
           ROS_ERROR("Unable to allocate encoder context");
           return;
        }
	    
	    
	    //Setup some parameter
        /* put sample parameters */
        encCdcCtx_->bit_rate = 512000; //Seems a good starting point
        encCdcCtx_->qmax = 51; //Allow big degradation
        /* resolution must be a multiple of two */
        encCdcCtx_->width = width;
        encCdcCtx_->height = height;
        /* frames per second */
        //WARNING FPS = 1/TIME_BASE---------------------
        AVRational fr;
        fr.num = fps;
        fr.den = 1;
        encCdcCtx_->time_base.num = fr.den;
        encCdcCtx_->time_base.den = fr.num;
        
        //Theory : High gop_size (more b-frame and p-frame) = High CPU, Low Bandwidth
        //       : Low gop_size (more intra-frame)  = Low CPU, High Bandwidth
        encCdcCtx_->gop_size = (fr.num/fr.den)/2; /* emit one group of picture (which has an intra frame) every  frameRate/2 */
        encCdcCtx_->max_b_frames=2;
        encCdcCtx_->pix_fmt = PIX_FMT_YUV420P;
        
        av_opt_set(encCdcCtx_->priv_data, "profile", "main", AV_OPT_SEARCH_CHILDREN);
        av_opt_set(encCdcCtx_->priv_data, "tune", "zerolatency", AV_OPT_SEARCH_CHILDREN);
        av_opt_set(encCdcCtx_->priv_data, "preset", "ultrafast", AV_OPT_SEARCH_CHILDREN);
        
	    /* open the encoder codec */
        if (avcodec_open2(encCdcCtx_, codec, NULL) < 0)
        {
            ROS_ERROR("Could not open the encoder");
            return;
        }

        /** allocate and AVFRame for encoder **/
        encFrame_ = avcodec_alloc_frame();
        if (!encFrame_)
        {
            ROS_ERROR("Cannot allocate frame");
            return;
        }

        // Prepare the software scale context
        // Will convert from RGB24 to YUV420P
        if (encoding == enc::BGR8)
        {        
            sws_ctx_ = sws_getContext(width, height, PIX_FMT_BGR24, //src
                                    encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
                                    SWS_FAST_BILINEAR, NULL, NULL, NULL);
        }
        else if (encoding == enc::RGB8)
        {
            sws_ctx_ = sws_getContext(width, height, PIX_FMT_RGB24, //src
                                    encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
                                    SWS_FAST_BILINEAR, NULL, NULL, NULL);
        }
        else
        {
            ROS_WARN("Encoding not supported : %s, default to RGB8",encoding.c_str());
            sws_ctx_ = sws_getContext(width, height, PIX_FMT_RGB24, //src
                                    encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
                                    SWS_FAST_BILINEAR, NULL, NULL, NULL);
        }

        //Allocate picture region
        avpicture_alloc((AVPicture *)encFrame_, PIX_FMT_YUV420P, width, height);

        //Initialize packet
        av_init_packet(&encodedPacket_);
        encodedPacket_.data = NULL;    // packet data will be allocated by the encoder
        encodedPacket_.size = 0;
			
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
		      
		
		      initialize_codec(width,height,fps, message.encoding);  
		}

        //Something went wrong
        if (!initialized_)
            return;
        
        //Pointer to RGB DATA    
        unsigned char* ptr[1];		
		ptr[0] = (unsigned char*) &message.data[0];
           
        //Let's convert the image to something ffmpeg/x264 understands		
   		//Get scaling context...    
        sws_scale(sws_ctx_,ptr,&srcstride,0,height,encFrame_->data, encFrame_->linesize);
 
        //int got_output = 0;
        int ret = 0;
        uint8_t buffer[height * srcstride];
        
        //Soon to be used...
        //ret = avcodec_encode_video2(encCdcCtx_, &encodedPacket_, encFrame_, &got_output);
        ret = avcodec_encode_video(encCdcCtx_, buffer, height * srcstride, encFrame_);
 
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
		    	
                //Affect header
                packet.header = message.header;

		    	//publish
		    	publish_fn(packet);
		    	
		    	//Not yet used...
		    	av_free_packet(&encodedPacket_);
                av_init_packet(&encodedPacket_);        
        }		
	}
} //namespace x264_image_transport
