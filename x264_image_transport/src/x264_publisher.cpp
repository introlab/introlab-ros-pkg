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
	:  x264_encoder(NULL)
	{
		
		//TODO configurable
		unsigned int fps = 15;
		unsigned int width = 1024;
		unsigned int height = 768;
		
		
		x264_param_default_preset(&x264_codec_param, "veryfast", "zerolatency");
		x264_codec_param.i_threads = 1;
		x264_codec_param.i_width = width; 
		x264_codec_param.i_height = height; 
		x264_codec_param.i_fps_num = fps;
		x264_codec_param.i_fps_den = 1;
		// Intra refresh:
		x264_codec_param.i_keyint_max = fps;
		x264_codec_param.b_intra_refresh = 1;
		//Rate control:
		x264_codec_param.rc.i_rc_method = X264_RC_CRF;
		x264_codec_param.rc.f_rf_constant = 25;
		x264_codec_param.rc.f_rf_constant_max = 35;
		//For streaming:
		x264_codec_param.b_repeat_headers = 1;
		x264_codec_param.b_annexb = 1;
		x264_param_apply_profile(&x264_codec_param, "baseline");
		
		//Setup encoder
		x264_encoder = x264_encoder_open(&x264_codec_param);
		
		
		//Alloc images (x264 requires YUV420P)
		x264_picture_alloc(&x264_pic_in, X264_CSP_I420, width, height);
		
		
		ROS_INFO("x264Publisher::x264Publisher() initialized");
		
	}
	
	x264Publisher::~x264Publisher()
	{
		
		x264_encoder_close(x264_encoder);
		
		//TODO destroy pictures...
		
		
		
	
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
		ROS_ERROR("Configuration not handled");
	  
	  
	}
	
	void x264Publisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
	{
		
	  ROS_INFO("x264Publisher::connectCallback");
	 
	}
	
	
	
	void x264Publisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
	{
		//ROS_INFO("x264Publisher::publish");
		
		//cv_bridge::CvImageConstPtr cv_ptr;
		
		//Let's convert the image to something x264
	   		
	   		int width = 1024;
	   		int height = 768;
	   		int srcstride = width * 3;
	   		
	   		//Get scaling context...
	   		struct SwsContext* convertCtx = sws_getContext(width, height, PIX_FMT_RGB24, width, height, PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
	   		
	 
	   		//unsigned char data[width*height*3];
	   		//sws_scale(encoderSwsContext, encoderRawFrame->data, encoderRawFrame->linesize, 0, src_height, encoderRescaledFrame->data, encoderRescaledFrame->linesize); 
	   		
	   		/**
 * Scales the image slice in srcSlice and puts the resulting scaled
 * slice in the image in dst. A slice is a sequence of consecutive
 * rows in an image.
 *
 * Slices have to be provided in sequential order, either in
 * top-bottom or bottom-top order. If slices are provided in
 * non-sequential order the behavior of the function is undefined.
 *
 * @param context   the scaling context previously created with
 *                  sws_getContext()
 * @param srcSlice  the array containing the pointers to the planes of
 *                  the source slice
 * @param srcStride the array containing the strides for each plane of
 *                  the source image
 * @param srcSliceY the position in the source image of the slice to
 *                  process, that is the number (counted starting from
 *                  zero) in the image of the first row of the slice
 * @param srcSliceH the height of the source slice, that is the number
 *                  of rows in the slice
 * @param dst       the array containing the pointers to the planes of
 *                  the destination image
 * @param dstStride the array containing the strides for each plane of
 *                  the destination image
 * @return          the height of the output slice
 
int sws_scale(struct SwsContext *context, const uint8_t* const srcSlice[], const int srcStride[],
              int srcSliceY, int srcSliceH, uint8_t* const dst[], const int dstStride[]);
*/	   		

			unsigned char* ptr[1];
			
			ptr[0] = (unsigned char*) &message.data[0];

			sws_scale(convertCtx,ptr,&srcstride,0,height,x264_pic_in.img.plane, x264_pic_in.img.i_stride);
	   		
			sws_freeContext(convertCtx);  		
	   		
	   		//Encode
	   		x264_nal_t* nals;
			int i_nals;
			int frame_size = x264_encoder_encode(x264_encoder, &nals, &i_nals, &x264_pic_in, &x264_pic_out);
			if (frame_size >= 0)
			{
				//ROS_INFO("x264_encoder_encode returned size : %i i_nals: %i",frame_size,i_nals);
				
				for (int i= 0; i < i_nals; i++)
				{
					//ROS_INFO("NAL : %i, size: %i",i,nals[i].i_payload);
					
			    	// OK, Let's send our packets...
			    	x264_image_transport::x264Packet packet;
			    	
			    	packet.data.resize(nals[i].i_payload);
			    	
			    	//copy NAL data
			    	memcpy(&packet.data[0],nals[i].p_payload,nals[i].i_payload);
			    	
			    	//publish
			    	publish_fn(packet);
			    	
				}
			    
			}
	   		
	   			   		
		
	
	  
	    
	    
	}


} //namespace x264_image_transport
