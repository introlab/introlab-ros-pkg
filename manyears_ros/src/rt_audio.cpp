#include "RtAudio.h"
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
//ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include "global_value.h"
#include <manyears_ros/AudioStream.h>
//#include "topic_filters/filtered_publisher.hpp"

namespace rt_audio_node
{
	int record( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
			 double streamTime, RtAudioStreamStatus status, void *userData );

	class rt_audio
	{
	public:

		rt_audio(ros::NodeHandle n): local_nh_("~"),
			pub_(n.advertise<manyears_ros::AudioStream>("audio_stream",100)),
			frame_number_(0)
		{
			//Adertise ROS msg of audio stream
			local_nh_.param("save_raw_file",save_raw_file_, false);
			//Init
			rtaudio_ = new RtAudio();
			if ( rtaudio_->getDeviceCount() < 1 ) {
				ROS_ERROR("No audio devices found!");
			}
			// Determine the number of devices available
			unsigned int numberDevices = rtaudio_->getDeviceCount();

			// Variable to stock the approriate device
			uint selected_device = -1;

			// Scan through devices for various capabilities
			RtAudio::DeviceInfo info;

			for (unsigned int indexDevice = 0; indexDevice < numberDevices; indexDevice++ )
			{
				info = rtaudio_->getDeviceInfo(indexDevice);
				if ( info.probed == true )
				{
                                        ROS_INFO_STREAM(" card "<< indexDevice << info.name.c_str()<<" Inputs : "<<info.inputChannels);
					//Take the device with 8 output channels
                                        if( info.inputChannels >= 8)
                                        {
						selected_device = indexDevice;
                                                break;
                                        }
				}
			}

			RtAudio::StreamParameters iparameters;
			//Take the last device for the input (normally the PCI sound card)
			iparameters.deviceId = selected_device;
			iparameters.nChannels = manyears_global::nb_microphones_s;
			iparameters.firstChannel = 0;
			RtAudio::StreamOptions audioOptions;
			int sampleRate = manyears_global::sample_rate_s;
			unsigned int bufferFrames = manyears_global::samples_per_frame_s; // 512 sample frames is the buffer hope size for manyears

			//Write into file to sav audio stream
			if(save_raw_file_)
			{
				//Get now date
				time_t rawtime;
				struct tm * timeinfo;
				char buffer[80];
				time(&rawtime);
				timeinfo = localtime(&rawtime);
				strftime(buffer, 80, "/data/raw/%Y-%m-%d_%H-%M-%S.raw", timeinfo);

				//std::string save_file_path = ros::package::getPath("manyears_ros") + "/data/test_capture_raw_audio.raw";
				std::string save_file_path = ros::package::getPath("manyears_ros") + buffer;
				//output_file_ = fopen("data/test_capture_audio.raw","wb");
				output_file_ = fopen(save_file_path.c_str(),"wb");
				if(output_file_ == NULL)
					ROS_INFO("FILE %s not open", save_file_path.c_str());
				else
					ROS_INFO("Save FILE");
			}
			try {
				printf("deviceId : %d, nbChannels : %d, sample rate : %d, bufferSize : %d"
					, iparameters.deviceId, iparameters.nChannels, sampleRate, bufferFrames);

				rtaudio_->openStream( NULL, &iparameters, RTAUDIO_SINT16, sampleRate, &bufferFrames, &rt_audio_node::record, this, &audioOptions);
				ROS_INFO("Open done");
				rtaudio_->startStream();
			}
			catch ( RtError& e ) {
				e.printMessage();
				exit( 0 );
			}
			ROS_INFO("End of RtAudio init");

		}

		~rt_audio()
		{
			//destruction
			try {
				// Stop the stream
				rtaudio_->stopStream();
			}
			catch (RtError& e) {
				e.printMessage();
			}

			if ( rtaudio_->isStreamOpen() ) rtaudio_->closeStream();
			delete rtaudio_;
			//close file
			if(save_raw_file_)
				fclose(output_file_);
		}

		void publish_msg(manyears_ros::AudioStream& msg_to_send)
		{
			msg_to_send.frame_number = frame_number_;
			pub_.publish(msg_to_send);
			frame_number_++;
		}

		void write_in_file(short * data)
		{
			fwrite(data, sizeof(short), 1, output_file_);
		}

		bool get_save_raw_file()
		{
			return save_raw_file_;
		}

	private:
		//ROS variables
		ros::NodeHandle local_nh_;
		ros::Publisher pub_;
		//topic_filters::filtered_publisher pub_;
		//RT audio variables
		RtAudio* rtaudio_;
		uint frame_number_;
		
		bool save_raw_file_;
		FILE* output_file_;
	};

	int record( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
			 double streamTime, RtAudioStreamStatus status, void *userData )
	{
		if ( status )
			std::cout << "Stream overflow detected!" << std::endl;

		rt_audio_node::rt_audio* rtaudio_ptr = static_cast<rt_audio_node::rt_audio*>(userData);
		// Do something with the data in the "inputBuffer" buffer.
		manyears_ros::AudioStream audio_stream_msg;
		audio_stream_msg.stream_buffer.resize(nBufferFrames*manyears_global::nb_microphones_s);

		signed short *  buffer_to_send = (signed short *)inputBuffer;
		for(uint i=0; i< nBufferFrames*manyears_global::nb_microphones_s; i++)
		{
			audio_stream_msg.stream_buffer[i] = (short) *(buffer_to_send++);
			if(rtaudio_ptr->get_save_raw_file() == true)
				rtaudio_ptr->write_in_file(&audio_stream_msg.stream_buffer[i]);
		}
		rtaudio_ptr->publish_msg(audio_stream_msg);

		return 0;
	}

}

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "rt_audio_publisher");
	ros::NodeHandle n;	

	rt_audio_node::rt_audio ra(n);

	//ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	//spinner.spin();
	ros::spin();

	return 0;
}

