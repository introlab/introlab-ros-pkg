/*************************
 *
 * manyears_ros.cpp
 *
 * ROS node to manage the manyears library
 *
 * ***********************/

#include <ros/ros.h>
#include "global_value.h"
#include <manyears_ros/ManyEarsTrackedAudioSource.h>
#include <manyears_ros/AudioStream.h>
//#include <topic_filters/filtered_publisher.hpp>
//Standard C includes
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <sstream>
#include <string>

#include <sstream>
#include <fstream>

using namespace std;

//Manyears includes
extern "C"
{
    //#include "beamformer.h"
    //#include "mixture.h"
#include "overallContext.h"
}

namespace manyears_node{

    class many_ears
    {
    public:
        many_ears(ros::NodeHandle n): local_nh_("~"),
        pub_(n.advertise<manyears_ros::ManyEarsTrackedAudioSource>("tracked_sources",1000))
        {
            sequence = 0;

            //Init variables
            ros_init_time_ = ros::Time::now();
            frame_number_ = 0;
            //set data to zero
            memset(audio_raw_data_,0,manyears_global::raw_buffer_size_s * sizeof(short));

            //Init ROS connection and parameters
            local_nh_.param("use_audio_stream", use_audio_stream_, false);
            std::string config_file;
            local_nh_.param("config_file", config_file, std::string("../data/rect_cube.mes"));
            std::string raw_file;
            local_nh_.param("raw_file", raw_file, std::string("../data/one_source.raw"));
            local_nh_.param("frame_id", frame_id_, std::string(manyears_global::microphones_frame_s));
            local_nh_.param("instant_time", instant_time_, false);

            if (instant_time_)
                ROS_INFO("Using instantaneous system time for tracked sources.");
            else
                ROS_INFO("Using estimated time from audio stream for tracked sources.");

            //Separation enable
            local_nh_.param("enable_separation", enable_separation_, false);



            //Use raw file if audio stream is not choose
            if(use_audio_stream_ == false){
                ROS_INFO_STREAM("Use "<< raw_file << " ''raw file''\n");
                //Open file in binary mode
                filePtr_ = fopen(raw_file.c_str(), "rb");
                if (filePtr_ == NULL)
                {
                    ROS_ERROR("Invalid file %s \n",raw_file.c_str());
                }

            }
            else
                sub_ = n.subscribe("stream", 100, &many_ears::audio_stream_cb, this);

            //Write into file to sav audio stream
            std::string save_audio_file;
            local_nh_.param("save_audio_file", save_audio_file, std::string(""));
            if(save_audio_file.compare("") != 0)
            {
                is_audio_saved_ = true;
                output_file_ = fopen(save_audio_file.c_str(),"wb");
                if(output_file_ == NULL)
                    ROS_INFO("FILE not open");
            }
            else
                is_audio_saved_ = false;

            libraryContext_ = createEmptyOverallContext();
            //libraryContext_.myParameters = new struct ParametersStruct;

            //Default parameters
            ParametersLoadDefault(libraryContext_.myParameters);
            //Change your microphone positions here...
            read_file_for_load_param(config_file);




            // Initialize the microphones
            //microphonesInit(libraryContext_.myMicrophones, manyears_global::nb_microphones_s);
            //Change your microphone positions here...

            //This is a copy of what is in myParameters for now
            setup_microphone_positions_and_gains(libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the preprocessor
            preprocessorInit(libraryContext_.myPreprocessor, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the beamformer
            beamformerInit(libraryContext_.myBeamformer, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the mixture
            mixtureInit(libraryContext_.myMixture, libraryContext_.myParameters);

            // Initialize the gss
            gssInit(libraryContext_.myGSS, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the postfilter
            postfilterInit(libraryContext_.myPostfilter, libraryContext_.myParameters);

            // Initialize the postprocessor
            postprocessorInit(libraryContext_.myPostprocessorSeparated, libraryContext_.myParameters);
            postprocessorInit(libraryContext_.myPostprocessorPostfiltered, libraryContext_.myParameters);

            // Initialize the potential sources
            potentialSourcesInit(libraryContext_.myPotentialSources, libraryContext_.myParameters);

            //Initialize the sources
            trackedSourcesInit(libraryContext_.myTrackedSources, libraryContext_.myParameters);

            // Initialize the separated sources
            separatedSourcesInit(libraryContext_.mySeparatedSources, libraryContext_.myParameters);

            // Initialize the postfiltered sources
            postfilteredSourcesInit(libraryContext_.myPostfilteredSources, libraryContext_.myParameters);

            // Initialize the output ?
            outputInit(libraryContext_.myOutputSeparated, libraryContext_.myParameters, NULL, NULL, "/tmp/source_sep_****.wav", '*');
            outputInit(libraryContext_.myOutputPostfiltered, libraryContext_.myParameters, NULL, NULL, "/tmp/source_post****.wav", '*');




        }

        ~many_ears()
        {

            ROS_INFO("~many_ears");



            //terminate
            preprocessorTerminate(libraryContext_.myPreprocessor);
            beamformerTerminate(libraryContext_.myBeamformer);
            mixtureTerminate(libraryContext_.myMixture);
            gssTerminate(libraryContext_.myGSS);
            postfilterTerminate(libraryContext_.myPostfilter);
            postprocessorTerminate(libraryContext_.myPostprocessorSeparated);
            postprocessorTerminate(libraryContext_.myPostprocessorPostfiltered);

            potentialSourcesTerminate(libraryContext_.myPotentialSources);
            trackedSourcesTerminate(libraryContext_.myTrackedSources);
            separatedSourcesTerminate(libraryContext_.mySeparatedSources);
            postfilteredSourcesTerminate(libraryContext_.myPostfilteredSources);

            //Output terminate
            outputTerminate(libraryContext_.myOutputSeparated);
            outputTerminate(libraryContext_.myOutputPostfiltered);



            //Close file (will cleanup memory)
            fclose(filePtr_);
            if(is_audio_saved_ == true)
                fclose(output_file_);

        }

        void terminate()
        {
            manyears_ros::ManyEarsTrackedAudioSource tracked_source_msg;
            //Fill the message header
            //Calculate the estimated time when the trame was catch by the microphones
            
            tracked_source_msg.header.stamp = getTimeStamp();
            //tracked_source_msg.header.stamp = ros_init_time_ + ros::Duration((GLOBAL_FRAMESIZE*GLOBAL_OVERLAP/GLOBAL_FS)*frame_number_);
            tracked_source_msg.header.frame_id = frame_id_;


            pub_.publish(tracked_source_msg);

        }

        ros::Time getTimeStamp()
        {
            if (instant_time_)
                return ros::Time::now();
            else
                return ros_init_time_ + ros::Duration(((float)manyears_global::samples_per_frame_s/manyears_global::sample_rate_s)*frame_number_);
        }

        bool fill_buffer_data_with_raw_file()
        {
            //This does not work well because it does not take account of the channel order
            //bool success = (bool) fread(audio_raw_data_,sizeof(short),manyears_global::raw_buffer_size_s,filePtr_);

            signed short inputShort;
            unsigned int indexBuffer;
            bool success;

            indexBuffer = 0;
            success = true;

            // Load the samples for a complete hop size
            for (int indexSample = 0; indexSample < manyears_global::samples_per_frame_s; indexSample++)
            {
                for (int indexChannel = 0; indexChannel < manyears_global::nb_microphones_s; indexChannel++)
                {
                    if (feof(this->filePtr_)  == 0)
                    {
                        int e = fread(&inputShort, sizeof(short), 1, this->filePtr_);
                        if( e == 0)
                            continue;
                    }
                    else
                    {
                        inputShort = 0;
                        success = false;
                    }
                    this->audio_raw_data_[indexBuffer] = inputShort;
                    indexBuffer++;
                }
            }

            return success;
        }

        void processing_buffer()
        {
            //ROS_INFO("Processing frame : %i \n",frame_number_);
            signed short* buffer_to_write = audio_raw_data_;

            //#1 - Let's create the float data for processing
            for (int channel = 0; channel < manyears_global::nb_microphones_s; channel++)
            {
                for (int frame_index = 0; frame_index < manyears_global::samples_per_frame_s; frame_index++)
                {
                    audio_float_data_[channel][frame_index] = (float) audio_raw_data_[channel + (manyears_global::nb_microphones_s * frame_index)];// / 32768.0;

                    audio_float_data_[channel][frame_index] /= 32768.0;

                    if(is_audio_saved_ == true)
                        fwrite(buffer_to_write++, sizeof(short), 1, output_file_);
                }

                // Copy frames to the beamformer frames, will do 50% overlap internally
                preprocessorPushFrames(libraryContext_.myPreprocessor, manyears_global::samples_per_frame_s, channel);
                preprocessorAddFrame(libraryContext_.myPreprocessor, &audio_float_data_[channel][0], channel, manyears_global::samples_per_frame_s);
            }

            //#2 Preprocess
            preprocessorProcessFrame(libraryContext_.myPreprocessor);

            //#3 Find potential sources from the beamformer
            beamformerFindMaxima(libraryContext_.myBeamformer, libraryContext_.myPreprocessor, libraryContext_.myPotentialSources);

            //#4 Track Sources
            mixtureUpdate(libraryContext_.myMixture, libraryContext_.myTrackedSources, libraryContext_.myPotentialSources);

            if (enable_separation_)
            {

                //#5 Separate sources
                gssProcess(libraryContext_.myGSS, libraryContext_.myPreprocessor, libraryContext_.myTrackedSources, libraryContext_.mySeparatedSources);
                postfilterProcess(libraryContext_.myPostfilter, libraryContext_.mySeparatedSources, libraryContext_.myPreprocessor, libraryContext_.myPostfilteredSources);

                //#6 Postprocess
                postprocessorProcessFrameSeparated(libraryContext_.myPostprocessorSeparated, libraryContext_.myTrackedSources, libraryContext_.mySeparatedSources);
                postprocessorProcessFramePostfiltered(libraryContext_.myPostprocessorPostfiltered, libraryContext_.myTrackedSources, libraryContext_.myPostfilteredSources);
            }


            //#7 Output result
            //Init ROS msg
            manyears_ros::ManyEarsTrackedAudioSource tracked_source_msg;
            //Fill the message header
            //Calculate the estimated time when the trame was catch by the microphones
            tracked_source_msg.header.stamp = getTimeStamp();
            //tracked_source_msg.header.stamp = ros_init_time_ + ros::Duration((GLOBAL_FRAMESIZE*GLOBAL_OVERLAP/GLOBAL_FS)*frame_number_);
            tracked_source_msg.header.frame_id = frame_id_;


            //Output to file
            //outputProcess(libraryContext_.myOutputSeparated, libraryContext_.myPostprocessorSeparated);
            //outputProcess(libraryContext_.myOutputPostfiltered, libraryContext_.myPostprocessorPostfiltered);


            //Search for tracked source
            for (int source_index = 0; source_index < libraryContext_.myParameters->P_GEN_DYNSOURCES; source_index++)
            {
                if(trackedSourcesGetID(libraryContext_.myTrackedSources,source_index) != -1)
                {
                    /*
					ROS_DEBUG("source index : %i X:%f, Y:%f, Z:%f, id: %i \n",source_index,
						   libraryContext_.myTrackedSources->sourcesPosition[source_index][0],
						   libraryContext_.myTrackedSources->sourcesPosition[source_index][1],
						   libraryContext_.myTrackedSources->sourcesPosition[source_index][2],
						   libraryContext_.myTrackedSources->sourcesID[source_index]);

					*/	

                    //Fill an instance of source info msg
                    manyears_ros::SourceInfo tracked_source;
                    tracked_source.source_id = trackedSourcesGetID(libraryContext_.myTrackedSources,source_index);
                    tracked_source.source_pos.x = trackedSourcesGetX(libraryContext_.myTrackedSources, source_index);
                    tracked_source.source_pos.y = trackedSourcesGetY(libraryContext_.myTrackedSources, source_index);
                    tracked_source.source_pos.z = trackedSourcesGetZ(libraryContext_.myTrackedSources, source_index);
                    tracked_source.longitude = atan2f(tracked_source.source_pos.y, tracked_source.source_pos.x) * 180/M_PI;
                    tracked_source.latitude = asinf(tracked_source.source_pos.z/
                                                    (tracked_source.source_pos.z*tracked_source.source_pos.z +
                                                     tracked_source.source_pos.y*tracked_source.source_pos.y +
                                                     tracked_source.source_pos.x*tracked_source.source_pos.x )) * 180/M_PI;

/*

                    ROS_INFO("source index : %i X:%f, Y:%f, Z:%f, id: %i \n",source_index,
						   tracked_source.source_pos.x,
						   tracked_source.source_pos.y,
						   tracked_source.source_pos.z,
						   tracked_source.source_id);
*/


                    if (enable_separation_)
                    {

                        int size = (int)((float)GLOBAL_FRAMESIZE * GLOBAL_OVERLAP);


                        //Fill separation data for the source
                        tracked_source.separation_data.resize(size);



                        postprocessorExtractHop(libraryContext_.myPostprocessorSeparated,
                                                tracked_source.source_id, &tracked_source.separation_data[0]);

                        //Apply GAIN
                        for (int j = 0; j < size; j++)
                        {
                            tracked_source.separation_data[j] *= 50.0;
                        }

                    }






                    //Disappeared in new version of manyearslib ?
                    //tracked_source.source_energy = potentialSourcesGetEnergy(libraryContext_.myPotentialSources, source_index);
                    //Add it to the tracked sources
                    tracked_source_msg.tracked_sources.push_back(tracked_source);
                }

            }

            //if(tracked_source_msg.tracked_sources.size() != 0)
            //{
                //Send the message

                tracked_source_msg.sequence = sequence++;
                pub_.publish(tracked_source_msg);
            //}

            // ??????
            //ros::spinOnce();

            //next frame
            frame_number_++;

        }


        bool get_use_audio_stream()
        {
            return use_audio_stream_;
        }

    private:
        void read_file_for_load_param(std::string& file_param_name)
        {
            ROS_INFO("Loading param file %s...",file_param_name.c_str());
            char current_line[100];
            std::string current_str;
            FILE* file_param_ptr;
            file_param_ptr = fopen(file_param_name.c_str(), "rb");
            if(file_param_ptr == NULL)
                ROS_WARN("Can't open param file");
            else{
                while(fgets(current_line,100,file_param_ptr) != NULL){
                    current_str.assign(current_line);
                    int equal_pos = current_str.find("=");
                    std::string key_str = current_str.substr(0,equal_pos);
                    std::string value_str = current_str.substr(equal_pos+2,current_str.size()-equal_pos-4);
                    std::istringstream stream_val (value_str);
                    float val;
                    stream_val >> val;
                    ROS_DEBUG("key %s, value %s, val %f",key_str.c_str(), value_str.c_str(),val);
                    load_parameter(key_str, val);
                }
                fclose(file_param_ptr);
            }
        }

        void setup_microphone_positions_and_gains(struct ParametersStruct *parametersStruct, struct objMicrophones* myMicrophones)
        {

            // Set the number of microphones
            microphonesInit(myMicrophones, 8);

            // Add microphone 1...
            microphonesAdd(myMicrophones,
                           0,
                           parametersStruct->P_GEO_MICS_MIC1_X,
                           parametersStruct->P_GEO_MICS_MIC1_Y,
                           parametersStruct->P_GEO_MICS_MIC1_Z,
                           parametersStruct->P_GEO_MICS_MIC1_GAIN
                           );

            // Add microphone 2...
            microphonesAdd(myMicrophones,
                           1,
                           parametersStruct->P_GEO_MICS_MIC2_X,
                           parametersStruct->P_GEO_MICS_MIC2_Y,
                           parametersStruct->P_GEO_MICS_MIC2_Z,
                           parametersStruct->P_GEO_MICS_MIC2_GAIN
                           );

            // Add microphone 3...
            microphonesAdd(myMicrophones,
                           2,
                           parametersStruct->P_GEO_MICS_MIC3_X,
                           parametersStruct->P_GEO_MICS_MIC3_Y,
                           parametersStruct->P_GEO_MICS_MIC3_Z,
                           parametersStruct->P_GEO_MICS_MIC3_GAIN
                           );

            // Add microphone 4...
            microphonesAdd(myMicrophones,
                           3,
                           parametersStruct->P_GEO_MICS_MIC4_X,
                           parametersStruct->P_GEO_MICS_MIC4_Y,
                           parametersStruct->P_GEO_MICS_MIC4_Z,
                           parametersStruct->P_GEO_MICS_MIC4_GAIN
                           );

            // Add microphone 5...
            microphonesAdd(myMicrophones,
                           4,
                           parametersStruct->P_GEO_MICS_MIC5_X,
                           parametersStruct->P_GEO_MICS_MIC5_Y,
                           parametersStruct->P_GEO_MICS_MIC5_Z,
                           parametersStruct->P_GEO_MICS_MIC5_GAIN
                           );

            // Add microphone 6...
            microphonesAdd(myMicrophones,
                           5,
                           parametersStruct->P_GEO_MICS_MIC6_X,
                           parametersStruct->P_GEO_MICS_MIC6_Y,
                           parametersStruct->P_GEO_MICS_MIC6_Z,
                           parametersStruct->P_GEO_MICS_MIC6_GAIN
                           );

            // Add microphone 7...
            microphonesAdd(myMicrophones,
                           6,
                           parametersStruct->P_GEO_MICS_MIC7_X,
                           parametersStruct->P_GEO_MICS_MIC7_Y,
                           parametersStruct->P_GEO_MICS_MIC7_Z,
                           parametersStruct->P_GEO_MICS_MIC7_GAIN
                           );

            // Add microphone 8...
            microphonesAdd(myMicrophones,
                           7,
                           parametersStruct->P_GEO_MICS_MIC8_X,
                           parametersStruct->P_GEO_MICS_MIC8_Y,
                           parametersStruct->P_GEO_MICS_MIC8_Z,
                           parametersStruct->P_GEO_MICS_MIC8_GAIN
                           );


        }

        void load_parameter(std::string key, float val)
        {
            if(key.compare("BEAMFORMER_MAXSOURCES") == 0){
                libraryContext_.myParameters->P_BF_MAXSOURCES = (int)val;
                //ROS_INFO("final : %d", libraryContext_.myParameters->P_BF_MAXSOURCES );
            }
            if(key.compare("BEAMFORMER_ET") == 0)
                libraryContext_.myParameters->P_BF_ET = (float)val;
            if(key.compare("BEAMFORMER_FILTERRANGE") == 0)
                libraryContext_.myParameters->P_BF_FILTERRANGE = (float)val;
            if(key.compare("BEAMFORMER_RESETRANGE") == 0)
                libraryContext_.myParameters->P_BF_RESETRANGE = (float)val;
            if(key.compare("GEO_MICS_MIC1_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC1_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC1_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC1_X = (float)val;
            if(key.compare("GEO_MICS_MIC1_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC1_Y = (float)val;
            if(key.compare("GEO_MICS_MIC1_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC1_Z = (float)val;
            if(key.compare("GEO_MICS_MIC2_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC2_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC2_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC2_X = (float)val;
            if(key.compare("GEO_MICS_MIC2_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC2_Y = (float)val;
            if(key.compare("GEO_MICS_MIC2_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC2_Z = (float)val;
            if(key.compare("GEO_MICS_MIC3_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC3_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC3_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC3_X = (float)val;
            if(key.compare("GEO_MICS_MIC3_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC3_Y = (float)val;
            if(key.compare("GEO_MICS_MIC3_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC3_Z = (float)val;
            if(key.compare("GEO_MICS_MIC4_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC4_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC4_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC4_X = (float)val;
            if(key.compare("GEO_MICS_MIC4_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC4_Y = (float)val;
            if(key.compare("GEO_MICS_MIC4_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC4_Z = (float)val;
            if(key.compare("GEO_MICS_MIC5_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC5_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC5_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC5_X = (float)val;
            if(key.compare("GEO_MICS_MIC5_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC5_Y = (float)val;
            if(key.compare("GEO_MICS_MIC5_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC5_Z = (float)val;
            if(key.compare("GEO_MICS_MIC6_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC6_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC6_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC6_X = (float)val;
            if(key.compare("GEO_MICS_MIC6_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC6_Y = (float)val;
            if(key.compare("GEO_MICS_MIC6_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC6_Z = (float)val;
            if(key.compare("GEO_MICS_MIC7_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC7_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC7_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC7_X = (float)val;
            if(key.compare("GEO_MICS_MIC7_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC7_Y = (float)val;
            if(key.compare("GEO_MICS_MIC7_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC7_Z = (float)val;
            if(key.compare("GEO_MICS_MIC8_GAIN") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC8_GAIN = (float)val;
            if(key.compare("GEO_MICS_MIC8_X") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC8_X = (float)val;
            if(key.compare("GEO_MICS_MIC8_Y") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC8_Y = (float)val;
            if(key.compare("GEO_MICS_MIC8_Z") == 0)
                libraryContext_.myParameters->P_GEO_MICS_MIC8_Z = (float)val;

            // Filter
            if(key.compare("FILTER_STANDARDDEVIATION") == 0)
                libraryContext_.myParameters->P_FILTER_STDDEVIATION = (float)val;
            if(key.compare("FILTER_PREDICTION_STOP_ALPHA") == 0)
                libraryContext_.myParameters->P_FILTER_ALPHASTOP = (float)val;
            if(key.compare("FILTER_PREDICTION_STOP_BETA") == 0)
                libraryContext_.myParameters->P_FILTER_BETASTOP = (float)val;
            if(key.compare("FILTER_PREDICTION_CONSTANT_ALPHA") == 0)
                libraryContext_.myParameters->P_FILTER_ALPHACONST = (float)val;
            if(key.compare("FILTER_PREDICTION_CONSTANT_BETA") == 0)
                libraryContext_.myParameters->P_FILTER_BETACONST = (float)val;
            if(key.compare("FILTER_PREDICTION_ACCELERATED_ALPHA") == 0)
                libraryContext_.myParameters->P_FILTER_ALPHAEXC = (float)val;
            if(key.compare("FILTER_PREDICTION_ACCELERATED_BETA") == 0)
                libraryContext_.myParameters->P_FILTER_BETAEXC = (float)val;
            if(key.compare("FILTER_INERTIA_X") == 0)
                libraryContext_.myParameters->P_FILTER_INERTIAX = (float)val;
            if(key.compare("FILTER_INERTIA_Y") == 0)
                libraryContext_.myParameters->P_FILTER_INERTIAY = (float)val;
            if(key.compare("FILTER_INERTIA_Z") == 0)
                libraryContext_.myParameters->P_FILTER_INERTIAZ = (float)val;
            if(key.compare("FILTER_DELTAT") == 0)
                libraryContext_.myParameters->P_FILTER_DELTAT = (float)val;
            if(key.compare("FILTER_STATEUPDATE") == 0)
                libraryContext_.myParameters->P_FILTER_STATEUPDT = (float)val;
            if(key.compare("FILTER_STOP_PERCENTAGE") == 0)
                libraryContext_.myParameters->P_FILTER_NEWSTOP = (float)val;
            if(key.compare("FILTER_CONSTANT_PERCENTAGE") == 0)
                libraryContext_.myParameters->P_FILTER_NEWCONST = (float)val;
            if(key.compare("FILTER_ACCELERATED_PERCENTAGE") == 0)
                libraryContext_.myParameters->P_FILTER_NEWEXC = (float)val;
            if(key.compare("FILTER_ACTIVE_ACTIVE") == 0)
                libraryContext_.myParameters->P_FILTER_AJT_AJTM1 = (float)val;
            if(key.compare("FILTER_INACTIVE_ACTIVE") == 0)
                libraryContext_.myParameters->P_FILTER_AJT_NOTAJTM1 = (float)val;
            if(key.compare("FILTER_P0") == 0)
                libraryContext_.myParameters->P_FILTER_P0 = (float)val;
            if(key.compare("FILTER_RESAMPLING_THRESHOLD") == 0)
                libraryContext_.myParameters->P_FILTER_RSTHRESHOLD = (float)val;
            if(key.compare("FILTER_BUFFERSIZE") == 0)
                libraryContext_.myParameters->P_FILTER_BUFFERSIZE = (int)val;

            // Mixture
            if(key.compare("GEN_DYNSOURCES") == 0)
                libraryContext_.myParameters->P_GEN_DYNSOURCES = (int)val;
            if(key.compare("MIXTURE_PNEW") == 0)
                libraryContext_.myParameters->P_MIXTURE_PNEW = (float)val;
            if(key.compare("MIXTURE_PFALSE") == 0)
                libraryContext_.myParameters->P_MIXTURE_PFALSE = (float)val;
            if(key.compare("MIXTURE_NEWSOURCE_THRESHOLD") == 0)
                libraryContext_.myParameters->P_MIXTURE_NEWTHRESHOLD = (float)val;
            if(key.compare("MIXTURE_CONFIRM_SOURCE_EXISTS") == 0)
                libraryContext_.myParameters->P_MIXTURE_CONFIRMEXISTS = (float)val;
            if(key.compare("MIXTURE_CONFIRM_COUNT_THRESHOLD") == 0)
                libraryContext_.myParameters->P_MIXTURE_CONFIRMCOUNTTS = (float)val;
            if(key.compare("MIXTURE_CONFIRM_COUNT_COUNTER") == 0)
                libraryContext_.myParameters->P_MIXTURE_CONFIRMCOUNT = (int)val;
            if(key.compare("MIXTURE_NEWSOURCE_HORIZONTALANGLE") == 0)
                libraryContext_.myParameters->P_MIXTURE_NEWANGLE = (float)val;
            if(key.compare("MIXTURE_CUMULATIVE_TIME_PROBATION") == 0)
                libraryContext_.myParameters->P_MIXTURE_CUMULATIVETIMEPROB = (int)val;
            if(key.compare("MIXTURE_NOTOBSERVED_PROBATION_THRESHOLD") == 0)
                libraryContext_.myParameters->P_MIXTURE_TOBSPROB = (float)val;
            if(key.compare("MIXTURE_CUMULATIVE_TIME_LEVEL1") == 0)
                libraryContext_.myParameters->P_MIXTURE_CUMULATIVETIME1 = (int)val;
            if(key.compare("MIXTURE_NOTOBSERVED_LEVEL1_THRESHOLD") == 0)
                libraryContext_.myParameters->P_MIXTURE_TOBS1 = (float)val;
            if(key.compare("MIXTURE_CUMULATIVE_TIME_LEVEL2") == 0)
                libraryContext_.myParameters->P_MIXTURE_CUMULATIVETIME2 = (int)val;
            if(key.compare("MIXTURE_NOTOBSERVED_LEVEL2_THRESHOLD") == 0)
                libraryContext_.myParameters->P_MIXTURE_TOBS2 = (float)val;

            // Microphone Sound Track
            if(key.compare("MICST_ALPHAD") == 0)
                libraryContext_.myParameters->P_MICST_ALPHAD = (float)val;
            if(key.compare("MICST_GAMMA") == 0)
                libraryContext_.myParameters->P_MICST_GAMMA = (float)val;
            if(key.compare("MICST_DELTA") == 0)
                libraryContext_.myParameters->P_MICST_DELTA = (float)val;
            if(key.compare("MICST_MCRA_ALPHAS") == 0)
                libraryContext_.myParameters->P_MCRA_ALPHAS = (float)val;
            if(key.compare("MICST_MCRA_ALPHAP") == 0)
                libraryContext_.myParameters->P_MCRA_ALPHAP = (float)val;
            if(key.compare("MICST_MCRA_ALPHAD") == 0)
                libraryContext_.myParameters->P_MCRA_ALPHAD = (float)val;
            if(key.compare("MICST_MCRA_L") == 0)
                libraryContext_.myParameters->P_MCRA_L = (int)val;
            if(key.compare("MICST_MCRA_DELTA") == 0)
                libraryContext_.myParameters->P_MCRA_DELTA = (float)val;

        }

        void audio_stream_cb(const manyears_ros::AudioStreamConstPtr& data_in){

            for (uint i=0; i<data_in->stream_buffer.size() && i<manyears_global::raw_buffer_size_s; i++)
            {
                audio_raw_data_[i] = data_in->stream_buffer[i];
            }
            processing_buffer();
        }

        //ROS variables
        ros::NodeHandle local_nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        //topic_filters::filtered_publisher pub_;
        ros::Time ros_init_time_;
        bool use_audio_stream_;
        bool is_audio_saved_;
        bool enable_separation_;
        std::string frame_id_;
        bool instant_time_;

        //Manyears variables
        int frame_number_;
        short audio_raw_data_[manyears_global::raw_buffer_size_s];
        float audio_float_data_[manyears_global::nb_microphones_s][manyears_global::samples_per_frame_s];
        FILE* filePtr_;
        FILE* output_file_;
        struct objOverall libraryContext_;
        unsigned int sequence;

    };

}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "many_ears");
    ros::NodeHandle n;

    manyears_node::many_ears men(n);

    bool use_callback = men.get_use_audio_stream();

    if(use_callback)
    {
        ROS_INFO("Use stream data");
        ros::spin();
    }
    else
    {
        ROS_INFO("Use file data");
        while(men.fill_buffer_data_with_raw_file() && ros::ok())
        {
            men.processing_buffer();
            ros::spinOnce();
        }


        men.terminate();
        ros::spinOnce();

    }

    return 0;
}

