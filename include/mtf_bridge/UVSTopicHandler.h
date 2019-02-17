#ifndef UVSTOPICHANDLER_H
#define UVSTOPICHANDLER_H
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <mtf_bridge/InitTracker.h>
#include "mtf_bridge/BufferInit.h"

// Shared memory buffer
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class UVSTopicHandler {
public:
    std::string msg;
    std::string am;
    std::string sm;
    std::string ssm;
    std::vector<double> corners{0, 0, 0, 0};
    UVSTopicHandler();
	bool isInitialized() { return initialized; };
    bool checkReset() { return reset; };
    bool newTracker() { return new_tracker; };
	cv::Mat* getFrame();
	int getHeight() { return height; };
    int getWidth() { return width; };
	int getFrameID() { return frame_id; };
    std::string getSM() { return sm; };
    std::string getAM() { return am; };
    std::string getSSM() { return ssm; };
    std::vector<double> getNewCorners() { return corners; };
    void clearSrv();
    void doneReset();

private:
    bool initialized;
    bool new_tracker;
    bool reset;
    int height;
    int width;
    int channels;
    int buffer_count;
    int frame_size;
    int frame_id;
    int buffer_id;

    uchar** shared_mem_addrs;
    std::string shm_name;
    boost::interprocess::shared_memory_object* shm;
    boost::interprocess::mapped_region* region;
    cv::Mat **frame_buffer;

    ros::Subscriber init_buffer_sub;
    ros::Subscriber image_index_sub;
    // ros::Subscriber task_coord_sub;
    ros::Subscriber reset_sub;
    ros::ServiceServer init_tracker_service;
    bool callback_initialize_tracker(mtf_bridge::InitTracker::Request &req, mtf_bridge::InitTracker::Response &res);
    void callback_image_properties(mtf_bridge::BufferInitConstPtr buffer_init);
    void callback_image_index(std_msgs::UInt32ConstPtr index);
    // void callback_corner_coordinates(std_msgs::StringConstPtr msg_data);
    void callback_reset(std_msgs::BoolConstPtr reset_all);
    void initialize_shm();
};
#endif /* ifndef UVSTOPICHANDLER_H */
