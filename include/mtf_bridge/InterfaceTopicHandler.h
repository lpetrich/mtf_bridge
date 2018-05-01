#ifndef INTERFACETOPICHANDLER_H
#define INTERFACETOPICHANDLER_H value
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "mtf_bridge/BufferInit.h"

// Shared memory buffer
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class InterfaceTopicHandler {
public:
    InterfaceTopicHandler ();
	bool isInitialized() { return initialized; };
    bool checkReset() { return reset; };

	cv::Mat* getFrame();
	int getHeight() { return height; };
    int getWidth() { return width; };
	int getFrameID() { return frame_id; };
    std::string getMsg() { return msg; };

private:
    bool initialized;
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
    std::string msg;
    boost::interprocess::shared_memory_object* shm;
    boost::interprocess::mapped_region* region;
    cv::Mat **frame_buffer;

    ros::Subscriber init_buffer_sub;
    ros::Subscriber image_index_sub;
    ros::Subscriber task_coord_sub;
    ros::Subscriber reset_sub;

    void update_image_properties(mtf_bridge::BufferInitConstPtr buffer_init);
    void update_image_index(std_msgs::UInt32ConstPtr index);
    void initialize_shm();
    void update_task_coords(std_msgs::StringConstPtr msg_data);
    void reset_trackers(std_msgs::BoolConstPtr reset_all);

};
#endif /* ifndef INTERFACETOPICHANDLER_H */
