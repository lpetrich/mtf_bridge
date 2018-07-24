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
    std::string msg;
    InterfaceTopicHandler ();
	bool isInitialized() { return initialized; };
    bool checkReset() { return reset; };
	cv::Mat* getFrame();
	int getHeight() { return height; };
    int getWidth() { return width; };
	int getFrameID() { return frame_id; };
    std::string getMsg() { return msg; };
    void clearMsg();
    void doneReset();

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
    boost::interprocess::shared_memory_object* shm;
    boost::interprocess::mapped_region* region;
    cv::Mat **frame_buffer;

    ros::Subscriber init_buffer_sub;
    ros::Subscriber image_index_sub;
    ros::Subscriber task_coord_sub;
    ros::Subscriber reset_sub;

    void callback_image_properties(mtf_bridge::BufferInitConstPtr buffer_init);
    void callback_image_index(std_msgs::UInt32ConstPtr index);
    void callback_task_coords(std_msgs::StringConstPtr msg_data);
    void callback_reset(std_msgs::BoolConstPtr reset_all);
    void initialize_shm();
};
#endif /* ifndef INTERFACETOPICHANDLER_H */
