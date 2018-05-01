#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include "mtf_bridge/InterfaceTopicHandler.h"
#include "mtf_bridge/PatchTrackers.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// C++
#include <vector>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <cstring>
#include <Eigen/Core>

// Modular Tracking Framework
#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/objUtils.h"

#include "std_msgs/String.h"

#include <sstream>

using namespace mtf;
using namespace mtf::params;
using namespace mtf::utils;

typedef std::shared_ptr<mtf::TrackerBase> Tracker_;
// neded to avoid duplicate preprocessors
std::vector<PreProc_> pre_procs;

int const rate = 30;

using namespace mtf::params;

struct TrackerStruct{
	TrackerStruct(Tracker_ &_tracker, PreProc_ &_pre_proc, int _id) :
		tracker(_tracker), pre_proc(_pre_proc), id(_id){
	}
	bool update(const cv::Mat &frame, int frame_id = -1) {
		try{
			//! update pre-processor
			pre_proc->update(frame, frame_id);
			//! update tracker
			tracker->update();
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while updating the tracker: %s\n",
				err.type(), err.what());
			return false;
		}
		return true;
	}
	void setRegion(const cv::Mat& corners){
		tracker->setRegion(corners);
	}
	const cv::Mat& getRegion() {
		return tracker->getRegion();
	}
	int getID() const{ return id; }
private:
	Tracker_ tracker;
	PreProc_ pre_proc;
	int id;
};


cv::Mat display_frame;
std::vector<TrackerStruct> trackers;
int tracker_id;
std::vector<cv::Scalar> colors;
int n_colors;
bool trackers_initialized;
std::string msg;
std::string ids;
std::vector<int> task_ids;
std::vector<std::vector<double>> task_coords;
int num_tasks;
double tracker_size;
cv::Point2d static_tracker[4];
std::vector<cv::Point> static_tracker_centers;
cv::Point static_center;

ros::Publisher tracker_pub;
InterfaceTopicHandler *topic_handler;

cv::Point getPatchCenter(const cv::Point2d (&cv_corners)[4]) {
	Eigen::Vector3d tl(cv_corners[0].x, cv_corners[0].y, 1);
	Eigen::Vector3d tr(cv_corners[1].x, cv_corners[1].y, 1);
	Eigen::Vector3d br(cv_corners[2].x, cv_corners[2].y, 1);
	Eigen::Vector3d bl(cv_corners[3].x, cv_corners[3].y, 1);

	Eigen::Vector3d center_vec = center_vec = tl.cross(br).cross(tr.cross(bl));

	cv::Point center;
	center.x = center_vec(0) / center_vec(2);
	center.y = center_vec(1) / center_vec(2);
	return center;
}

std::string getPatch(TrackerStruct &tracker) {
	// [top_left, top_right, bot_right, bot_left]
	cv::Point2d cv_corners[4];
	mtf::utils::Corners(tracker.getRegion()).points(cv_corners);
  	
  	std::stringstream corners;
  	// for (int i = 0; i < 4; i++) {
  	// 	corners << cv_corners[i].x << " " << cv_corners[i].y << " ";
  	// }
	cv::Point center_point = getPatchCenter(cv_corners);
	corners << center_point.x << " " << center_point.y << " ";

    std::string str_corners = corners.str();
	return str_corners;
}

void updateTrackers() {
	if (trackers.empty()) {
		return;
	}
	bool r = topic_handler->checkReset();
	if (topic_handler->checkReset()) {
		std::cout << "MTF: Deleting trackers.\n";
		trackers.clear();
		pre_procs.clear();
		num_tasks = 0;
		tracker_id = 0;
		trackers_initialized = false;
	} else {
		std::stringstream corners;
	    std::string tracker_msg = corners.str();
		for(std::vector<TrackerStruct>::iterator tracker = trackers.begin(); 
			tracker != trackers.end(); ++tracker) {
			(*tracker).update(*(topic_handler->getFrame()), topic_handler->getFrameID());
			std::string tracker_patch = getPatch(*tracker);
			tracker_msg += tracker_patch;
		}
		tracker_pub.publish(tracker_msg);
	}
}

void drawPatch(const cv::Point2d* corners, cv::Scalar color) {
	line(display_frame, corners[0], corners[1], color);
	line(display_frame, corners[1], corners[2], color);
	line(display_frame, corners[2], corners[3], color);
	line(display_frame, corners[3], corners[0], color);
}

void drawFrame(std::string cv_window_title) {
	cv::cvtColor(*(topic_handler->getFrame()), display_frame, cv::COLOR_RGB2BGR);
	// Draw trackers
	if (!trackers.empty()) {
		for(std::vector<TrackerStruct>::iterator tracker = trackers.begin();
			tracker != trackers.end(); ++tracker) {
			cv::Point2d cv_corners[4];
			mtf::utils::Corners((*tracker).getRegion()).points(cv_corners);
			cv::Scalar obj_col = colors[(*tracker).getID() % n_colors];
			drawPatch(cv_corners, obj_col);
			cv::Point center = getPatchCenter(cv_corners);
			cv::circle(display_frame, center, 5, obj_col, -1);
			cv::circle(display_frame, center, 5, cv::Scalar(0, 0, 0), 2);
		}
	}
	// // Draw stationary trackers
	// if (num_tasks != 0) {
	// 	for(std::vector<cv::Point>::iterator it = static_tracker_centers.begin(); it != static_tracker_centers.end(); ++it) {
	// 		cv::circle(display_frame, *it, 5, (255,255,255), -1);
	// 	}
	// }
	imshow(cv_window_title, display_frame);
	char key = cv::waitKey(10);
	if (key == 'd') {
		if (trackers.size() > 0) {
			static_tracker_centers.clear();
			trackers.erase(trackers.begin());
			pre_procs.erase(pre_procs.begin());
			num_tasks = 0;
			trackers_initialized = false;
		}
	}
}

bool createTracker(const cv::Mat init_frame, const cv::Mat init_corners, int frame_id) {
	Tracker_ tracker;
	PreProc_ pre_proc;
	try{
		tracker.reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
		if(!tracker){
			printf("Tracker could not be created successfully\n");
			return false;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while creating the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		pre_proc = mtf::getPreProc(pre_procs, tracker->inputType(), pre_proc_type);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while creating the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		pre_proc->initialize(init_frame, frame_id);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		for(PreProc_ curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
			tracker->setImage(curr_obj->getFrame());
		}
		tracker->initialize(init_corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return false;
	}

	trackers.push_back(TrackerStruct(tracker, pre_proc, tracker_id));
	pre_procs.push_back(pre_proc);
	++tracker_id;
	return true;
}
// points are ordered: 1 ---- 2
//                     |      |
//                     4 ---- 3
void createDynamicTracker(const double x_center, double y_center) {
	if(!readParams(0, nullptr)){ return; }

	double x_left = x_center - tracker_size;
	double x_right = x_center + tracker_size;
	double y_bottom = y_center + tracker_size;
	double y_top = y_center - tracker_size;

	cv::Mat cv_corners(2, 4, CV_64FC1); 
	cv_corners.at<double>(0, 0) = x_left;
	cv_corners.at<double>(1, 0) = y_top;
	cv_corners.at<double>(0, 1) = x_right;
	cv_corners.at<double>(1, 1) = y_top;
	cv_corners.at<double>(0, 2) = x_right;
	cv_corners.at<double>(1, 2) = y_bottom;
	cv_corners.at<double>(0, 3) = x_left;
	cv_corners.at<double>(1, 3) = y_bottom;

	// std::cout << "xl: " << x_left << " xr: " << x_right << " ytop: " << y_top << " ybot: " << y_bottom << "\n";

	if(!createTracker(*(topic_handler->getFrame()), cv_corners, topic_handler->getFrameID())) {
		std::cout << "MTF: Tracker could not be created.\n";
	}
	std::cout << "MTF: Dynamic tracker initialized.\n";
}

void handleTaskTrackers(const std::vector<double> v) {
	double x;
	double y;
	for (std::vector<double>::size_type i = 0; i != v.size(); i++) {
		if (i % 2 == 0) {
			x = v[i];
		} else {
			y = v[i];
			createDynamicTracker(x, y);
		}
	}
}

void initializeTrackers(std::string msg) {
	std::string s = ";";
	std::string::size_type sz;
	std::vector<double> temp;
	int i;
	double d;
	istringstream iss(msg);
	do {
		std::string subs;
		iss >> subs;
		i = iss.peek();
		if (i != -1) {
			if (subs == s){
				handleTaskTrackers(temp);
				temp.clear();
			} else {
				d = std::stod (subs, &sz);
				temp.push_back(d);
			}
		}
	} while (iss);
	// drawFrame("TrackingNode");
}

void trackerCheck() {
	msg = topic_handler->getMsg();
	if (!trackers_initialized) {
		if (msg.size() > 0) { 
			initializeTrackers(msg);
			trackers_initialized = true;
			std::cout << "MTF: All tasks initialized.\n";
		} else {
			ros::spinOnce();
			ros::Duration(0.7).sleep();	
		}
	}
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "tracking_node");
	ros::NodeHandle nh_("~");
	image_transport::ImageTransport it(nh_);

	// Initialize input_obj
	config_dir = ros::package::getPath("mtf_bridge") + "/cfg";
	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

	printf("*******************************\n");
	printf("MTF: Using parameters:\n");
	printf("\tn_trackers: %u\n", n_trackers);
	printf("\timg_source: %c\n", img_source);
	printf("\tsource_name: %s\n", seq_name.c_str());
	printf("\tpipeline: %c\n", pipeline);
	printf("\tmtf_sm: %s\n", mtf_sm);
	printf("\tmtf_am: %s\n", mtf_am);
	printf("\tmtf_ssm: %s\n", mtf_ssm);
	printf("\tmtf_ilm: %s\n", mtf_ilm);
	printf("*******************************\n");

	ObjUtils(obj_cols).getCols(colors);
	n_colors = colors.size();

	topic_handler = new InterfaceTopicHandler();
	tracker_id = 0;
	n_trackers = 0;
	tracker_size = 30.0;
	num_tasks = 0;
	trackers_initialized = false;
	// Initialize OpenCV window 
	std::string cv_window_title = "TrackingNode";
	cv::namedWindow(cv_window_title, cv::WINDOW_AUTOSIZE);
	std::cout << "MTF: Publishing to ~/trackers/patch_tracker\n";
	tracker_pub = nh_.advertise<std_msgs::String>("patch_tracker", 1);
	ros::Rate loop_rate(rate);

	while(!topic_handler->isInitialized()) {
		ros::spinOnce();
		ros::Duration(0.7).sleep();
	}

	while(ros::ok()){
		ros::spinOnce();
		trackerCheck();
		if (trackers_initialized) {
			updateTrackers();
			// drawFrame(cv_window_title);
		} 
		loop_rate.sleep();
	}
	return 0;
}
