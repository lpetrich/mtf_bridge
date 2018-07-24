#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include "mtf_bridge/InterfaceTopicHandler.h"
#include "mtf_bridge/PatchTrackers.h"
#include "visual_servoing/TrackedPoints.h"
#include "visual_servoing/TrackPoint.h"

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
std::string ids;
std::vector<int> task_ids;
std::vector<std::vector<double>> task_coords;
int num_tasks;
double tracker_size;
cv::Point2d static_tracker[4];
std::vector<cv::Point> static_tracker_centers;
cv::Point static_center;

ros::Publisher patch_pub;
ros::Publisher center_pub;
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
  	for (int i = 0; i < 4; i++) {
  		corners << cv_corners[i].x << " " << cv_corners[i].y << " ";
  	}
    std::string str_corners = corners.str();
	return str_corners;
}

std::string getCenter(TrackerStruct &tracker) {
	cv::Point2d cv_corners[4];
	mtf::utils::Corners(tracker.getRegion()).points(cv_corners);
  	std::stringstream ss;
	cv::Point center_point = getPatchCenter(cv_corners);
	ss << center_point.x << " " << center_point.y << " ";
    std::string str_center = ss.str();
	return str_center;
}

visual_servoing::TrackPoint getCenterPoint(TrackerStruct &tracker) {
	cv::Point2d cv_corners[4];
	mtf::utils::Corners(tracker.getRegion()).points(cv_corners);
	cv::Point center_point = getPatchCenter(cv_corners);
	visual_servoing::TrackPoint p;
	p.x = center_point.x;
	p.y = center_point.y;
	return p;
}

void updateTrackers() {
	if (trackers.empty()) {
		return;
	}
	if (topic_handler->checkReset()) {
		std::cout << "MTF: Resetting.\n";
        if (trackers.size() > 0) {
            trackers.erase(trackers.begin(), trackers.end());
            pre_procs.erase(pre_procs.begin(), pre_procs.end());
        }
		// trackers.clear();
		// pre_procs.clear();
		num_tasks = 0;
		tracker_id = 0;
		topic_handler->doneReset();
	} else {
	    std::string patch_msg = "";
	    visual_servoing::TrackedPoints center_msg;
		for(std::vector<TrackerStruct>::iterator tracker = trackers.begin(); 
			tracker != trackers.end(); ++tracker) {
			(*tracker).update(*(topic_handler->getFrame()), topic_handler->getFrameID());
			std::string patch = getPatch(*tracker);
			// std::string center = getCenter(*tracker);
			visual_servoing::TrackPoint p = getCenterPoint(*tracker);
			patch_msg += patch;
			center_msg.points.push_back(p);
		}
		patch_pub.publish(patch_msg);
		center_pub.publish(center_msg);
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

void createMat(const std::vector<double> v) {
    cv::Mat cv_corners(2, 4, CV_64FC1);
    int j = 0;
	for (std::vector<double>::size_type i = 0; i != v.size(); i+=2) {
		cv_corners.at<double>(0, j) = v[i];
		cv_corners.at<double>(1, j) = v[i+1];
		++j;
	}	
	if(!createTracker(*(topic_handler->getFrame()), cv_corners, topic_handler->getFrameID())) {
		std::cout << "MTF: Tracker could not be created.\n";
	}
	std::cout << "MTF: Dynamic tracker initialized.\n";
}

void initializeTrackers(std::string clicked_points) {
	std::string s = ";";
	std::string::size_type sz;
	std::vector<double> t;
	int i;
	double d;
	istringstream iss(clicked_points);
	do {
		std::string subs;
		iss >> subs;
		i = iss.peek();
		if (i != -1) {
			if (subs == s){
				createMat(t);
				t.clear();
			} else {
				d = std::stod (subs, &sz);
				t.push_back(d);
			}
		}
	} while (iss);
}

void trackerCheck() {
	std::string clicked_points = topic_handler->getMsg();
	if (clicked_points.size() > 0) { 
		topic_handler->clearMsg();
		initializeTrackers(clicked_points);
		std::cout << "MTF: All tasks initialized.\n";
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
	std::cout << "MTF: Publishing to ~/trackers/patch_tracker\n";
	std::cout << "MTF: Publishing to ~/trackers/centers\n";
	patch_pub = nh_.advertise<std_msgs::String>("patch_tracker", 1);
	center_pub = nh_.advertise<visual_servoing::TrackedPoints>("centers", 1);
	ros::Rate loop_rate(rate);

	while(!topic_handler->isInitialized()) {
		ros::spinOnce();
		ros::Duration(0.7).sleep();
	}

	while(ros::ok()){
		ros::spinOnce();
		trackerCheck();
		updateTrackers();
		loop_rate.sleep();
		// if (!loop_rate.sleep())
		// {
		// 	// std::cout << "WARNING: Publishing loop in vs_tracker_node.cpp (line 322) is too slow for given " << rate << " Hz rate." << std::endl;
		// }
	}
	return 0;
}
