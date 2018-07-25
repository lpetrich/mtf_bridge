#include "mtf_bridge/InterfaceTopicHandler.h"

InterfaceTopicHandler::InterfaceTopicHandler() : initialized(false), frame_id(0){
	ros::NodeHandle nh_("~");
	msg = "";
	reset = false;
	// Read in parameters and subscribe
	nh_.param<std::string>("shm_name", shm_name, "SharedBuffer");
	std::string init_topic;
	std::string image_index_topic;
	std::string task_coord_topic;

	nh_.param<std::string>("init_topic", init_topic, "/init_buffer");
	nh_.param<std::string>("image_index_topic", image_index_topic, "/input_image");
	nh_.param<std::string>("task_coord_topic", task_coord_topic, "/task_coordinates");

	init_buffer_sub = nh_.subscribe(init_topic, 1, &InterfaceTopicHandler::callback_image_properties, this);	
	image_index_sub = nh_.subscribe(image_index_topic, 1, &InterfaceTopicHandler::callback_image_index, this);
	task_coord_sub = nh_.subscribe(task_coord_topic, 1, &InterfaceTopicHandler::callback_task_coords, this);
	reset_sub = nh_.subscribe("/reset", 1, &InterfaceTopicHandler::callback_reset, this);

	std::cout << "MTF: Initializing " << shm_name << "\n";
	std::cout << "MTF: Subscribing to ~" << init_topic << "\n";
	std::cout << "MTF: Subscribing to ~" << image_index_topic << "\n";
	std::cout << "MTF: Subscribing to ~" << task_coord_topic << "\n";
	std::cout << "MTF: Subscribing to /reset\n";
	std::cout << "MTF: Topic handler initialized.\n";
}

void InterfaceTopicHandler::clearMsg() {
	msg.clear();
}

void InterfaceTopicHandler::doneReset() {
	reset = false;
}

void InterfaceTopicHandler::callback_task_coords(std_msgs::StringConstPtr msg_data) {
	std::cout << "New coordinates received";
	msg = msg_data->data;
}

void InterfaceTopicHandler::callback_reset(std_msgs::BoolConstPtr reset_all) {
	reset = reset_all->data;
	std::cout << "MTF: reset received" << std::endl;
	msg.clear();
}

void InterfaceTopicHandler::callback_image_index(std_msgs::UInt32ConstPtr index) {
	buffer_id = index->data;
	++frame_id;
}

void InterfaceTopicHandler::callback_image_properties(mtf_bridge::BufferInitConstPtr buffer_init) {
	if(initialized)  {
		return;
	}
	initialized = true;
	height = buffer_init->height;
	width = buffer_init->width;
	channels = buffer_init->channels;
	buffer_count = buffer_init->buffer_count;
	frame_size = buffer_init->frame_size;
	buffer_id = buffer_init->init_id;
	frame_id = 0;
	std::cout << "MTF: Image properties:\n"
		"\theight: " << height << "\n"
		"\twidth: " << width << "\n"
		"\tchannels: " << channels << "\n"
		"\tbuffer_count: " << buffer_count << "\n"
		"\tframe_size: " << frame_size << "\n"
		"\tbuffer_id: " << buffer_id << "\n";
	initialize_shm();
	init_buffer_sub.shutdown();
	std::cout << "MTF: Node initialized.\n\n";
}

void InterfaceTopicHandler::initialize_shm() {
	// Opens read only shared region of memmory
	shm = new boost::interprocess::shared_memory_object(boost::interprocess::open_only,
		shm_name.c_str(),
		boost::interprocess::read_write);
	region = new boost::interprocess::mapped_region(*shm, boost::interprocess::read_only);
	uchar* start_addr = static_cast<uchar*>(region->get_address());

	// Create Mat objects for each of the frames in the buffer
	frame_buffer = new cv::Mat*[buffer_count];
	shared_mem_addrs = new uchar*[buffer_count];
	for(int i = 0; i < buffer_count; ++i){
		frame_buffer[i] = new cv::Mat(height, width, CV_8UC3);
		shared_mem_addrs[i] = start_addr + (i * frame_size * sizeof(uchar));
		// Set the data pointer to be located in the correct place of the smh.
		frame_buffer[i]->data = shared_mem_addrs[i];
	}
}

cv::Mat* InterfaceTopicHandler::getFrame() {
	return frame_buffer[buffer_id];
}
