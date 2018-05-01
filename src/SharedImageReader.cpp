#include "mtf_bridge/SharedImageReader.h"

SharedImageReader::SharedImageReader() : initialized(false), frame_id(0){
	ros::NodeHandle nh_("~");
	// Read in parameters and subscribe

	std::string init_topic;
	std::string image_index_topic;
	nh_.param<std::string>("shm_name", shm_name, "SharedBuffer");
	nh_.param<std::string>("init_topic", init_topic, "/init_buffer");
	nh_.param<std::string>("image_index_topic", image_index_topic, "/input_image");

	init_buffer_sub = nh_.subscribe(init_topic, 1, &SharedImageReader::update_image_properties, this);
	image_index_sub = nh_.subscribe(image_index_topic, 1, &SharedImageReader::update_image_index, this);

	std::cout << "MTF-SIR: Read Param shm_name: " << shm_name << "\n";
	std::cout << "MTF-SIR: Read Param init_topic: " << init_topic << "\n";
	std::cout << "MTF-SIR: Read Param image_index_topic: " << image_index_topic << "\n";
	std::cout << "MTF-SIR: Shared image reader initialized.\n";
}

void SharedImageReader::update_image_index(std_msgs::UInt32ConstPtr index) {
	buffer_id = index->data;
	++frame_id;
}

void SharedImageReader::update_image_properties(mtf_bridge::BufferInitConstPtr buffer_init) {
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
	std::cout << "*******************************\n";
	std::cout << "MTF: Image properties:\n"
		"\theight: " << height << "\n"
		"\twidth: " << width << "\n"
		"\tchannels: " << channels << "\n"
		"\tbuffer_count: " << buffer_count << "\n"
		"\tframe_size: " << frame_size << "\n"
		"\tbuffer_id: " << buffer_id << "\n";
	std::cout << "*******************************\n";
	initialize_shm();
	init_buffer_sub.shutdown();
	std::cout << "MTF-SIR: SHM initialized.\n";
}

void SharedImageReader::initialize_shm() {
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

cv::Mat* SharedImageReader::getFrame() {
	return frame_buffer[buffer_id];
}
