cmake_minimum_required(VERSION 2.8.3)
project(mtf_bridge)

# option(WITH_TEMPLATED "Enable templated implementations of search methods (faster at runtime but take much longer to compile)" ON)
option(WITH_THIRD_PARTY "Enable Third Party Trackers" ON)

# set(MTF_RUNTIME_FLAGS -Wfatal-errors -Wno-write-strings -O3 -DDISABLE_XVISION -DDISABLE_PFSL3 -DDISABLE_GOTURN -DDISABLE_GRID_FEAT -DDISABLE_REGNET -DDISABLE_DFM) 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/mtfConfig.cmake")

# if(NOT WITH_TEMPLATED)
# message(STATUS "Templated implementations of SMs are disabled")
# set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DENABLE_ONLY_NT  -DDISABLE_FLANN)
# else()
# find_package(FLANN)
# find_package(HDF5)
# if(FLANN_FOUND AND HDF5_FOUND) 
# set(MTF_LIBS ${MTF_LIBS} ${FLANN_LIBS} ${HDF5_LIBRARIES})
# else()
# message(STATUS "FLANN/HDF5 library not found so the templated version NN search method has been disabled")
# set(MTF_RUNTIME_FLAGS ${MTF_RUNTIME_FLAGS} -DDISABLE_FLANN)
# endif()
# endif()

if(NOT WITH_THIRD_PARTY)
	message(STATUS "Third party trackers disabled")
	set(MTF_DEFINITIONS ${MTF_DEFINITIONS} -DDISABLE_THIRD_PARTY_TRACKERS)
else()
	option(WITH_VISP "Enable ViSP template tracker and pipeline" ON)
	if(NOT WITH_VISP)
		message(STATUS "ViSP disabled")
		set(MTF_DEFINITIONS ${MTF_DEFINITIONS} -DDISABLE_VISP)
	endif()
	option(WITH_DFT "Enable Descriptor Fields tracker (third party) " ON)
	if(NOT WITH_DFT)
		message(STATUS "DFT disabled")
		set(MTF_DEFINITIONS ${MTF_DEFINITIONS} -DDISABLE_DFT)
	endif()
	option(WITH_MIL "Enable Multiple Instance Learning based tracker (third party)" ON)
	if(NOT WITH_MIL)
		message(STATUS "MIL tracker disabled")
		set(MTF_DEFINITIONS ${MTF_DEFINITIONS} -DDISABLE_MIL)
	endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  cmake_modules
)

# find_package(MTF REQUIRED)
# find_package(Eigen REQUIRED)
# find_package(Boost REQUIRED COMPONENTS random filesystem system)

# set(MTF_LIBS ${MTF_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Patch.msg
  PatchTrackers.msg
  Point.msg
  BufferInit.msg
  #Error.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   InitTracker.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs  
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES mtf_bridge
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
)

###########
## Build ##
###########

# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${MTF_CXX_FLAGS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MTF_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include
  /usr/local/include  
)

link_directories(
  /usr/lib
  /usr/local/lib
  /usr/X11R6/lib64
  /usr/X11R6/lib
  /lib/
)
# ADD LIBRARIES ##################################################
#### SharedImageReader ####
add_library(SharedImageReader 
  src/SharedImageReader.cpp
)
add_dependencies(SharedImageReader 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(SharedImageReader
  ${catkin_LIBRARIES}
)

#### InterfaceTopicHandler ####
add_library(InterfaceTopicHandler 
  src/InterfaceTopicHandler.cpp
)
add_dependencies(InterfaceTopicHandler 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(InterfaceTopicHandler
  ${catkin_LIBRARIES}
)
#### UVSTopicHandler ####
add_library(UVSTopicHandler 
  src/UVSTopicHandler.cpp
)
add_dependencies(UVSTopicHandler 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(UVSTopicHandler
  ${catkin_LIBRARIES}
)
# ADD EXECUTABLES ##################################################
#### SharedImageWriter ####
add_executable(SharedImageWriter 
  src/SharedImageWriter.cpp
)
add_dependencies(SharedImageWriter 
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(SharedImageWriter
  ${catkin_LIBRARIES}
  rt
)

#### sample_tracker_node ####
add_executable(sample_tracker_node 
  src/sample_tracker_node.cpp
)
target_compile_definitions(sample_tracker_node PUBLIC 
  ${MTF_DEFINITIONS}
)
target_compile_options(sample_tracker_node PUBLIC 
  ${MTF_RUNTIME_FLAGS}
)
target_link_libraries(sample_tracker_node
  SharedImageReader
  ${catkin_LIBRARIES}
  ${MTF_LIBRARIES}
)
add_dependencies(sample_tracker_node 
  ${PROJECT_NAME}_generate_messages_cpp
)
# set_target_properties(sample_tracker_node PROPERTIES COMPILE_FLAGS ${MTF_RUNTIME_FLAGS}) 

#### interface_tracker_node ####
add_executable(interface_tracker_node 
  src/interface_tracker_node.cpp
)
target_compile_definitions(interface_tracker_node PUBLIC 
  ${MTF_DEFINITIONS}
)
target_compile_options(interface_tracker_node PUBLIC 
  ${MTF_RUNTIME_FLAGS}
)
target_link_libraries(interface_tracker_node
  InterfaceTopicHandler
  SharedImageReader
  ${catkin_LIBRARIES}
  ${MTF_LIBRARIES}
)
add_dependencies(interface_tracker_node 
  ${PROJECT_NAME}_generate_messages_cpp
)

#### vs_tracker_node ####
add_executable(vs_tracker_node 
  src/vs_tracker_node.cpp
)
target_compile_definitions(vs_tracker_node PUBLIC 
  ${MTF_DEFINITIONS}
)
target_compile_options(vs_tracker_node PUBLIC 
  ${MTF_RUNTIME_FLAGS}
)
target_link_libraries(vs_tracker_node
  InterfaceTopicHandler
  SharedImageReader
  ${catkin_LIBRARIES}
  ${MTF_LIBRARIES}
)

add_dependencies(vs_tracker_node 
  ${PROJECT_NAME}_generate_messages_cpp
)

#### uvs_trackers ####
add_executable(uvs_trackers 
  src/uvs_trackers.cpp
)
target_compile_definitions(uvs_trackers PUBLIC 
  ${MTF_DEFINITIONS}
)
target_compile_options(uvs_trackers PUBLIC 
  ${MTF_RUNTIME_FLAGS}
)
target_link_libraries(uvs_trackers
  UVSTopicHandler
  SharedImageReader
  ${catkin_LIBRARIES}
  ${MTF_LIBRARIES}
)
add_dependencies(uvs_trackers 
  ${PROJECT_NAME}_generate_messages_cpp
)

#### testing ####
add_executable(testing 
  src/some_tracker_node.cpp
)
target_compile_definitions(testing PUBLIC 
  ${MTF_DEFINITIONS}
)
target_compile_options(testing PUBLIC 
  ${MTF_RUNTIME_FLAGS}
)
target_link_libraries(testing
  SharedImageReader
  ${catkin_LIBRARIES}
  ${MTF_LIBRARIES}
)
add_dependencies(testing 
  ${PROJECT_NAME}_generate_messages_cpp
)

################################################################

#add_dependencies(SharedImageReader ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(InterfaceTopicHandler ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(SharedImageWriter ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(sample_tracker_node ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(interface_tracker_node ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(vs_tracker_node ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

message(STATUS "MTF_DEFINITIONS: ${MTF_DEFINITIONS}")
# message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "MTF_RUNTIME_FLAGS: ${MTF_RUNTIME_FLAGS}")
message(STATUS "MTF_INCLUDE_DIRS: ${MTF_INCLUDE_DIRS}")
message(STATUS "MTF_LIBRARIES: ${MTF_LIBRARIES}")
