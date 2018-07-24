#  - Basic config file generator for MTF
# It defines the following variables
#  MTF_INCLUDE_DIRS - include directories for MTF
#  MTF_RUNTIME_FLAGS - Compile flags for GCC
#  MTF_LIBRARIES    - libraries to link against
#  MTF_DEFINITIONS - preprocessor definitions for MTF
#  MTF_VERSION - current MTF version
#  MTF_LIB_DIRS - directory where MTF compiled lib is installed
 
# Compute paths
set(MTF_INCLUDE_DIRS "/usr/local/include /usr/include/opencv;/usr/include;/usr/local/include/eigen3;/usr/include;/usr/include;/usr/include")
set(MTF_RUNTIME_FLAGS "-Wfatal-errors;-Wno-write-strings;-Wno-unused-result;-Wformat=0;-std=c++11;-O3;-msse2")
set(MTF_LIBRARIES "mtf;opencv_videostab;opencv_video;opencv_ts;opencv_superres;opencv_stitching;opencv_photo;opencv_ocl;opencv_objdetect;opencv_nonfree;opencv_ml;opencv_legacy;opencv_imgproc;opencv_highgui;opencv_gpu;opencv_flann;opencv_features2d;opencv_core;opencv_contrib;opencv_calib3d;/usr/lib/x86_64-linux-gnu/libboost_random.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;cmt;opentld;cvblobs;struck;mil;frg;/usr/lib/x86_64-linux-gnu/libhdf5.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libdl.so;/usr/lib/x86_64-linux-gnu/libm.so")
set(MTF_DEFINITIONS "NDEBUG;EIGEN_NO_DEBUG;DISABLE_DFT;DISABLE_PFSL3;DISABLE_VISP;DISABLE_CV3;DISABLE_GOTURN;DISABLE_XVISION;DISABLE_DFM;DISABLE_TEMPLATED_SM;DISABLE_REGNET;DISABLE_SPI")
set(MTF_VERSION "1.0")
set(MTF_LIB_DIRS "/usr/local/lib")
