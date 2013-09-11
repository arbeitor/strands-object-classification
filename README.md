strands-object-classification
=============================

A object recognition framework based on learned object classes.


Prerequisites
-------------

* [PCL](https://github.com/PointCloudLibrary/pcl)


Installation of PCL 
-------------------

1. Get the PCL source code:
    
        git clone https://github.com/PointCloudLibrary/pcl pcl-trunk

    
2. Configure and compile PCL:

   <strong>NOTE</strong>: when compiling PCL please make sure that "opt/ros/groovy/setup.bash" is <strong>NOT</strong> sourced (see http://www.pcl-users.org/PCL-1-7-Compilation-Problems-td4028868.html)
    
        cd pcl-trunk && mkdir build && cd build
        ccmake ..
        
    In the interactive menu of ccmake hit `c` for configure. Then enable the variable `BUILD_apps` (by setting it to `ON`) and hit `c` for configure. Afterward set the varibale `BUILD_app_3d_rec_framework` to `ON` and configure again (hit `c`). Finally, hit `g` for generate and exit.

    Back on the command line type:

        make
        
    The installation of PCL is optional, as we can build the ROS package against the library in PCL's `build` directory:
    
        sudo make install 


Installation of this package 
-----------------------------

1. Get the code:

        git clone https://github.com/arbeitor/strands-object-classification 
        
2. Compile the code using catkin (Please note: it will fail! Just continue with the next step):

        cd strands-object-classification
        catkin_make --force-cmake -DPCL_DIR=/path/tp/pcl/build
        
3. As the compilation above fails, configure now two missing variables: `PCL_APP_3D_REC_FRAMEWORK_INCLUDE_DIR` and `PCL_APP_3D_REC_FRAMEWORK_LIBRARY`. Do this as follows:

        cd build
        ccmake ../src -DCATKIN_DEVEL_PREFIX=../devel -DPCL_DIR=/path/to/pcl/build
        
   Within the interactive menu of `ccmake` set the variables:
   
   `PCL_APP_3D_REC_FRAMEWORK_INCLUDE_DIR` to `/path/to/pcl/pcl/apps/3d_rec_framework/include` and 
   
   `PCL_APP_3D_REC_FRAMEWORK_LIBRARY` to `/path/to/pcl/build/lib/FOO`
   
4. Compile again:

        cd ..
        catkin_make --force-cmake -DPCL_DIR=/path/tp/pcl/build
        



Getting Started
---------------

Get a models archive and unzip somewhere on your disk. 

Start some terminals and run the commands below:



1. Fire up roscore:
   
        $ roscore

2. Start the recognition service:

        $ rosrun shape_simple_classifier shape_simple_classifier_node -models_dir /path/to/models/data/ -training_dir /path/to/models/trained/ -nn 10

3. Plug-in the kinect and start openni:

        $ roslaunch openni_launch openni.launch

4. Run a client to test the service:

        $ rosrun soc_test soc_test_node
