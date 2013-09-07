/*
 * main.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "soc_msg_and_serv/segment_and_classify.h"

class SOCDemo
{
  private:
    int kinect_trials_;
    int service_calls_;
    ros::NodeHandle n_;
    std::string camera_topic_;
    bool KINECT_OK_;

    void
    checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      KINECT_OK_ = true;
    }

    void
    checkKinect ()
    {
      ros::Subscriber sub_pc = n_.subscribe (camera_topic_, 1, &SOCDemo::checkCloudArrive, this);
      ros::Rate loop_rate (1);
      kinect_trials_ = 0;
      while (!KINECT_OK_ && ros::ok ())
      {
        std::cout << "Checking kinect status..." << std::endl;
        ros::spinOnce ();
        loop_rate.sleep ();
        kinect_trials_++;
        if(kinect_trials_ >= 5)
        {
          std::cout << "Kinect is not working..." << std::endl;
          return;
        }
      }

      KINECT_OK_ = true;
      std::cout << "Kinect is up and running" << std::endl;
    }

    void
    callService (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      if( (service_calls_ % (30 * 5)) == 0)
      {
        std::cout << "going to call service..." << std::endl;
        ros::ServiceClient client = n_.serviceClient<soc_msg_and_serv::segment_and_classify>("segment_and_classify");
        soc_msg_and_serv::segment_and_classify srv;
        srv.request.cloud = *msg;
        if (client.call(srv))
        {
          std::cout << "Found categories:" << static_cast<int>(srv.response.categories_found.size()) << std::endl;
          for(size_t i=0; i < srv.response.categories_found.size(); i++)
          {
            std::cout << "   => " << srv.response.categories_found[i] << std::endl;
          }
        }
      }
      service_calls_++;
    }

  public:
    SOCDemo()
    {
      KINECT_OK_ = false;
      camera_topic_ = "/camera/depth/points";
      kinect_trials_ = 5;
    }

    bool initialize(int argc, char ** argv)
    {
      checkKinect();
      return KINECT_OK_;
    }

    void run()
    {
      ros::Subscriber sub_pc = n_.subscribe (camera_topic_, 1, &SOCDemo::callService, this);
      ros::spin();
      /*ros::Rate loop_rate (5);
      while (ros::ok () && (service_calls_ < 5)) //only calls 5 times
      {
        ros::spinOnce ();
        loop_rate.sleep ();
      }*/
    }
};

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "SOC_demo");

  SOCDemo m;
  m.initialize (argc, argv);
  m.run();
  return 0;
}
