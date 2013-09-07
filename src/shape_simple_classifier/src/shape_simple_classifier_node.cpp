/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/vfh_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/cvfh_estimator.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include "soc_msg_and_serv/segment_and_classify.h"
#include "segmenter.h"
#include <pcl/apps/dominant_plane_segmentation.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#define SOC_VISUALIZE

class ShapeClassifier
{
  private:
    typedef pcl::PointXYZ PointT;
    std::string models_dir_;
    std::string training_dir_;
    std::string desc_name_;
    int NN_;
    float chop_at_z_;
    std::vector<std::string> text_3d_;

    boost::shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, PointT, pcl::ESFSignature640> > classifier_;
    ros::ServiceServer segment_and_classify_service_;
    ros::NodeHandle n_;
#ifdef SOC_VISUALIZE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
#endif

    bool segmentAndClassify(soc_msg_and_serv::segment_and_classify::Request & req,
                               soc_msg_and_serv::segment_and_classify::Response & response)
    {
      pcl::PointCloud<PointT>::Ptr frame (new pcl::PointCloud<PointT>);
      pcl::fromROSMsg (req.cloud, *frame);

      /*std::vector<pcl::PointIndices> indices;
      Eigen::Vector4f table_plane;
      doSegmentation<PointT>(frame, indices, table_plane, chop_at_z_);*/

      float tolerance = 0.005f;
      pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
      dps.setInputCloud (frame);
      dps.setMaxZBounds (chop_at_z_);
      dps.setObjectMinHeight (tolerance);
      dps.setMinClusterSize (1000);
      dps.setWSize (9);
      dps.setDistanceBetweenClusters (0.1f);
      std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
      std::vector<pcl::PointIndices> indices;
      dps.setDownsamplingSize (0.01f);
      dps.compute_fast (clusters);
      dps.getIndicesClusters (indices);
      Eigen::Vector4f table_plane;
      dps.getTableCoefficients (table_plane);

#ifdef SOC_VISUALIZE
      for(size_t i=0; i < text_3d_.size(); i++)
      {
        vis_->removeText3D(text_3d_[i]);
      }
      text_3d_.clear();
      vis_->removeAllShapes();
      vis_->removeAllPointClouds();
      vis_->addPointCloud(frame, "scene_cloud");
      vis_->addCoordinateSystem(0.2f);

      //show table plane
      std::vector<int> plane_indices;
      for(size_t i=0; i < frame->points.size(); i++)
      {
        Eigen::Vector3f xyz_p = frame->points[i].getVector3fMap ();

        if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
          continue;

        float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

        if (val <= tolerance && val >= -tolerance)
        {
          plane_indices.push_back(i);
        }
      }

      pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*frame, plane_indices, *plane);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> random_handler (plane, 0, 255, 0);
      vis_->addPointCloud<PointT> (plane, random_handler, "table plane");
      vis_->spinOnce();
#endif

      classifier_->setInputCloud(frame);

      for(size_t i=0; i < indices.size(); i++)
      {

#ifdef SOC_VISUALIZE
        pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*frame, indices[i], *cluster);
        std::stringstream cluster_name;
        cluster_name << "cluster_" << i;
        pcl::visualization::PointCloudColorHandlerRandom<PointT> random_handler (cluster);
        vis_->addPointCloud<PointT> (cluster, random_handler, cluster_name.str ());
#endif
        classifier_->setIndices(indices[i].indices);
        classifier_->classify ();

        std::vector < std::string > categories;
        std::vector<float> conf;
        classifier_->getCategory (categories);
        classifier_->getConfidence (conf);

#ifdef SOC_VISUALIZE
        float text_scale = 0.015f;
        float dist_ = 0.03f;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*frame, indices[i].indices, centroid);
        for (size_t kk = 0; kk < categories.size (); kk++)
        {
          pcl::PointXYZ pos;
          pos.x = centroid[0] + table_plane[0] * static_cast<float> (kk + 1) * dist_;
          pos.y = centroid[1] + table_plane[1] * static_cast<float> (kk + 1) * dist_;
          pos.z = centroid[2] + table_plane[2] * static_cast<float> (kk + 1) * dist_;

          std::ostringstream prob_str;
          prob_str.precision (2);
          prob_str << categories[kk] << " [" << conf[kk] << "]";

          std::stringstream cluster_text;
          cluster_text << "cluster_" << i << "_" << kk << "_text";
          text_3d_.push_back(cluster_text.str());
          vis_->addText3D (prob_str.str (), pos, text_scale, 1, 0, 1, cluster_text.str (), 0);
        }
#endif

        if(categories.size() == 1)
        {
          std_msgs::String ss;
          ss.data = categories[0];
          response.categories_found.push_back(ss);
        }
        else if(categories.size() == 0)
        {
          //weird case, do nothing...
        }
        else
        {
          //at least 2 categories
          std::vector< std::pair<float, std::string> > conf_categories_map_;
          for (size_t kk = 0; kk < categories.size (); kk++)
          {
            conf_categories_map_.push_back(std::make_pair(conf[kk], categories[kk]));
          }

          std::sort (conf_categories_map_.begin (), conf_categories_map_.end (),
                     boost::bind (&std::pair<float, std::string>::first, _1) > boost::bind (&std::pair<float, std::string>::first, _2));

          /*for (size_t kk = 0; kk < categories.size (); kk++)
          {
            std::cout << conf_categories_map_[kk].first << std::endl;
          }*/

          if( (conf_categories_map_[1].first / conf_categories_map_[0].first) < 0.85f)
          {


            if (!boost::starts_with(conf_categories_map_[0].second, "unknown"))
            {
              std_msgs::String ss;
              ss.data = conf_categories_map_[0].second;
              response.categories_found.push_back(ss);
            }
          }
        }
      }

#ifdef SOC_VISUALIZE
      vis_->spin();
#endif

      return true;
    }
  public:
    ShapeClassifier()
    {
      //default values
      desc_name_ = "esf";
      NN_ = 50;
      chop_at_z_ = 1.f;
#ifdef SOC_VISUALIZE
      vis_.reset(new pcl::visualization::PCLVisualizer("classifier visualization"));
#endif
    }

    void initialize(int argc, char ** argv)
    {

      pcl::console::parse_argument (argc, argv, "-models_dir", models_dir_);
      pcl::console::parse_argument (argc, argv, "-training_dir", training_dir_);
      pcl::console::parse_argument (argc, argv, "-descriptor_name", desc_name_);
      pcl::console::parse_argument (argc, argv, "-nn", NN_);
      pcl::console::parse_argument (argc, argv, "-chop_z", chop_at_z_);

      if(models_dir_.compare("") == 0)
      {
        PCL_ERROR("Set -models_dir option in the command line, ABORTING");
        return;
      }

      if(training_dir_.compare("") == 0)
      {
        PCL_ERROR("Set -training_dir option in the command line, ABORTING");
        return;
      }

      boost::shared_ptr<pcl::rec_3d_framework::MeshSource<PointT> > mesh_source (new pcl::rec_3d_framework::MeshSource<PointT>);
      mesh_source->setPath (models_dir_);
      mesh_source->setResolution (150);
      mesh_source->setTesselationLevel (0);
      mesh_source->setViewAngle (57.f);
      mesh_source->setRadiusSphere (1.f);
      mesh_source->setModelScale (1.f);
      mesh_source->generate (training_dir_);

      boost::shared_ptr<pcl::rec_3d_framework::Source<PointT> > cast_source;
      cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<PointT> > (mesh_source);

      boost::shared_ptr<pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640> > estimator;
      estimator.reset (new pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640>);

      boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<PointT, pcl::ESFSignature640> > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640> > (estimator);

      classifier_.reset(new pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, PointT, pcl::ESFSignature640>);
      classifier_->setDataSource (cast_source);
      classifier_->setTrainingDir (training_dir_);
      classifier_->setDescriptorName (desc_name_);
      classifier_->setFeatureEstimator (cast_estimator);
      classifier_->setNN (NN_);
      classifier_->initialize (false);

      segment_and_classify_service_ = n_.advertiseService("segment_and_classify", &ShapeClassifier::segmentAndClassify, this);

      ros::spin();
    }
};

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "master_demo");

  ShapeClassifier m;
  m.initialize (argc, argv);

  return 0;
}
