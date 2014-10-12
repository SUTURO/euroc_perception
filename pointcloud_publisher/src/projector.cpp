#include "suturo_pointcloud_publisher/projector.h"

using namespace suturo_perception;

void CloudProjector::printTransform(const tf::StampedTransform &transform)
{ 
  tf::Vector3 ot = transform.getOrigin();
  tf::Quaternion qt = transform.getRotation();
  ROS_INFO("  translation: [ %f , %f , %f ]", ot[0], ot[1], ot[2]);
  ROS_INFO("  rotation:    [ %f , %f , %f , %f ]", qt.x(), qt.y(), qt.z(), qt.w());
}

bool CloudProjector::getTransform(const ros::NodeHandle &node, const std::string &frame_from, const std::string &frame_to, tf::StampedTransform &transform_) 
{
  tf::TransformListener listener;
  int tries = 0;
  ros::Rate rate(10.0);
  while (node.ok() && tries < 10)
  {
    try
    {
      listener.waitForTransform(frame_from, frame_to, /*frame_rgb, frame,*/ ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(frame_from, frame_to, /*frame_rgb, frame,*/ ros::Time(0), transform_);
      return true;
    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
    tries++;
  }
  return false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudProjector::depthProject(const cv::Mat &depth_image_in, const cv::Mat &rgb_image, const tf::StampedTransform &transform, const bool projectColors)
{
  cv::Mat depth_image;
  if (depth_image_in.type() == CV_16U)
   depth_image_in.convertTo(depth_image, CV_32F, 0.001, 0.0);
  else
   depth_image = depth_image_in;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->height = depth_image.rows;
  cloud->width = depth_image.cols;
  cloud->is_dense = true;
  cloud->points.resize(cloud->height * cloud->width);
  register const float bad_point = std::numeric_limits<float>::quiet_NaN();
  register const int
      centerX = (cloud->width >> 1),
      centerY = (cloud->height >> 1);

  double fov_v = 0.817109355f; // TODO: get this from yaml!
  double fov_h = 1.047f; // TODO: get this from yaml!
  double h1 = tan(fov_h/2);
  double v1 = tan(fov_v/2);

#pragma omp parallel for
  for (int y = 0; y < depth_image.rows; ++y)
  {
    pcl::PointXYZRGB *pPt = &cloud->points[y * depth_image.cols];

    const float *pDepth = depth_image.ptr<float>(y);

    for (register int u = 0; u < centerX*2; ++u, ++pPt, ++pDepth)
    {
      float depth = *pDepth;
      
      // Check for invalid measurements
      if (isnan(depth) || depth == 0)
      {
        // not valid
        pPt->x = pPt->y = pPt->z = bad_point;
        continue;
      }
     
      // calculate point
      pPt->z = depth;
      pPt->y = -depth * (v1 - (2*v1 *( y/480.0) )); // TODO: get this from yaml!
      pPt->x = -depth * (h1 - (2*h1 *( u/640.0) )); // TODO: get this from yaml!

      // calculate color pixel
			if (!projectColors)
			{
				continue;
			}
      tf::Stamped<tf::Point> p1, p2;
      
      p1.m_floats[0] = pPt->x;
      p1.m_floats[1] = pPt->y;
      p1.m_floats[2] = pPt->z;

      p2.setData(transform * p1);
     
      int pixx = (( 640.0 * ( p2.m_floats[0] * h1 - p2.m_floats[1] ) ) / ( 2.0 * p2.m_floats[0] * h1 )); // TODO: get this from yaml!
      int pixy = ( 480.0 * ( p2.m_floats[0] * v1 - p2.m_floats[2] ) ) / ( 2.0 * p2.m_floats[0] * v1 ); // TODO: get this from yaml!
      
      if (pixx < 0 || pixx > 640 || pixy < 0 || pixy > 480) // TODO: get this from yaml!
      {
        // not valid
        pPt->x = pPt->y = pPt->z = bad_point;
        continue;
      }
      else
      {
        cv::Vec3b bgrPixel = rgb_image.at<cv::Vec3b>(pixy, pixx);
        pPt->r = bgrPixel.val[2];
        pPt->g = bgrPixel.val[1];
        pPt->b = bgrPixel.val[0];
      }
    }
  }
  return cloud;
}


ROI getCloudROI(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const tf::StampedTransform &transform)
{
  ROI retroi;
  retroi.x = -1;
  retroi.y = -1;
  retroi.w = -1;
  retroi.h = -1;
  
  ROI tmproi;
  tmproi.x = 640;
  tmproi.y = 480;
  tmproi.w = 0;
  
  
  double fov_v = 0.817109355f; // TODO: get this from yaml!
  double fov_h = 1.047f; // TODO: get this from yaml!
  double h1 = tan(fov_h/2);
  double v1 = tan(fov_v/2);
  
#pragma omp parallel for
  for (int i = 0; i < cloud_in->points.size(); i++)
  {
    tf::Stamped<tf::Point> p1, p2;
    p1.m_floats[0] = cloud_in->points[i].x;
    p1.m_floats[0] = cloud_in->points[i].y;
    p1.m_floats[0] = cloud_in->points[i].z;
    p2.setData(transform * p1);
    int pixx = (( 640.0 * ( p2.m_floats[0] * h1 - p2.m_floats[1] ) ) / ( 2.0 * p2.m_floats[0] * h1 )); // TODO: get this from yaml!
    int pixy = ( 480.0 * ( p2.m_floats[0] * v1 - p2.m_floats[2] ) ) / ( 2.0 * p2.m_floats[0] * v1 ); // TODO: get this from yaml!
    //if (pixx < tmproi
  }
}
