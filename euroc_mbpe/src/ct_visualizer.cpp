#include <pcl/visualization/pcl_visualizer.h>
#include "perception_utils/logger.h"
#include "euroc_mbpe/correspondence_transform.h"

typedef Eigen::Matrix< float, 6, 1 > 	Vector6f;
typedef Eigen::Matrix< float, 12, 1 > 	Vector12f;

int main(int argc, const char *argv[])
{
  CorrespondenceTransform ct;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_correspondences (new pcl::PointCloud<pcl::PointXYZ>);
  // Init the model correspondences. Let's take some of the corners of a box
  pcl::PointXYZ c;
  c.x = -0.3;
  c.y = 0.3;
  c.z = -0.3; 
  model_correspondences->push_back(c);
  c.x = -0.3;
  c.y = 0.3;
  c.z = 0.3; 
  model_correspondences->push_back(c);
  c.x = -0.3;
  c.y = -0.3;
  c.z = 0.3; 
  model_correspondences->push_back(c);
  c.x = 0.3;
  c.y = -0.3;
  c.z = 0.3; 
  model_correspondences->push_back(c);

  // Transform the given points and think of these new points as the observed ones
  pcl::PointCloud<pcl::PointXYZ>::Ptr observed_correspondences (new pcl::PointCloud<pcl::PointXYZ>);
  Vector6f d; // (theta_x, theta_y, theta_z, x, y, z)
  d[0] = d[1] = d[2] = d[3] = d[4] = d[5] = 0;
  d[2] = M_PI/4; // Rotate pi/4 around the z-axis
  d[3] = 0.5; // Translation on the x axis
  Eigen::Matrix4f transform_1 = ct.getRotationMatrixFromPose(d);
  pcl::transformPointCloud(*model_correspondences, *observed_correspondences, transform_1 );

  ct.setCorrespondences(model_correspondences, observed_correspondences);

  // Start time measuring ...
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  ct.execute();
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  std::cout << "Final pose estimation from lib:" << std::endl << ct.getEstimatedPose() << std::endl;
  std::cout << "Done in " << ct.getIterationCount() << " iterations" << std::endl;
  Eigen::Matrix< float, 6, 1 > x = ct.getEstimatedPose();
  suturo_perception::Logger logger("correspondence_transform");
  logger.logTime(s, end, "pose estimation");

  // Start the visualizer
  pcl::visualization::PCLVisualizer p ("Pose Estimation test");
  p.addCoordinateSystem (0.3);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(observed_correspondences, 0, 255, 0);
  p.addPointCloud<pcl::PointXYZ> (model_correspondences, "model_correspondences");
  p.addPointCloud<pcl::PointXYZ> (observed_correspondences, green_color, "observed_correspondences");
  p.addText("White pts = model correspondences, Green pts = observed correspondences, Green box = The model fitted to the observed points, Yellow pts = line between correspondences", 5, 10 , "caption");

  // Create a model of the box as a Polygon
  pcl::PointCloud<pcl::PointXYZ>::Ptr box_model = ct.generateBoxModelFromPose(x);
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[0], box_model->points[1], 0.0, 1.0, 0.0,"line1");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[1], box_model->points[3], 0.0, 1.0, 0.0,"line2");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[1], box_model->points[5], 0.0, 1.0, 0.0,"line3");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[0], box_model->points[2], 0.0, 1.0, 0.0,"line4");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[0], box_model->points[4], 0.0, 1.0, 0.0,"line5");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[2], box_model->points[3], 0.0, 1.0, 0.0,"line6");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[4], box_model->points[5], 0.0, 1.0, 0.0,"line7");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[6], box_model->points[7], 0.0, 1.0, 0.0,"line8");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[6], box_model->points[2], 0.0, 1.0, 0.0,"line9");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[6], box_model->points[4], 0.0, 1.0, 0.0,"line10");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[7], box_model->points[3], 0.0, 1.0, 0.0,"line11");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[7], box_model->points[5], 0.0, 1.0, 0.0,"line12");

  // Add Correspondences
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[0], observed_correspondences->points[0], 1.0, 1.0, 0.0,"corrline1");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[1], observed_correspondences->points[1], 1.0, 1.0, 0.0,"corrline2");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[2], observed_correspondences->points[2], 1.0, 1.0, 0.0,"corrline3");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[3], observed_correspondences->points[3], 1.0, 1.0, 0.0,"corrline4");

  p.spin();
  
  return 0;
}
