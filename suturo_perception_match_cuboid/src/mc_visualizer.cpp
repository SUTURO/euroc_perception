#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/distances.h>
// #include <point_cloud_operations.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/format.hpp>
#include <suturo_perception_match_cuboid/detected_plane.h>
#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <perception_utils/cuboid.hpp>
#include <pcl/ModelCoefficients.h>

#define MIN_ANGLE 5 // the minimum angle offset between to norm vectors
                    // if this threshold is not reached, no rotation will be made on this axis

void printDuration(boost::posix_time::ptime s, boost::posix_time::ptime e, std::string text)
{
    boost::posix_time::time_duration d = e - s;
    float diff = (float)d.total_microseconds() / (float)1000;
    std::cout << ((boost::format("Time for %s: %s ms") % text % diff).str()) << std::endl;
}

pcl::PointXYZ getPointXYZFromVector3f(Eigen::Vector3f vec)
{
  return pcl::PointXYZ( vec[0], vec[1],vec[2]);
}

pcl::PointXYZ getPointXYZFromVector4f(Eigen::Vector4f vec)
{
  return pcl::PointXYZ( vec[0], vec[1],vec[2]);
}

pcl::PointXYZ getPointXYZFromModelCoefficients(pcl::ModelCoefficients::Ptr mc)
{
  return pcl::PointXYZ( mc->values.at(0), mc->values.at(1) , mc->values.at(2) );
}

Eigen::Vector3f getVector3fFromModelCoefficients(pcl::ModelCoefficients::Ptr mc)
{
  Eigen::Vector3f ret(
      mc->values.at(0),
      mc->values.at(1),
      mc->values.at(2)
      );
  return ret;
}

// Cut off the fourth component
Eigen::Vector3f getVector3fFromVector4f(Eigen::Vector4f vec)
{
  Eigen::Vector3f ret(
      vec(0),
      vec(1),
      vec(2)
      );
  return ret;
}

// The corners MUST be in the order which is defined in computeCuboidCornersWithMinMax3D!
// Otherwise this method will not work
// The viewport is the related to your open viewports in the PCLVisualizer instance
// If you have only one viewport, you can pass 0 there or leave it empty
void drawBoundingBoxLines(pcl::visualization::PCLVisualizer &visualizer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points, int viewport=0)
{
  if( corner_points->points.size() != 8)
  {
    std::cerr << "The corner pointcloud should contain 8 elements. Actual size: " << corner_points->points.size() << std::endl;
    return;
  }
  // Front face after the transformation
  visualizer.addLine(corner_points->points.at(0), corner_points->points.at(2), "bb_line_1",viewport);
  visualizer.addLine(corner_points->points.at(0), corner_points->points.at(3), "bb_line_2",viewport);
  visualizer.addLine(corner_points->points.at(6), corner_points->points.at(2), "bb_line_3",viewport);
  visualizer.addLine(corner_points->points.at(6), corner_points->points.at(3), "bb_line_4",viewport);

  // Back face after the transformation
  visualizer.addLine(corner_points->points.at(4), corner_points->points.at(7), "bb_line_5",viewport);
  visualizer.addLine(corner_points->points.at(4), corner_points->points.at(5), "bb_line_6",viewport);
  visualizer.addLine(corner_points->points.at(1), corner_points->points.at(7), "bb_line_7",viewport);
  visualizer.addLine(corner_points->points.at(1), corner_points->points.at(5), "bb_line_8",viewport);

  // Connect both faces with each other
  visualizer.addLine(corner_points->points.at(0), corner_points->points.at(4), "bb_line_9",viewport);
  visualizer.addLine(corner_points->points.at(2), corner_points->points.at(7), "bb_line_10",viewport);
  visualizer.addLine(corner_points->points.at(3), corner_points->points.at(5), "bb_line_11",viewport);
  visualizer.addLine(corner_points->points.at(6), corner_points->points.at(1), "bb_line_12",viewport);

  // Draw two diagonal lines to see where the center should be
  visualizer.addLine(corner_points->points.at(0), corner_points->points.at(1), 255,255,0, "bb_diag_1",viewport);
  visualizer.addLine(corner_points->points.at(2), corner_points->points.at(5), 255,255,0, "bb_diag_2",viewport);

  // Draw the centroid of the object
  Eigen::Vector4f centroid;
  // CuboidMatcher::computeCentroid(corner_points, centroid);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::ConvexHull<pcl::PointXYZRGB> hull;
  hull.setInputCloud(corner_points);
  hull.setDimension(3);
  hull.reconstruct (*hull_points);
  // Centroid calulcation
  pcl::compute3DCentroid (*hull_points, centroid);  
  visualizer.addSphere(getPointXYZFromVector4f(centroid), 0.01, "centroid_bb", viewport);

}

// Translate a pointcloud by x,y and z
void translatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float x, float y, float z, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
    // Use the identity matrix as a start, since we don't do any rotations
    Eigen::Matrix< float, 4, 4 > translationMatrix =  Eigen::Matrix< float, 4, 4 >::Identity();;
    
    // Translation vector
    translationMatrix(0,3) = x;
    translationMatrix(1,3) = y;
    translationMatrix(2,3) = z;

    pcl::transformPointCloud (*cloud_in, *cloud_out, translationMatrix);   
}

// Make a translation of the given Vector by subtracting
// vec1 and vec2 and use this as translation parameters
Eigen::Vector3f moveVectorBySubtraction(Eigen::Vector3f input, Eigen::Vector3f vec1, Eigen::Vector3f vec2)
{
  Eigen::Translation3f tNorm = Eigen::Translation3f(
      vec1[0] - vec2[0],
      vec1[1] - vec2[1],
      vec1[2] - vec2[2]
      );
  Eigen::Vector3f newDest;
  newDest = tNorm * input;
  return newDest;
}

Eigen::Matrix< float, 3, 3 > removeTranslationVectorFromMatrix(Eigen::Matrix<float,4,4> m)
{
  Eigen::Matrix< float, 3, 3 > result;
  result.setZero();
  result(0,0) = m(0,0);
  result(1,0) = m(1,0);
  result(2,0) = m(2,0);

  result(0,1) = m(0,1);
  result(1,1) = m(1,1);
  result(2,1) = m(2,1);

  result(0,2) = m(0,2);
  result(1,2) = m(1,2);
  result(2,2) = m(2,2);
  return result;
}
int
main (int argc, char** argv)
{
  bool second_rotation_performed = false;
  bool visualize_results = true;

  boost::posix_time::ptime t_s = boost::posix_time::microsec_clock::local_time();

  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 1)
  {
    std::cout << "Usage: input_file_path.pcd\n";
    exit (-1);
  }
  std::string input_filename = argv[filenames.at(0)];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_filename, *original_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }
  std::cout << "Loaded "
            << original_cloud->width * original_cloud->height
            << " data points from input pcd" << std::endl;

  boost::posix_time::ptime t_file_loaded = boost::posix_time::microsec_clock::local_time();
  // Copy the original cloud to input cloud, which can be modified later during plane extraction
  pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*original_cloud, *input_cloud);

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(true);
  cm.setSaveIntermediateResults(true);
  /*
   * Use this code if you want to use the algorithm
   * with given table coefficients. The table coefficients
   * should describe the plane, where the input cloud is
   * standing on.
   *
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values.at(0) = 0.000624452;
  coefficients->values.at(1) = -0.796438;
  coefficients->values.at(2) = -0.60472;
  coefficients->values.at(3) = 0.915019;
  cm.setTableCoefficients(coefficients);
  cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
  */


  Cuboid cuboid;
  cm.execute(cuboid);
  boost::posix_time::ptime t_algorithm_done = boost::posix_time::microsec_clock::local_time();
  printDuration(t_file_loaded, t_algorithm_done, "Algorithm Runtime");

  std::cout << "Algorithm done. Visualize results" << std::endl;

  
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> intermediate_clouds;
  intermediate_clouds = cm.getIntermediateClouds();

  if(!visualize_results)
    exit(0);
  // Atleast one intermediate clouds means, that the user want's to display them here.
  
  std::cout << "The algorithm did " << intermediate_clouds.size() << " transformation(s)" << std::endl;

  // Set up a list of viewports
  std::vector<int> viewports;
  for (int i = 0; i < 2 + intermediate_clouds.size(); i++)
  {
    viewports.push_back(i);
  }

  double viewport_count = viewports.size();
  double items_per_row = 3;
  double visualizer_row_count = ( (viewports.size() -1) / items_per_row ) +1 ;
  
  // Create the viewports for the different objects
  pcl::visualization::PCLVisualizer viewer;
  int c = 0;
  for( int row = visualizer_row_count -1; row >= 0; row--)
  {
    double y_0 = row    *(1/visualizer_row_count);
    double y_1 = (row+1)*(1/visualizer_row_count);
    for( int column = 0; column < items_per_row; column++)
    {
      double x_0 =  column    *(1/items_per_row);
      double x_1 =  (column+1)*(1/items_per_row);
      if(c < viewport_count)
      {
        // std::cout << x_0 << " " << y_0 << " ";
        // std::cout << x_1 << " " << y_1 << " @" << viewports.at(c) <<std::endl;
        viewer.createViewPort(x_0, y_0, x_1, y_1, viewports.at(c) );
        viewer.addCoordinateSystem(0.7,viewports.at(c));
      }
      c++;
    }

  }

  // Visualize original pointcloud
  const int v_input  = 0;
  const int v_result = viewport_count -1;
  const int v_last_transform = viewport_count -2;
  if(v_last_transform<=0)
  {
    std::cerr << "Not enough transformations to visualize" << std::endl;
    return 0;
  }

  viewer.addText("Input Cloud", 0.1, 0.1 , "input_cloud_id", viewports.at(v_input));
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_pts (original_cloud, 255,0,0);
  viewer.addPointCloud<pcl::PointXYZRGB> (original_cloud, red_pts, "input_cloud", viewports.at(v_input) );

  // Visualize result
  viewer.addText("Matched Cuboid", 0.1, 0.1 , "result_cuboid", viewports.at(v_result));
  viewer.addPointCloud<pcl::PointXYZRGB> (original_cloud, red_pts, "input_cloud_w_result", viewports.at(v_result) );
  // Draw the bounding box from the given corner points
  drawBoundingBoxLines(viewer, cuboid.corner_points, viewports.at(v_result));

  viewer.addCube	(	cuboid.center,
      cuboid.orientation,
      cuboid.length1,
      cuboid.length2,
      cuboid.length3,
      "matched_cuboid_centered",
      viewports.at(v_result)
      );

  // viewer.addCube	(	Eigen::Vector3f(0,0,0),
  //     Eigen::Quaternion<float>::Identity(),
  //     cuboid.length1,
  //     cuboid.length2,
  //     cuboid.length3,
  //     "matched_cuboid",
  //     v2
  //   );


  // Visualize the found inliers.
  //
  // Skip the viewports for the input and the result
  // in the iteration
  int cloud_idx = 0;
  for (int i = v_input +1 ; i < v_result; i++)
  {
    // Translate the clouds
    // to the camera's origin
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_at_origin(new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Vector4f centroid;
    CuboidMatcher::computeCentroid(intermediate_clouds.at(cloud_idx), centroid);
    translatePointCloud(intermediate_clouds.at(cloud_idx), -centroid[0], -centroid[1], -centroid[2], cloud_at_origin);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (intermediate_clouds.at(cloud_idx), 255,0,0);

    std::stringstream ss;
    ss << "rotated_cloud_" << cloud_idx;
    std::string cloud_id = ss.str();
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud_at_origin, red, cloud_id,
       viewports.at(i) );
    cloud_idx++;
  }

  // Draw the corner points, that have been computed with pcl::getMinMax3D
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bb_corners(new pcl::PointCloud<pcl::PointXYZRGB>);
  cm.computeCuboidCornersWithMinMax3D(intermediate_clouds.at ( intermediate_clouds.size()-1 ) ,bb_corners);

  // Draw the bounding box edge points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bb_at_origin(new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::Vector4f centroid;
  CuboidMatcher::computeCentroid(intermediate_clouds.at ( intermediate_clouds.size()-1 ), centroid);
  translatePointCloud(bb_corners, -centroid[0], -centroid[1], -centroid[2], bb_at_origin);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white_pts_bb (bb_corners, 255,255,255);
  viewer.addPointCloud<pcl::PointXYZRGB> (bb_at_origin, white_pts_bb, "bb", viewports.at(v_last_transform) );
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "bb");

  // std::vector<DetectedPlane> *detected_planes = cm.getDetectedPlanes();
  // for (int i = 0; i < detected_planes->size(); i++)
  // {
  //   for (int j = 0; j < detected_planes->size(); j++)
  //   {
  //     if(i==j) continue;
  //     detected_planes->at(i).angleBetween(
  //         detected_planes->at(j).getCoefficientsAsVector3f() );
  //   }
  // }
  // detected_planes->at(0);
  viewer.spin();




  /*
  boost::posix_time::ptime t_extraction_done = boost::posix_time::microsec_clock::local_time();
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //
  // Store the possible different colors for the segmented planes
  std::vector<std::vector<int> > color_sequence;
  std::vector<int> green;
  green.push_back(0);
  green.push_back(255);
  green.push_back(0);
  std::vector<int> red;
  red.push_back(255);
  red.push_back(0);
  red.push_back(0);
  std::vector<int> blue;
  blue.push_back(0);
  blue.push_back(0);
  blue.push_back(255);
  color_sequence.push_back(green);
  color_sequence.push_back(red);
  color_sequence.push_back(blue);
  
  pcl::visualization::PCLVisualizer viewer;
  // Visualize original pointcloud
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 0.2, 1.0, v1);
  viewer.addCoordinateSystem(1.0,v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(original_cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (original_cloud, rgb, "sample cloud1", v1);

  // Visualize the found inliers
  int v2(1);
  viewer.createViewPort(0.2, 0.0, 0.6, 1.0, v2);
  viewer.addCoordinateSystem(1.0,v2);

  int v3(2);
  viewer.createViewPort(0.6, 0.0, 1.0, 1.0, v3);
  viewer.addCoordinateSystem(0.5,v3);

  // For each segmented plane
  for (int i = 0; i < detected_planes.size(); i++) 
  {
    // Select a color from the color_sequence vector for the discovered plane
    int color_idx = i % 3;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (detected_planes.at(i).getPoints(), color_sequence.at(color_idx).at(0), color_sequence.at(color_idx).at(1), color_sequence.at(color_idx).at(2));

    std::stringstream cloud_name("plane_cloud_");
    cloud_name << i;
    std::stringstream plane_name("model_plane_");
    plane_name << i;
    viewer.addPointCloud<pcl::PointXYZRGB> (detected_planes.at(i).getPoints(), single_color, cloud_name.str(), v2);
    viewer.addPlane (*detected_planes.at(i).getCoefficients(), plane_name.str(), v2);

    // Display the centroid of the planes
    pcl::PointXYZRGB c;
    c.x = detected_planes.at(i).getCentroid()(0);
    c.y = detected_planes.at(i).getCentroid()(1);
    c.z = detected_planes.at(i).getCentroid()(2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    centroid_cloud->push_back(c);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> centroid_yellow (centroid_cloud, 
        255,255,0);
    std::stringstream centroid_name("centroid_");
    centroid_name << i;
    viewer.addPointCloud<pcl::PointXYZRGB> (centroid_cloud, centroid_yellow, centroid_name.str(), v2);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        5, centroid_name.str());

    // Display the plane normal with the camera origin as the origin
    {
      pcl::PointXYZRGB origin;
      origin.x=0;
      origin.y=0;
      origin.z=0;

      // Get the normal from the plane
      pcl::PointXYZRGB dest;
      dest.x = detected_planes.at(i).getCoefficients()->values.at(0);
      dest.y = detected_planes.at(i).getCoefficients()->values.at(1);
      dest.z = detected_planes.at(i).getCoefficients()->values.at(2);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      origin_cloud->push_back(origin);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
      // Project the origin point to the plane
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (origin_cloud);
      proj.setModelCoefficients (detected_planes.at(i).getCoefficients());
      proj.filter (*origin_cloud_projected);
      if(origin_cloud_projected->points.size()!=1)
      {
        std::cout << "Projection fail" << std::endl;
        return 0;
      }

      std::stringstream plane_name("norm_plane_");
      plane_name << i;
      viewer.addLine(origin_cloud_projected->points.at(0),dest, color_sequence.at(color_idx).at(0), color_sequence.at(color_idx).at(1), color_sequence.at(color_idx).at(2), plane_name.str(),v2);
      Eigen::Vector3f norm_origin(
origin_cloud_projected->points.at(0).x,
origin_cloud_projected->points.at(0).y,
origin_cloud_projected->points.at(0).z
          );
      detected_planes.at(i).setNormOrigin(norm_origin);
    }

  }
  // Build a coordinate system for biggest plane
  // Step 1) Visualize the coordinate system
  //
  // Transform the destination of the norm vector, to let it point in a 90° Angle from the
  // centroid to its destination
  Eigen::Vector3f newDest = moveVectorBySubtraction( 
       detected_planes.at(0).getCoefficientsAsVector3f(),
       // The Centroid of the plane cloud
       detected_planes.at(0).getCentroidAsVector3f(),
       // And the center of the norm origin
       detected_planes.at(0).getNormOrigin()
      );
  // Translate the norm vector of the second plane to the centroid of the first plane
  viewer.addLine(
      getPointXYZFromVector4f(detected_planes.at(0).getCentroid()),
      getPointXYZFromVector3f(newDest),
        0, 255, 0, "f1",v2);

  Eigen::Vector3f secondDest = moveVectorBySubtraction( 
       detected_planes.at(1).getCoefficientsAsVector3f(),
       // The Centroid of the plane cloud
       detected_planes.at(0).getCentroidAsVector3f(),
       // And the center of the norm origin
       detected_planes.at(1).getNormOrigin()
      );
  // Translate the norm vector of the second plane to the centroid of the first plane
  viewer.addLine(
      getPointXYZFromVector4f(detected_planes.at(0).getCentroid()),
      getPointXYZFromVector3f(secondDest),
        255, 0, 0, "f2",v2);

  // Create the third axis by using the cross product
  Eigen::Vector3f thirdDest = newDest.cross(secondDest);
  viewer.addLine(
      getPointXYZFromVector4f(detected_planes.at(0).getCentroid()),
      getPointXYZFromVector3f(thirdDest),
        0, 0, 255, "f3",v2);



  //
  // ROTATE THE OBJECT to align it with the x-y and the x-z plane
  // TODO: dynamic
  //
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Align the front face with the cameras front plane
  {
    for (int i = 0; i < detected_planes.size(); i++) 
    {
      std::cout << "Angle between Normal " << i << " and x-y Normal ";

      Eigen::Vector3f n2( 0,0,1);
      float angle = detected_planes.at(i).angleBetween(n2);
      std::cout << ": " << angle << " RAD, " << ((angle * 180) / M_PI) << " DEG";
      std::cout << std::endl;
    }
    std::cout << "Angle between Normal 0 and x-y Normal ";

    Eigen::Vector3f n2( 0,0,1);
    float angle = detected_planes.at(0).angleBetween(n2);

    pcl::PointXYZRGB dest;
    dest.x = detected_planes.at(0).getCoefficients()->values.at(0);
    dest.y = detected_planes.at(0).getCoefficients()->values.at(1);
    dest.z = detected_planes.at(0).getCoefficients()->values.at(2);

    // Translate the first plane's origin to the camera origin
    translatePointCloud(original_cloud,
        -detected_planes.at(0).getCentroid()[0],
        -detected_planes.at(0).getCentroid()[1],
        -detected_planes.at(0).getCentroid()[2],
        rotated_cloud);


    // M
    Eigen::Vector3f plane_normal(dest.x, dest.y, dest.z);

    // Compute the necessary rotation to align a face of the object with the camera's
    // imaginary image plane
    // N
    Eigen::Vector3f camera_normal;
    camera_normal(0)=0;
    camera_normal(1)=0;
    camera_normal(2)=1;

    camera_normal = CuboidMatcher::reduceNormAngle(plane_normal, camera_normal);

    // float dotproduct3 = camera_normal.dot(plane_normal);
    // if(acos(dotproduct3)> M_PI/2)
    // {
    //   std::cout << "NORM IS ABOVE 90 DEG! TURN IN THE OTHER DIRECTION" << std::endl;
    //   camera_normal = -camera_normal;
    //   // rotated_normal_of_second_plane = - rotated_normal_of_second_plane;
    // }

    
    Eigen::Matrix< float, 4, 4 > rotationBox = 
      rotateAroundCrossProductOfNormals(camera_normal, plane_normal);
    
    pcl::transformPointCloud (*rotated_cloud, *rotated_cloud, rotationBox);   

    // Draw the rotated object in the first window
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_r(rotated_cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (rotated_cloud, rgb_r, "rotated_cloud", v2);

    // Rotate the normal of the second plane with the first rotation matrix
    Eigen::Vector3f normal_of_second_plane =
      detected_planes.at(1).getCoefficientsAsVector3f();

    Eigen::Vector3f rotated_normal_of_second_plane;
    rotated_normal_of_second_plane = 
      removeTranslationVectorFromMatrix(rotationBox) * normal_of_second_plane;

    // Now align the second normal (which has been rotated) with the x-z plane
    Eigen::Vector3f xz_plane;
    xz_plane(0)=0;
    xz_plane(1)=1;
    xz_plane(2)=0;

    std::cout << "Angle between rotated normal and xz-plane ";
    float dotproduct2 = xz_plane.dot(rotated_normal_of_second_plane);
    std::cout << ": " << acos(dotproduct2) << " RAD, " << ((acos(dotproduct2) * 180) / M_PI) << " DEG";
    std::cout << std::endl;

    if(acos(dotproduct2)> M_PI/2)
    {
      std::cout << "NORM IS ABOVE 90 DEG! TURN IN THE OTHER DIRECTION" << std::endl;
      rotated_normal_of_second_plane = - rotated_normal_of_second_plane;
    }

    
    float angle_between_xz_and_second_normal = ((acos(dotproduct2) * 180) / M_PI);
    Eigen::Matrix< float, 4, 4 > secondRotation;
    if(angle_between_xz_and_second_normal > MIN_ANGLE &&
        angle_between_xz_and_second_normal < 180-MIN_ANGLE)
    {
    
      // Rotate the original centroid
      Eigen::Vector4f offset_between_centroids =
          detected_planes.at(1).getCentroid() - detected_planes.at(0).getCentroid();
      Eigen::Vector3f rotated_offset =
        removeTranslationVectorFromMatrix(rotationBox) * getVector3fFromVector4f(offset_between_centroids);
   
      
      // translatePointCloud(rotated_cloud, 
      //     - rotated_offset[0],
      //     - rotated_offset[1],
      //     - rotated_offset[2],
      //     rotated_cloud);
          

      secondRotation = 
        rotateAroundCrossProductOfNormals(xz_plane, rotated_normal_of_second_plane);

      pcl::transformPointCloud (*rotated_cloud, *rotated_cloud, secondRotation);   
      second_rotation_performed = true;
    }
    else
    {
      secondRotation = Eigen::Matrix< float, 4, 4 >::Identity();
      second_rotation_performed = false;
      std::cout << "No second rotation, since the angle is too small: "<< angle_between_xz_and_second_normal << "DEG" << std::endl;
    }




    pcl::PointXYZ origin(0,0,0);
    viewer.addLine(origin, getPointXYZFromVector3f(rotated_normal_of_second_plane) , 255, 0, 0, "normal1",v2);

    // Draw the rotated object
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_r2(rotated_cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (rotated_cloud, rgb_r2, "rotated_cloud_second", v3);

    // Compute the bounding box for the rotated object
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*rotated_cloud, min_pt, max_pt);
    
    // Compute the bounding box edge points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr manual_bounding_box(new pcl::PointCloud<pcl::PointXYZRGB>);
    computeCuboidCornersWithMinMax3D(rotated_cloud,manual_bounding_box);

    // Draw the bounding box edge points
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_pts (manual_bounding_box, 255,0,0);
    viewer.addPointCloud<pcl::PointXYZRGB> (manual_bounding_box, red_pts, "bb", v3);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "bb");

    // Now reverse all the transformations to get the bounding box around the actual object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_on_object(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white_pts (bounding_box_on_object, 255,255,255);

    // Inverse the last rotation
    pcl::transformPointCloud (*manual_bounding_box, *bounding_box_on_object, secondRotation.transpose());   

    // Translate back to the first centroid, if this was done
    if(second_rotation_performed)
    {
      Eigen::Vector4f offset_between_centroids =
        detected_planes.at(1).getCentroid() - detected_planes.at(0).getCentroid();
      Eigen::Vector3f rotated_offset =
        removeTranslationVectorFromMatrix(rotationBox) * getVector3fFromVector4f(offset_between_centroids);
      // translatePointCloud(bounding_box_on_object, 
      //     rotated_offset[0], // Translation is now NOT negative
      //     rotated_offset[1],
      //     rotated_offset[2],
      //     bounding_box_on_object);
    }
    // Translated to first centroid
    
    
    // Inverse the second rotation
    pcl::transformPointCloud (*bounding_box_on_object, *bounding_box_on_object, rotationBox.transpose());   

    // And translate back to the original position
    translatePointCloud(bounding_box_on_object, 
        detected_planes.at(0).getCentroid()[0],  
        detected_planes.at(0).getCentroid()[1], 
        detected_planes.at(0).getCentroid()[2], 
        bounding_box_on_object);

    viewer.addPointCloud<pcl::PointXYZRGB> (bounding_box_on_object, white_pts, "bb_real", v3);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "bb_real");
    drawBoundingBoxLines(viewer, bounding_box_on_object, v3);
    // And finally, render the original cloud
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_v3(original_cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (original_cloud, rgb_v3, "original_cloud", v3);

    std::cout << "Cuboid statistics" << std::endl;
    Cuboid c = computeCuboidFromBorderPoints(bounding_box_on_object);
    std::cout << "Width: " << c.length1 << " Height: " << c.length2 << " Depth: " << c.length3 << " Volume: " << c.volume << " m^3" << std::endl;
  }

  boost::posix_time::ptime t_done = boost::posix_time::microsec_clock::local_time();

  printDuration(t_s, t_file_loaded, "Loaded file");
  printDuration(t_file_loaded, t_extraction_done, "Plane Extraction, Normal Estimation");
  printDuration(t_extraction_done, t_done, "Rotation and Visualization");
  printDuration(t_s, t_done, "Overall");

  // 
  // Create a plane x-y plane that originates in the kinect camera frame
  // 
  pcl::ModelCoefficients::Ptr kinect_plane_coefficients (new pcl::ModelCoefficients);
  // Let the Normal of the plane point along the z-Axis
  kinect_plane_coefficients->values.push_back(0); // x
  kinect_plane_coefficients->values.push_back(0); // y
  kinect_plane_coefficients->values.push_back(1); // z 
  kinect_plane_coefficients->values.push_back(0); // d 

  viewer.addPlane (*kinect_plane_coefficients, "kinect_plane", v2);
  viewer.spin();
*/
  return (0);
}
