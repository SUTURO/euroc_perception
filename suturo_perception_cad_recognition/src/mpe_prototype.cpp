#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>
#include <suturo_perception_cad_recognition/model_pose_estimation.h>
// 
// Eigen::Vector4f getTableNormalFromStringLine(std::string object_table_normal, std::string package_path)
// {
// 
//   // Process table normal
//   std::ifstream file( (package_path + "/" + object_table_normal).c_str() );
//   std::string table_normal_string; 
//   if(!std::getline(file, table_normal_string))
//   {
//     std::cout << "The table normal file can't be read";
//     exit (-1);
//   } 
//   std::vector<std::string> table_normal_components;
//   boost::split(table_normal_components, table_normal_string, boost::is_any_of(","));
// 
//   if(table_normal_components.size() != 4)
//   {
//     std::cout << "Extracted " << table_normal_components.size() << " components from the table normal file: " << object_table_normal << ". Required are exactly 4! See the pcl ModelCoefficients or a RANSAC plane for reference" << std::endl;
//     exit (-1);
//   }
//   Eigen::Vector4f table_normal(
//       atof(table_normal_components.at(0).c_str()),
//       atof(table_normal_components.at(1).c_str()),
//       atof(table_normal_components.at(2).c_str()),
//       atof(table_normal_components.at(3).c_str())
//       );
//   return table_normal;
// }
// 
// 
// int main(int argc, const char *argv[])
// {
//   
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   std::string package_path = ros::package::getPath("suturo_perception_mbpe");
//   // std::string modelpath  = "test_files/005box_4000pts.pcd";
//   std::string objectpath = "test_files/correctly_segmented_box.pcd";
//   std::string object_table_normal_path = "test_files/correctly_segmented_box.pcd_table_normal";
// 
//   if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + objectpath, *input_cloud) == -1)
//   {
//     PCL_ERROR ("Couldn't read input file\n");
//     exit (-1);
//   }
// 
//   Eigen::Vector4f table_normal = getTableNormalFromStringLine(object_table_normal_path,package_path);
//   std::cout << "Using table normal: " << table_normal << std::endl;
// 
//   // Prepare the model that should be matched against the input cloud
//   boost::shared_ptr<std::vector<suturo_msgs::Object> > objects(new std::vector<suturo_msgs::Object>);
//   suturo_msgs::Object obj;
//   obj.name="red_cube";
//   obj.color="ff0000";
//   obj.description="a red cube";
//   obj.surface_material = suturo_msgs::Object::ALUMINIUM;
// 
//   suturo_msgs::Shape shape1;
//   shape1.shape_type = shape1.BOX;
//   // 0.05 x 0.05 x 0.05
//   shape1.dimensions.push_back(0.05f);
//   shape1.dimensions.push_back(0.05f);
//   shape1.dimensions.push_back(0.05f);
//   shape1.pose.linear.x = 0;
//   shape1.pose.linear.y = 0;
//   shape1.pose.linear.z = 0;
//   shape1.pose.angular.x = 0;
//   shape1.pose.angular.y = 0;
//   shape1.pose.angular.z = 0;
//   obj.shapes.push_back(shape1);
//   objects->push_back(obj);
//   // shapes->push_back(shape1);
// 
//   ModelPoseEstimation mpe(objects);
//   mpe.setInputCloud(input_cloud);
//   mpe.setSurfaceNormal(table_normal);
//   mpe.setVoxelSize(0.003f);
//   mpe.execute();
// 
//   std::cout << mpe.getFitnessScore() << std::endl;
//   return 0;
// }
//
//
/*
 * This node uses the ICPFitter library to estimate
 * the pose of a segmented, partial pointcloud against a 
 * CAD Model. The results will be visualized as follows:
 *   1) The input cloud and the cloud of the CAD model
 *   2) The input cloud against the ICP-fitted model (after the initial alignment)
 *   3) Visualization of each step during the pose estimation process.
 *
 * The CAD model has to be subsampled as a Pointcloud and be
 * passed to this node.
 * You can subsample a CAD model with CloudCompare (http://www.danielgm.net/cc/)
 * Future releases of this software may automate this step.
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
// #include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
// #include <suturo_perception_utils.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/fpfh.h>
// #include <pcl/features/shot.h>
#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <suturo_perception_cad_recognition/icp_fitter.h>
#include <boost/algorithm/string.hpp>

namespace po = boost::program_options;
using namespace boost;
using namespace std;

void drawNormalizedVector(pcl::visualization::PCLVisualizer &viewer, Eigen::Vector3f origin, Eigen::Vector3f vector, std::string identifier, int &viewport)
{
  pcl::PointXYZ p_origin(origin[0], origin[1], origin[2] );
  pcl::PointXYZ p_vector(vector[0], vector[1], vector[2] );

  viewer.addLine<pcl::PointXYZ> (p_origin, p_vector, identifier, viewport);
}

int main(int argc, char** argv){
  std::string cad_model_pc_filename;
  std::string input_pc_filename;
  std::string table_normal_string;
  std::string downsample_size;
  int max_iterations=-1;
  int max_distance=-1;

  bool turn_model=false;
  bool use_leaf_size=false;

  // "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("input-pc,i", po::value<std::string>(&input_pc_filename)->required(), "The  filename of the input pointcloud.")
      ("cad-model-pc,m", po::value<std::string>(&cad_model_pc_filename)->required(), "A pointcloud of the CAD-Model to match. You can get a Pointcloud of your CAD-Model with CloudCompare.")
      ("max-iterations,c", po::value<int>(&max_iterations), "The max iteration count for ICP. Default: 60")
      ("max-correspondence-distance,d", po::value<int>(&max_distance), "The max iteration correspondence distance for ICP. If no value is set, the PCL default will be used")
      ("table_normal,t", po::value<std::string>(&table_normal_string)->required(), "The normal of the surface where the object rests on")
      ("downsample-size,l", po::value<std::string>(&downsample_size), "The leaf size for the downsampling process - Default = 0.005f")
      ("model-upside,u", po::value<bool>()->zero_tokens(), "Turn the model upwards before running ICP - Default=false")
    ;

    po::positional_options_description p;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      std::cout << "Usage: cad_recognition -i input_cloud.pcd -m cad_model_cloud.pcd -t 'table_normal'" << endl << endl;
      std::cout << desc << "\n";
      return 1;
    }

    if (vm.count("model-upside")) {
      turn_model = true;
    }

    if (vm.count("downsample-size")) {
      use_leaf_size = true;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

  }
  catch(std::exception& e)
  {
    std::cout << "Usage: cad_recognition -i input_cloud.pcd -m cad_model_cloud.pcd -t 'table_normal'" << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PolygonMesh::Ptr model_mesh (new pcl::PolygonMesh);

  boost::posix_time::ptime file_load_start = boost::posix_time::microsec_clock::local_time();
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_pc_filename, *input_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (cad_model_pc_filename, *model_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read cad model file (points)\n");
    exit (-1);
  }



  // if (pcl::io::loadPolygonFileSTL("test_data/pancake_mix.stl", *model_mesh) == -1) //* load the CAD model file
  // {
  //   PCL_ERROR ("Couldn't read cad model file (mesh)\n");
  //   exit (-1);
  // }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_voxeled (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud_voxeled (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Downsample both clouds
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (input_cloud);

  #define LEAF_SIZE 0.005f
  if(!use_leaf_size)
  {
    sor.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  }else{
    double size = atof(downsample_size.c_str());
    sor.setLeafSize (size, size, size);
  }
  sor.filter (*input_cloud_voxeled);

  sor.setInputCloud (model_cloud);
  sor.filter (*model_cloud_voxeled);

  boost::posix_time::ptime file_load_end = boost::posix_time::microsec_clock::local_time();
  // l.logTime(file_load_start,file_load_end,"File loading and voxeling done");

  std::vector<std::string> table_normal_components;
  boost::split(table_normal_components, table_normal_string, boost::is_any_of(","));

  if(table_normal_components.size() != 4)
  {
    std::cout << "Extracted " << table_normal_components.size() << " components from the table normal string. Required are exactly 4! See the pcl ModelCoefficients or a RANSAC plane for reference" << std::endl;
    return 1;
  }
  std::cout << "Using the following table normal: ";
  std::cout << atof(table_normal_components.at(0).c_str()) << ", ";
  std::cout << atof(table_normal_components.at(1).c_str()) << ", ";
  std::cout << atof(table_normal_components.at(2).c_str()) << ", ";
  std::cout << atof(table_normal_components.at(3).c_str()) << std::endl;
  Eigen::Vector4f table_normal(
      atof(table_normal_components.at(0).c_str()),
      atof(table_normal_components.at(1).c_str()),
      atof(table_normal_components.at(2).c_str()),
      atof(table_normal_components.at(3).c_str())
      );
  // Specify the table normal of the given model
  // Eigen::Vector4f table_normal(-0.0102523,-0.746435,-0.66538,0.92944); // pancake_fail
  // Eigen::Vector4f table_normal(0.0118185, 0.612902, 0.79007, -0.917831); // pancake 
  // Eigen::Vector4f table_normal(0.00924593, 0.697689, 0.716341, -0.914689); // pancake 0deg moved
  // Eigen::Vector4f table_normal(0.0102382,0.6985,0.715537,-0.914034); // pancake 0deg moved
  // Eigen::Vector4f table_normal(0.000572634, 0.489801, 0.871834, -0.64807); // euroc_mbpe/test_files/correctly_segmented_box.pcd
  // Eigen::Vector4f table_normal(0.169393, 0.488678, 0.855862, -0.596477); // euroc_mbpe/test_files/correctly_segmented_cylinder.pcd
  // Eigen::Vector4f table_normal(0.000309765, 0.601889, 0.79858, -0.782525); // euroc_mbpe/test_files/correctly_segmented_handlebar.pcd
 

  // Prepare the model that should be matched against the input cloud
  boost::shared_ptr<std::vector<suturo_msgs::Object> > objects(new std::vector<suturo_msgs::Object>);
  suturo_msgs::Object obj;
  obj.name="red_cube";
  obj.color="ff0000";
  obj.description="a red cube";
  obj.surface_material = suturo_msgs::Object::ALUMINIUM;

  suturo_msgs::Shape shape1;
  shape1.shape_type = shape1.BOX;
  // 0.05 x 0.05 x 0.05
  shape1.dimensions.push_back(0.05f);
  shape1.dimensions.push_back(0.05f);
  shape1.dimensions.push_back(0.05f);
  shape1.pose.linear.x = 0;
  shape1.pose.linear.y = 0;
  shape1.pose.linear.z = 0;
  shape1.pose.angular.x = 0;
  shape1.pose.angular.y = 0;
  shape1.pose.angular.z = 0;
  obj.shapes.push_back(shape1);
  objects->push_back(obj);
  // shapes->push_back(shape1);

  ModelPoseEstimation mpe(objects);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  mpe.setVoxelSize(0.003f);
  mpe.generateModels();
  // mpe.execute();

  std::cout << "Generated Pointcloud with " << model_cloud_voxeled->points.size() << "pts" << std::endl;
  std::cout << "Input Pointcloud with " << input_cloud_voxeled->points.size() << "pts" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*input_cloud_voxeled, *input_cloud_xyz);
  copyPointCloud(*model_cloud_voxeled, *model_cloud_xyz);

  ICPFitter ria(input_cloud_xyz, model_cloud_xyz, table_normal);
  if(max_iterations!=-1)
  {
    ria.setMaxICPIterations(max_iterations);
  }
  else
  {
    ria.setMaxICPIterations(60);
  }
  ria.rotateModelUp(turn_model);
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_initial_aligned = ria.execute();
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration d = end - start;
  float diff = (float)d.total_microseconds() / (float)1000;
  std::cout << "Runtime for ICPFitter execute(): " << diff << "ms" << std::endl;
  ria.dumpPointClouds();

  Eigen::Matrix<float, 4, 4> initial_alignment_rotation = 
    ria.getRotation();

  Eigen::Matrix<float, 4, 4> initial_alignment_translation = 
    ria.getTranslation();

  input_cloud = input_cloud_voxeled;
  model_cloud = model_cloud_voxeled;


  // Refine the result with ICP
  std::cout << "ICP 2" << std::endl;
  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  // icp.setInputSource(ria._upwards_object);
  // icp.setInputTarget(ria._upwards_model);
  /*
  // icp.setInputTarget(ria._upwards_object);
  // icp.setInputSource(ria._upwards_model);
  if(max_iterations!=-1)
  {
    std::cout << "Setting max iterations in ICP to: "<< max_iterations << std::endl;
    icp.setMaximumIterations(max_iterations);
  }
  else
  {
    icp.setMaximumIterations(60);
  }
  // icp.setEuclideanFitnessEpsilon (0.000001f);
  // icp.setEuclideanFitnessEpsilon (0.00000000001f);
  icp.setEuclideanFitnessEpsilon (0.00001f);
  // icp.setMaxCorrespondenceDistance (0.55);
  // icp.setRANSACOutlierRejectionThreshold(0.10f);
  //
  // Observation: The fitness score should be below 1e-5
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
  // icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  */
// 
//   // Check the result of the calculated transformation
//   pcl::PointCloud<pcl::PointXYZ>::Ptr model_initial_transformed (new pcl::PointCloud<pcl::PointXYZ>);
// 
//   Eigen::Matrix<float, 4, 4> icp_transform = icp.getFinalTransformation();
//   Eigen::Matrix<float, 4, 4> icp_transform_inverse = icp.getFinalTransformation().inverse();
//   pcl::transformPointCloud(*model_cloud_voxeled, *model_initial_transformed, ria.rotations_.at(0) );
//   pcl::transformPointCloud(*model_initial_transformed, *model_initial_transformed,icp_transform_inverse );
//   pcl::transformPointCloud(*model_initial_transformed, *model_initial_transformed, initial_alignment_translation );
//   pcl::transformPointCloud(*model_initial_transformed, *model_initial_transformed, ria.rotations_.at(1) );
// 
// 
//   pcl::visualization::PCLVisualizer viewer;
//   int v1,v2,v3,v4;
//   // viewer.createViewPort(0.0,0, 0.25,1, v1 );
//   // viewer.addText("Input Cloud", 0.1, 0.1 , "input_cloud_text_id", v1 );
//   viewer.createViewPort(0.0,0, 0.33,1, v2 );
//   viewer.addText("Model vs. Input Cloud - Roughly aligned (red=model, orange=upwards_model), ", 0.1, 0.1 , "model_cloud_text_id", v2 );
//   viewer.addCoordinateSystem(0.3,v2);
//   // pcl::PointXYZ a(0,1,0);
//   // viewer.addSphere(a,0.5,"sphere",v2);
//   viewer.createViewPort(0.33,0, 0.66  ,1, v3 );
//   viewer.addCoordinateSystem(0.3,v3);
//   viewer.addText("ICP (yellow=ICP result, green=input, orange=model)", 0.1, 0.1 , "icp_text", v3 );
//   viewer.createViewPort(0.66,0, 1  ,1, v4 );
//   viewer.addCoordinateSystem(0.3,v4);
//   viewer.addText("Pose estimation", 0.1, 0.1 , "pose_text", v4 );
// 
//   // Viewport 2
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(input_cloud, 0, 255, 0);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(model_cloud, 255, 0, 0);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upwards_color(ria._upwards_model, 255, 125, 0);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upwards_object(ria._upwards_object, 128, 255, 0);
//   viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color, "input_cloud_id2",v2);
//   viewer.addPointCloud<pcl::PointXYZ> (model_cloud, red_color, "model_cloud_id",v2);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_color",v2);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object, upwards_object, "upwards_object",v2);
//   drawNormalizedVector(viewer, Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0,1), "norm_at_origin", v2);
//   drawNormalizedVector(viewer, Eigen::Vector3f(0,0,0),
//       Eigen::Vector3f(table_normal[0],table_normal[1],table_normal[2])
//       ,"table_normal", v2);
// 
// 
//   // Viewport 3
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow_color(Final, 255, 255, 0);
//   viewer.addPointCloud<pcl::PointXYZ> (Final, yellow_color,"refined_aligned_cloud_id",v3);
//   viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color,"original_cloud_vs_refined_aligned_id",v3);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_model_vs_refined_cloud",v3);
// 
//   // Viewport 4
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pink_color(ria._upwards_object_s2, 255, 187, 255);
//   // viewer.addPointCloud<pcl::PointXYZ> (Final, yellow_color,"refined_aligned_cloud_id",v3);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_init(model_initial_aligned, 255, 255, 0);
//   viewer.addPointCloud<pcl::PointXYZ> (input_cloud, green_color,"original_cloud_vs_pose",v4);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object_s1, green_color,"upwards object s1",v4);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object_s2, pink_color,"upwards object s2",v4);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object_s3, green_color,"upwards object s3",v4);
//   viewer.addPointCloud<pcl::PointXYZ> (model_initial_transformed, red_init, "model_cloud_id_v4",v4);
//   // viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_model_vs_refined_cloud",v3);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_model, upwards_color, "upwards_color_v4",v4);
//   viewer.addPointCloud<pcl::PointXYZ> (ria._upwards_object, upwards_object, "upwards_object_v4",v4);
// 
//   pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed,icp_transform_inverse );
//   viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed, "model_icp_transformed",v4);
// 
//   pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed_s2 (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed_s2,icp_transform_inverse );
//   pcl::transformPointCloud(*model_icp_transformed_s2, *model_icp_transformed_s2, ria.translations_.at(0) );
//   viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed_s2, "model_transformed_s2",v4);
// 
//   pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed_s3 (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed_s3,icp_transform_inverse );
//   pcl::transformPointCloud(*model_icp_transformed_s3, *model_icp_transformed_s3, ria.translations_.at(0) );
//   pcl::transformPointCloud(*model_icp_transformed_s3, *model_icp_transformed_s3, ria.translations_.at(1) );
//   viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed_s3, "model_transformed_s3",v4);
// 
//   pcl::PointCloud<pcl::PointXYZ>::Ptr model_icp_transformed_s4 (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::transformPointCloud(*ria._upwards_model, *model_icp_transformed_s4, 
//       ria.rotations_.at(1) *
//       ria.translations_.at(1) * ria.translations_.at(0) * icp_transform_inverse );
//   viewer.addPointCloud<pcl::PointXYZ> (model_icp_transformed_s4, "model_transformed_s4",v4);
// 
//   // Transform object center
//   pcl::PointXYZ a(0,0,0);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr origin (new pcl::PointCloud<pcl::PointXYZ>);
//   origin->push_back(a);
//   pcl::transformPointCloud(*origin, *origin, 
//       ria.rotations_.at(1) *
//       ria.translations_.at(1) * ria.translations_.at(0) * icp_transform_inverse );
//   viewer.addSphere(origin->at(0),0.03,"sphere2",v4);
// 
//   viewer.spin();
// 
//   return 0;
};
