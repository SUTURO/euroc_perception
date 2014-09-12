#include <suturo_perception_mbpe/generate_pc_model.h>

#define POINTS_PER_BOX 4000
#define POINTS_PER_CYLINDER 5000

Eigen::Matrix4f GeneratePointCloudModel::getRotationMatrixFromPose(Eigen::Matrix< float, 6, 1 > pose)
{
  // The first three elments of the given vector are the
  // translations in x/y/z
  
  Eigen::Matrix3f rz = Eigen::Matrix3f::Identity();
  float theta = pose[5];
  rz (0,0) = cos (theta);
  rz (0,1) = -sin(theta);
  rz (1,0) = sin (theta);
  rz (1,1) = cos (theta);

  Eigen::Matrix3f ry = Eigen::Matrix3f::Identity();
  theta = pose[4];
  ry (0,0) = cos (theta);
  ry (2,0) = -sin(theta);
  ry (0,2) = sin (theta);
  ry (2,2) = cos (theta);

  Eigen::Matrix3f rx = Eigen::Matrix3f::Identity();
  theta = pose[3];
  rx (1,1) = cos (theta);
  rx (1,2) = -sin(theta);
  rx (2,1) = sin (theta);
  rx (2,2) = cos (theta);

  Eigen::Matrix3f rotation = rz * ry * rx;
  Eigen::Matrix4f result;
  // Copy rotational result
  result(0,0) = rotation(0,0);
  result(0,1) = rotation(0,1);
  result(0,2) = rotation(0,2);
  result(1,0) = rotation(1,0);
  result(1,1) = rotation(1,1);
  result(1,2) = rotation(1,2);
  result(2,0) = rotation(2,0);
  result(2,1) = rotation(2,1);
  result(2,2) = rotation(2,2);
  // Apply translation
  // These are the last three elements in the vector.
  result(0,3) = pose[0];
  result(1,3) = pose[1];
  result(2,3) = pose[2];
  result(3,3) = 0; // Set last translation element to 0

  return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeneratePointCloudModel::generateBox(double size_x, double size_y,
    double size_z, int total_points)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  int points_per_side = total_points / 6;

  // "Radius" from the center of the cube to it's surface
  double radius_x = size_x/2;
  double radius_y = size_y/2;
  double radius_z = size_z/2;

  output_cloud->resize(total_points + 60*8); // additional points for each side
  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;

  // Write yz Planes
  int pt_idx = 0;
  double raster_size = 0;
  raster_size = sqrt( (size_y * size_z)  / points_per_side);

  for(double y = -radius_y; y <= radius_y; y+= raster_size)
  {
    for(double z = -radius_z; z <= radius_z; z+= raster_size)
    {
      output_cloud->points[pt_idx].x = -radius_x;
      output_cloud->points[pt_idx].y = y;
      output_cloud->points[pt_idx].z = z;
      pt_idx++;
      output_cloud->points[pt_idx].x = radius_x;
      output_cloud->points[pt_idx].y = y;
      output_cloud->points[pt_idx].z = z;
      pt_idx++;
    }
  }

  // Write xz Planes
  raster_size = 0;
  raster_size = sqrt( (size_x * size_z)  / points_per_side);

  for(double x = -radius_x; x <= radius_x; x+= raster_size)
  {
    for(double z = -radius_z; z <= radius_z; z+= raster_size)
    {
      output_cloud->points[pt_idx].x = x;
      output_cloud->points[pt_idx].y = -radius_y;
      output_cloud->points[pt_idx].z = z;
      pt_idx++;
      output_cloud->points[pt_idx].x = x;
      output_cloud->points[pt_idx].y = radius_y;
      output_cloud->points[pt_idx].z = z;
      pt_idx++;
    }
  }


  // Write xy Planes
  raster_size = 0;
  raster_size = sqrt( (size_x * size_y)  / points_per_side);

  for(double x = -radius_x; x <= radius_x; x+= raster_size)
  {
    for(double y = -radius_y; y <= radius_y; y+= raster_size)
    {
      output_cloud->points[pt_idx].x = x;
      output_cloud->points[pt_idx].y = y;
      output_cloud->points[pt_idx].z = -radius_z;
      pt_idx++;
      output_cloud->points[pt_idx].x = x;
      output_cloud->points[pt_idx].y = y;
      output_cloud->points[pt_idx].z = radius_z;
      pt_idx++;
    }
  }
  output_cloud->points.resize(pt_idx);
  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;
  return output_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeneratePointCloudModel::generateCylinder(double length, double radius, int total_points)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  double cylinder_circle_area = 2 * M_PI * radius * radius; 
  double cylinder_mantle_area = 2 * M_PI * radius * length; 
  double cylinder_area = cylinder_circle_area + cylinder_mantle_area;

  double circle_ratio = (cylinder_circle_area / cylinder_area)/2;
  double mantle_ratio = (cylinder_mantle_area / cylinder_area);

  std::cout << circle_ratio << " " << mantle_ratio << std::endl;

  int points_per_circle = circle_ratio * total_points;
  int points_for_mantle = total_points - points_per_circle*2;
  std::cout << points_per_circle << " " << points_for_mantle << std::endl;

  output_cloud->resize(total_points*2); 
  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;

  // // Write circles 
  int pt_idx = 0;
  double raster_size = 0;
  raster_size = sqrt( cylinder_area / total_points);

  for(double x = -radius; x <= radius; x+= raster_size)
  {
    for(double y = -radius; y <= radius; y+= raster_size)
    {
      if(sqrt(x*x + y*y) <= radius)
      {
        output_cloud->points[pt_idx].x = x;
        output_cloud->points[pt_idx].y = y;
        output_cloud->points[pt_idx].z = length/2;
        pt_idx++;
        output_cloud->points[pt_idx].x = x;
        output_cloud->points[pt_idx].y = y;
        output_cloud->points[pt_idx].z = -length/2;
        pt_idx++;
      } 
    }
  }

  // Write the mantle
  for(double z = -length/2 + raster_size; z<=length/2; z+= raster_size)
  {
    for(double x = -radius; x <= radius; x+= raster_size)
    {
      for(double y = -radius; y <= radius; y+= raster_size)
      {
        double dist = sqrt(x*x + y*y);
        if(dist <= radius && dist > radius*0.945)
        {
          output_cloud->points[pt_idx].x = x;
          output_cloud->points[pt_idx].y = y;
          output_cloud->points[pt_idx].z = z;
          pt_idx++;
        } 
      }
    }
  }


  output_cloud->points.resize(pt_idx);
  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;
  return output_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeneratePointCloudModel::generateComposed()
{
  Eigen::VectorXf pose_cylinder = Eigen::Matrix< float, 6, 1 >::Zero();
  pose_cylinder[0] = pose_cylinder[1] =  pose_cylinder[3] =  pose_cylinder[4] = pose_cylinder[5] = 0;
  pose_cylinder[2] = 0.175;

  Eigen::VectorXf pose_box1 = Eigen::Matrix< float, 6, 1 >::Zero();
  pose_box1[0] = pose_box1[1] = pose_box1[2] =  pose_box1[3] =  pose_box1[4] = pose_box1[5] = 0;

  Eigen::VectorXf pose_box2 = Eigen::Matrix< float, 6, 1 >::Zero();
  pose_box2[0] = pose_box2[1] = pose_box2[3] =  pose_box2[4] = pose_box2[5] = 0;
  pose_box2[2] =  0.35;

  // Generate each model
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);
  cylinder = generateCylinder(0.3, 0.01, POINTS_PER_CYLINDER);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr box1(new pcl::PointCloud<pcl::PointXYZRGB>);
  box1 = generateBox(0.05, 0.05, 0.05, POINTS_PER_BOX);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr box2(new pcl::PointCloud<pcl::PointXYZRGB>);
  box2 = generateBox(0.05, 0.05, 0.05, POINTS_PER_BOX);

  pcl::transformPointCloud(*cylinder, *cylinder, getRotationMatrixFromPose(pose_cylinder) );
  pcl::transformPointCloud(*box2, *box2, getRotationMatrixFromPose(pose_box2) );

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
  result = cylinder;
  *result += *box1;
  *result += *box2;
  return result;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeneratePointCloudModel::generateComposed(std::vector<shape_msgs::SolidPrimitive>  &primitives, std::vector<geometry_msgs::Pose> &poses)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < primitives.size(); i++) {
    std::cout << "Generating shape " << i << " with type " << ((int) primitives.at(i).type);
    std::cout << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr shape_part(new pcl::PointCloud<pcl::PointXYZRGB>);
    switch(primitives.at(i).type)
    {
      case shape_msgs::SolidPrimitive::BOX:
        shape_part = generateBox(primitives.at(i).dimensions[0], 
            primitives.at(i).dimensions[1],
            primitives.at(i).dimensions[2],
            POINTS_PER_BOX);
        break;
      case shape_msgs::SolidPrimitive::CYLINDER:
        shape_part = generateCylinder(primitives.at(i).dimensions[0], 
            primitives.at(i).dimensions[1], POINTS_PER_CYLINDER);
        break;

    }

    // Check if the pose is !=0 and a transformation is necessary
    if(poses.at(i).position.x != 0   || 
        poses.at(i).position.y != 0  || 
        poses.at(i).position.z != 0  || 
        poses.at(i).orientation.x != 0 || 
        poses.at(i).orientation.y != 0 || 
        poses.at(i).orientation.z != 0  )
    {
      
      Eigen::VectorXf pose = Eigen::Matrix< float, 6, 1 >::Zero();
      pose[0] = poses.at(i).position.x;
      pose[1] = poses.at(i).position.y;
      pose[2] = poses.at(i).position.z;
      pose[3] = poses.at(i).orientation.x;
      pose[4] = poses.at(i).orientation.y;
      pose[5] = poses.at(i).orientation.z;
      pcl::transformPointCloud(*shape_part, *shape_part, getRotationMatrixFromPose(pose) );
    }
    *result += *shape_part;

  }
  return result;
}
