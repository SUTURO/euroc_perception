#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <math.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateBox(double size_x, double size_y,
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateCylinder(double length, double radius, int total_points)
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
        if(dist <= radius && dist > radius*0.95)
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

int
main (int argc, char** argv)
{
  // vector of filename indices
  std::vector<int> filenames;
  std::string output_filename;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 1)
  {
    std::cout << "Usage: output_file_path.pcd\n";
    exit (-1);
  }
  output_filename = argv[filenames.at(0)];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  output_cloud = generateBox(0.05, 0.05, 0.05, 3000);
  // output_cloud = generateCylinder(0.3, 0.2, 10000);

  // write pcd
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << output_filename;
  writer.write(ss.str(), *output_cloud);
  std::cerr << "Saved " << output_cloud->points.size () << " data points" << std::endl;

  return (0);
}

