#ifndef BASIC_IO_PCD_H
#define BASIC_IO_PCD_H
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>

/**
 * This class provides a basic interface for the following pipeline:
 * 1) read in two .pcd filenames from the commandline. The first given
 *    .pcd Filename will be the input file, while the second will
 *    be the filename of the output file.
 *    If these 2 filenames are missing or the input file can not
 *    be opened, the program will exit(-1)
 *
 * 2) execute() a method on the given input.
 *    You can inherit from this class to overwrite the desired behaviour
 *    of this method. By default, it will just point the output pointcloud
 *    to the input pointcloud. Therefore, you will get a copy of your input
 *    cloud
 *
 * 3) write the resultant cloud to disk, according to the given output 
 *    filename.
 */
class BasicIOPCD {
  protected:
    char* input_filename_;
    char* output_filename_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_;
  public:
    BasicIOPCD(int argc, char** argv);
    void execute();
    void write_pcd();
};
#endif
