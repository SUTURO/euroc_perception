#ifndef PIPELINE_OBJECT_H
#define PIPELINE_OBJECT_H

#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include <boost/thread.hpp>
#include <perception_utils/cuboid.hpp>
#include <perception_utils/point.hpp>
#include <perception_utils/logger.h>
#include <suturo_perception_msgs/EurocObject.h>

namespace suturo_perception
{
  class PipelineObject
  {
    public:
      typedef boost::shared_ptr<PipelineObject> Ptr;

      PipelineObject() : mutex(new boost::signals2::mutex()) {
        c_id = -1;
        c_centroid.x = -1;
        c_centroid.y = -1;
        c_centroid.z = -1;
        c_volume = -1.0;
        c_cuboid.length1 = -1;
        c_cuboid.length2 = -1;
        c_cuboid.length3 = -1;
        c_cuboid.volume = -1;

        logger = Logger("pipeline_object");
      };

      // Threadsafe getters      
      int get_c_id() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_id; 
      };
      
      Point get_c_centroid() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        return c_centroid;
      };
      
      double get_c_volume() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_volume;
      };
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_pointCloud() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return pointCloud; 
      };

      Cuboid get_c_cuboid() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_cuboid; 
      };

      // Threadsafe setters
      void set_c_id(int value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_id = value;
      };
      void set_c_centroid(Point value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_centroid = value;
      };
      void set_c_volume(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_volume = value;
      };
      void set_pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        pointCloud = value;
      };
      void set_c_cuboid(Cuboid value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_cuboid = value;
      };

      suturo_perception_msgs::EurocObject toEurocObject()
      {
        suturo_perception_msgs::EurocObject obj;
        obj.c_id = get_c_id();
        obj.frame_id = "";
        Point centroid = get_c_centroid();
        obj.c_centroid.x = centroid.x;
        obj.c_centroid.y = centroid.y;
        obj.c_centroid.z = centroid.z;
        obj.c_type = suturo_perception_msgs::EurocObject::UNKNOWN; 
        
        // cuboid
        Cuboid cub = get_c_cuboid();
        shape_msgs::SolidPrimitive cuboid_primitive;
        cuboid_primitive.type = shape_msgs::SolidPrimitive::BOX;
        logger.logInfo("before");
        cuboid_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = cub.length1;
        cuboid_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = cub.length2;
        cuboid_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = cub.length3;
        logger.logInfo("after");
        obj.object.primitives.push_back(cuboid_primitive);

        geometry_msgs::Pose cuboid_pose;
        cuboid_pose.position.x = cub.center[0];
        cuboid_pose.position.y = cub.center[1];
        cuboid_pose.position.z = cub.center[2];
        // TODO: find out which one is right
        //cuboid_pose.position.x = cub.center[0] - cub.length1 / 2;
        //cuboid_pose.position.y = cub.center[1] - cub.length2 / 2;
        //cuboid_pose.position.z = cub.center[2] - cub.length3 / 2;
        cuboid_pose.orientation.x = cub.orientation.x();
        cuboid_pose.orientation.y = cub.orientation.y();
        cuboid_pose.orientation.z = cub.orientation.z();
        cuboid_pose.orientation.w = cub.orientation.w();
        obj.object.primitive_poses.push_back(cuboid_pose);

        return obj;
      }
    
    private:
      int c_id;
      Point c_centroid;
      double c_volume;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
      Cuboid c_cuboid;

      boost::shared_ptr<boost::signals2::mutex> mutex;

      Logger logger;
  };
}

#endif
