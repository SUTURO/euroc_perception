#ifndef PIPELINE_OBJECT_H
#define PIPELINE_OBJECT_H

#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include <boost/thread.hpp>
#include <perception_utils/cuboid.hpp>
#include <perception_utils/point.hpp>
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
        // TODO: cuboid
        return obj;
      }
    
    private:
      int c_id;
      Point c_centroid;
      double c_volume;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
      Cuboid c_cuboid;


      boost::shared_ptr<boost::signals2::mutex> mutex;
  };
}

#endif
