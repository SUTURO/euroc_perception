#ifndef PIPELINE_OBJECT_H
#define PIPELINE_OBJECT_H

#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <perception_utils/cuboid.hpp>
#include <perception_utils/point.hpp>
#include <perception_utils/logger.h>
#include <perception_utils/shape.hpp>
#include <suturo_perception_msgs/EurocObject.h>


namespace suturo_perception
{
  class PipelineObject
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef boost::shared_ptr<PipelineObject> Ptr;
      typedef std::vector<suturo_perception::PipelineObject::Ptr, Eigen::aligned_allocator<suturo_perception::PipelineObject::Ptr> > VecPtr;

      PipelineObject() : mutex(new boost::signals2::mutex()) {
        c_id = -1;
        c_centroid.x = -1;
        c_centroid.y = -1;
        c_centroid.z = -1;
        c_volume = -1.0;
        c_cuboid = Cuboid::Ptr(new Cuboid());
        c_cuboid->length1 = -1;
        c_cuboid->length2 = -1;
        c_cuboid->length3 = -1;
        c_cuboid->volume = -1;
        c_cuboid_success = false;
        c_shape = None;
        c_avg_col_h = -1;
        c_avg_col_s = -1.0;
        c_avg_col_v = -1.0;
        c_mpe_success = false;
        c_mpe_object = boost::shared_ptr<moveit_msgs::CollisionObject>(new moveit_msgs::CollisionObject());
        c_height = -1.0;
        c_color_class = "";

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

      Cuboid::Ptr get_c_cuboid() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_cuboid; 
      };

      Shape get_c_shape() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_shape; 
      };

      int get_c_avg_col_h() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_avg_col_h; 
      };

      double get_c_avg_col_s() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_avg_col_s; 
      };

      double get_c_avg_col_v() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_avg_col_v; 
      };
      
      boost::shared_ptr<moveit_msgs::CollisionObject> get_c_mpe_object() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_mpe_object; 
      };
      
      bool get_c_mpe_success() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_mpe_success; 
      };

      double get_c_height() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_height; 
      };

      bool get_c_cuboid_success() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_cuboid_success; 
      };

      std::vector<int> get_mpe_settings() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return mpe_settings; 
      };
      
      std::string get_c_color_class() const
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex); 
        return c_color_class; 
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
      void set_c_cuboid(Cuboid::Ptr &value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_cuboid = value;
      };
      void set_c_shape(Shape value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_shape = value;
      };
      void set_c_avg_col_h(int value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_avg_col_h = value;
      };
      void set_c_avg_col_s(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_avg_col_s = value;
      };
      void set_c_avg_col_v(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_avg_col_v = value;
      };
      void set_c_mpe_object(boost::shared_ptr<moveit_msgs::CollisionObject> value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_mpe_object = value;
      };
      void set_c_mpe_success(bool value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_mpe_success = value;
      };
      void set_c_height(double value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_height = value;
      };
      void set_c_cuboid_success(bool value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_cuboid_success = value;
      };
      void set_mpe_settings(std::vector<int> value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        mpe_settings = value;
      };
      void set_c_color_class(std::string value)
      {
        boost::lock_guard<boost::signals2::mutex> lock(*mutex);
        c_color_class = value;
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
        obj.c_volume = c_volume;
        obj.c_type = suturo_perception_msgs::EurocObject::UNKNOWN; 
        
        // cuboid
        obj.c_cuboid_success = get_c_cuboid_success();
        if (get_c_cuboid_success())
        {
          Cuboid::Ptr cub = get_c_cuboid();
          shape_msgs::SolidPrimitive cuboid_primitive;
          cuboid_primitive.type = shape_msgs::SolidPrimitive::BOX;
          cuboid_primitive.dimensions.resize(3);
          cuboid_primitive.dimensions.at(shape_msgs::SolidPrimitive::BOX_X) = cub->length1;
          cuboid_primitive.dimensions.at(shape_msgs::SolidPrimitive::BOX_Y) = cub->length2;
          cuboid_primitive.dimensions.at(shape_msgs::SolidPrimitive::BOX_Z) = cub->length3;
          obj.object.primitives.push_back(cuboid_primitive);

          geometry_msgs::Pose cuboid_pose;
          cuboid_pose.position.x = cub->center[0];
          cuboid_pose.position.y = cub->center[1];
          cuboid_pose.position.z = cub->center[2];
          // TODO: find out which one is right
          //cuboid_pose.position.x = cub->center[0] - cub->length1 / 2;
          //cuboid_pose.position.y = cub->center[1] - cub->length2 / 2;
          //cuboid_pose.position.z = cub->center[2] - cub->length3 / 2;
          cuboid_pose.orientation.x = cub->orientation.x();
          cuboid_pose.orientation.y = cub->orientation.y();
          cuboid_pose.orientation.z = cub->orientation.z();
          cuboid_pose.orientation.w = cub->orientation.w();
          obj.object.primitive_poses.push_back(cuboid_pose);
        }

        // shape
        switch(c_shape)
        {
          case Sphere:
            obj.c_shape = suturo_perception_msgs::EurocObject::SHAPE_SPHERE;
          break;
          case Box:
            obj.c_shape = suturo_perception_msgs::EurocObject::SHAPE_BOX;
          break;
          case Cylinder:
            obj.c_shape = suturo_perception_msgs::EurocObject::SHAPE_CYLINDER;
          break;
          case None:
          default:
            obj.c_shape = suturo_perception_msgs::EurocObject::SHAPE_UNKNOWN;
        }

        // average color
        obj.c_avg_col_h = c_avg_col_h;
        obj.c_avg_col_s = c_avg_col_s;
        obj.c_avg_col_v = c_avg_col_v;
        
        // color class
        obj.c_color_class = c_color_class;
        
        // mbpe_object
        obj.mpe_success = c_mpe_success;
        obj.mpe_object = *c_mpe_object;

        // object height
        obj.c_height = c_height;
        return obj;
      }
    
    private:
      int c_id;
      Point c_centroid;
      double c_volume;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
      Cuboid::Ptr c_cuboid;
      bool c_cuboid_success;
      Shape c_shape;
      int c_avg_col_h;
      double c_avg_col_s;
      double c_avg_col_v;
      std::string c_color_class;
      boost::shared_ptr<moveit_msgs::CollisionObject> c_mpe_object;
      bool c_mpe_success;
      double c_height;
      std::vector<int> mpe_settings;

      boost::shared_ptr<boost::signals2::mutex> mutex;

      Logger logger;
  };
}

#endif
