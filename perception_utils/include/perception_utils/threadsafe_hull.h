#ifndef THREADSAFE_HULL_H
#define THREADSAFE_HULL_H

#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/impl/point_types.hpp>
#include <boost/shared_ptr.hpp>

namespace perception_utils
{
	/**
	 * Helper class to compute a ConvexHull threadsafe.
   * PCL calls libqhull in a non-threadsafe way.
   * We can overcome this limitation by implementing a static
   * method that calls the ConvexHull Calculation. During this computation, the static method will be locked with a mutex.
	 */
  class ThreadsafeHull
  {
    public:
      
      template<typename PointT>
      static void computeConvexHull(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_in, boost::shared_ptr<pcl::PointCloud<PointT> > hull_points)
      {
        static boost::signals2::mutex mutex;
        mutex.lock(); // Lock this method, since libqull is NOT threadsafe

        if(cloud_in == NULL || cloud_in->points.size() == 0)
        {
          std::cerr << "ThreadsafeHull::computeConvexHull with empty cloud called" << std::endl;
          return;
        }

        pcl::ConvexHull<PointT> hull;
        hull.setInputCloud(cloud_in);
        hull.setDimension(3);
        hull.reconstruct (*hull_points);
        mutex.unlock();
      }
	};
}

#endif
