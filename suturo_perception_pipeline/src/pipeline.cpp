#include "suturo_perception_pipeline/pipeline.h"

#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <suturo_perception_centroid_calc/centroid_calc.h>

#include <boost/asio.hpp>

using namespace suturo_perception;

Pipeline::Pipeline()
{
}

void
Pipeline::execute(PipelineData::Ptr pipeline_data, PipelineObject::VecPtr pipeline_objects)
{
  // Execution pipeline
  // Each capability provides an enrichment for the pipelineObject

  // initialize threadpool
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  std::auto_ptr<boost::asio::io_service::work> work(
    new boost::asio::io_service::work(ioService));

  // Add worker threads to threadpool
  for(int i = 0; i < pipeline_data->numThreads_; ++i)
  {
    threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService)
      );
  }

  int object_cnt = pipeline_objects.size();
  std::vector<CuboidMatcher*> cmvec;
  std::vector<CentroidCalc*> ccvec;
  
  // Initialize Capabilities
  for (int i = 0; i < object_cnt; i++)
  {
    cmvec.push_back(new CuboidMatcher(pipeline_objects[i]));
    ccvec.push_back(new CentroidCalc(pipeline_objects[i]));
  }
  
  // post work to threadpool
  for (int i = 0; i < object_cnt; i++) 
  {
    // cuboid calculation
    CuboidMatcher *cm = cmvec.at(i);
    cm->setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    cm->setTableCoefficients(pipeline_data->coefficients_);
    cm->setInputCloud(pipeline_objects[i]->get_pointCloud());
    ioService.post(boost::bind(&CuboidMatcher::execute, cm, pipeline_objects[i]->get_c_cuboid()));
    
    // centroid calculation
    CentroidCalc *cc = ccvec.at(i);
    ioService.post(boost::bind(&CentroidCalc::execute, cc));
  }
  
  //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  // wait for thread completion.
  // destroy the work object to wait for all queued tasks to finish
  work.reset();
  ioService.run();
  threadpool.join_all();
  
  // Deinitialize Capabilities
  for (int i = 0; i < object_cnt; i++) 
  {
    delete cmvec.at(i);
    delete ccvec.at(i);
  }
}