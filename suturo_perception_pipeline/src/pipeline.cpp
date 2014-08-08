#include "suturo_perception_pipeline/pipeline.h"

#include <suturo_perception_match_cuboid/cuboid_matcher.h>

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

  std::vector<CuboidMatcher*> cmvec;
  for (int i = 0; i < pipeline_objects.size(); i++) 
  {
    // Initialize Capabilities
    
    // suturo_perception_3d_capabilities::CuboidMatcherAnnotator cma(perceivedObjects[i]);
    // Init the cuboid matcher with the table coefficients
    CuboidMatcher *cm = new CuboidMatcher(pipeline_objects[i]);
    cmvec.push_back(cm);
    cm->setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    cm->setTableCoefficients(pipeline_data->coefficients_);
    cm->setInputCloud(pipeline_objects[i]->get_pointCloud());

    // post work to threadpool
    ioService.post(boost::bind(&CuboidMatcher::execute, cm, pipeline_objects[i]->get_c_cuboid()));
  }
  //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  // wait for thread completion.
  // destroy the work object to wait for all queued tasks to finish
  work.reset();
  ioService.run();
  threadpool.join_all();
  
  for (int i = 0; i < cmvec.size(); i++) 
  {
    delete cmvec.at(i);
  }
}