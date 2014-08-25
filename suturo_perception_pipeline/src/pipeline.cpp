#include "suturo_perception_pipeline/pipeline.h"

#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <suturo_perception_centroid_calc/centroid_calc.h>
#include <suturo_perception_shape_detection/shape_detector.h>

#include <boost/asio.hpp>

using namespace suturo_perception;

Pipeline::Pipeline()
{
}

Capability* 
Pipeline::instantiateCapability(CapabilityType type, PipelineObject::Ptr obj, bool enabled)
{
  switch (type)
  {
    case CUBOID_MATCHER:
    return new CuboidMatcher(obj);
    break;  

    case CENTROID_CALC:
    return new CentroidCalc(obj);
    break;

    case SHAPE_DETECTOR:
    return new ShapeDetector(obj);
    break;

    default:
    return NULL;
    break;
  }
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

  // Number of objects to process
  int object_cnt = pipeline_objects.size();
  
  // Available Capabilities
  int avail_capabilities = LAST_CAPABILITY;
  
  // Initialize Capabilities
  std::vector<std::vector<Capability*> > capabilities;
  for (int i = 0; i < object_cnt; i++)
  {
    std::vector<Capability*> object_capabilities;
    for (int j = 0; j < avail_capabilities; j++)
    {
      object_capabilities.push_back(instantiateCapability((CapabilityType)j, pipeline_objects[i]));
    }
    capabilities.push_back(object_capabilities);
    /*
    cmvec.push_back(new CuboidMatcher(pipeline_objects[i]));
    ccvec.push_back(new CentroidCalc(pipeline_objects[i]));
    sdvec.push_back(new ShapeDetector(pipeline_objects[i]));
    */
  }
  
  // post work to threadpool
  for (int i = 0; i < object_cnt; i++) 
  {
    for (int j = 0; j < avail_capabilities; j++)
    {
      if (capabilities[i][j]->isEnabled())
        ioService.post(boost::bind(&Capability::execute, capabilities[i][j]));
    }
    /*
    // cuboid calculation
    CuboidMatcher *cm = cmvec.at(i);
    cm->setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    cm->setTableCoefficients(pipeline_data->coefficients_);
    cm->setInputCloud(pipeline_objects[i]->get_pointCloud());
    if (cm->isEnabled())
      ioService.post(boost::bind(&CuboidMatcher::execute, cm, pipeline_objects[i]->get_c_cuboid()));
    
    // centroid calculation
    CentroidCalc *cc = ccvec.at(i);
    if (cc->isEnabled())
      ioService.post(boost::bind(&CentroidCalc::execute, cc));
    
    // shape detection
    ShapeDetector *sd = sdvec.at(i);
    if (sd->isEnabled())
      ioService.post(boost::bind(&ShapeDetector::execute, sd));
    */
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
    for (int j = 0; j < avail_capabilities; j++)
    {
      delete capabilities[i][j];
    }
    /*
    Cuboid::Ptr c = pipeline_objects[i]->get_c_cuboid();
    std::cout << "Cuboid statistics: ";
    std::cout << "Width: " << c->length1 << " Height: " << c->length2 << " Depth: " << c->length3 << " Volume: " << c->volume;
    std::cout << " m^3" << "O: " << c->orientation.w() << " " << c->orientation.x() << " " << c->orientation.y() << " " << c->orientation.z() << std::endl;
    
    delete cmvec.at(i);
    delete ccvec.at(i);
    delete sdvec.at(i);
    */
  }
}

int
Pipeline::capabilityEnabled(std::string capability_settings, std::string capability_name)
{
  std::vector<std::string> enabled_capabilities = split(capability_settings, ',');
  if (enabled_capabilities.size() == 0)
    return -1;
  return std::find(enabled_capabilities.begin(), enabled_capabilities.end(), capability_name) != enabled_capabilities.end()==0?0:1;
}
// taken from: http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
std::vector<std::string> &Pipeline::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

// taken from: http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
std::vector<std::string> Pipeline::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}
