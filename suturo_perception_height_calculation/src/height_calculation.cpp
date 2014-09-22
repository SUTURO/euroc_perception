#include "suturo_perception_height_calculation/height_calculation.h"

using namespace suturo_perception;

HeightCalculation::HeightCalculation(PipelineData::Ptr data, PipelineObject::Ptr obj) : Capability(data, obj)
{
  logger = Logger("height_calculation");
}

void
HeightCalculation::execute()
{
  pipelineObject_->set_c_height(-1.0);
}
