#ifndef TASK6_CLASSIFICATION
#define TASK6_CLASSIFICATION

#include <perception_utils/cuboid.hpp>
#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/logger.h>
#include <suturo_msgs/Task.h>
#include <shape_msgs/SolidPrimitive.h>
#include <boost/format.hpp>

namespace suturo_perception
{
	class Task6Classification
	{
	public:
		static bool validObject(PipelineObject::Ptr pipeline_object, suturo_msgs::Task task)
		{
			Logger logger("Task 6 classification");
			logger.logInfo("classifying pipeline object");
			if (!pipeline_object->get_c_cuboid_success())
			{
				logger.logInfo("No cuboid success");
				return false;
			}
			Cuboid::Ptr cuboid = pipeline_object->get_c_cuboid();
			logger.logInfo((boost::format("Cuboid dimensions: (%s,%s,%s)") % cuboid->length1 % cuboid->length2 % cuboid->length3).str());
			std::vector<double> cuboid_lengths;
			cuboid_lengths.push_back(cuboid->length1);
			cuboid_lengths.push_back(cuboid->length2);
			cuboid_lengths.push_back(cuboid->length3);
			
			logger.logInfo("analysing task description");
			std::vector<double> expected_lengths;
			bool found_object = false;
			for (int i = 0; i < task.objects.size(); i++)
			{
				if (found_object)
				{
					logger.logError("PANIC! task description contains more than one cube object! Considering all cuboids as valid");
					return true;
				}
				if (task.objects[i].primitives.size() != 1)
				{
					logger.logError("PANIC! task description object doesn't have exactly one primitive!");
					continue;
				}
				if (task.objects[i].primitives[0].type != shape_msgs::SolidPrimitive::BOX)
				{
					logger.logError("PANIC! task description object primitive is no BOX!");
					continue;
				}
				expected_lengths.push_back(task.objects[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]);
				expected_lengths.push_back(task.objects[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
				expected_lengths.push_back(task.objects[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
				found_object = true;
			}
			if (!found_object)
			{
				logger.logError("Task description doesn't expect a cube! Considering all cuboids as valid");
				return true;
			}
			logger.logInfo((boost::format("expected cube: (%s,%s,%s)") % expected_lengths[0] % expected_lengths[1] % expected_lengths[2]).str());
			
			double match_threshold = 0.005; // 5mm
			std::vector<bool> found_matching_length;
			found_matching_length.resize(3, false);
			for (int i = 0; i < cuboid_lengths.size(); i++)
			{
				for (int j = 0; j < expected_lengths.size(); j++)
				{
					double diff = cuboid_lengths[i] - expected_lengths[j];
					diff = diff < 0 ? -diff : diff;
					if (diff < match_threshold)
					{
						found_matching_length[i] = true;
						break;
					}
				}
			}
			for (int i = 0; i < cuboid_lengths.size(); i++)
			{
				if (!found_matching_length[i])
					return false;
			}
			return true;
		}
	};
}
#endif