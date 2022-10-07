//
// Created by wj on 22. 4. 19.
//

#ifndef CAMEL_PERCEPTION_HEIGHTMAP_MAPTREENODEROS_HPP
#define CAMEL_PERCEPTION_HEIGHTMAP_MAPTREENODEROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "heightmap_core/heightmap_core.hpp"
#include "heightmap_ros/MapDataNodeROS.hpp"

namespace camel
{

class MapTreeNodeROS : public camel::MapTreeNode
{
public:
	MapTreeNodeROS();
	MapTreeNodeROS(MapDataNodeROS* mapDataNode);
    MapTreeNodeROS(MapDataNodeROS* mapDataNode, BoundingBox boundingBox);
	MapTreeNodeROS(BoundingBox& boundingBox, int depth, int capacity);
	MapTreeNodeROS(BoundingBox& boundingBox, int depth);
	MapTreeNodeROS(BoundingBox& boundingBox);
	~MapTreeNodeROS();
};

}


#endif //CAMEL_PERCEPTION_HEIGHTMAP_MAPTREENODEROS_HPP
