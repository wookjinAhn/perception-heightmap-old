//
// Created by wj on 22. 4. 19.
//

#ifndef OLDMAP_MAPTREENODEROS_HPP
#define OLDMAP_MAPTREENODEROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "oldmap_core/oldmap_core.hpp"
#include "oldmap_ros/MapDataNodeROS.hpp"

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


#endif //OLDMAP_MAPTREENODEROS_HPP
