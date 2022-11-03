//
// Created by wj on 22. 4. 19.
//

//#include "../include/heightmap_ros/MapTreeNodeROS.h"
#include "oldmap_ros/MapTreeNodeROS.hpp"

namespace camel
{

    MapTreeNodeROS::MapTreeNodeROS()
        : camel::MapTreeNode()
    {
    }

    MapTreeNodeROS::MapTreeNodeROS(MapDataNodeROS* mapDataNode)
        : camel::MapTreeNode(mapDataNode)
    {
    }

    MapTreeNodeROS::MapTreeNodeROS(MapDataNodeROS* mapDataNode, BoundingBox boundingBox)
        : camel::MapTreeNode(mapDataNode, boundingBox)
    {
    }

    MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox, int depth, int capacity)
        : camel::MapTreeNode(boundingBox, depth, capacity)
    {
    }

    MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox, int depth)
        : camel::MapTreeNode(boundingBox, depth)
    {
    }

    MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox)
        : camel::MapTreeNode(boundingBox)
    {
    }

    MapTreeNodeROS::~MapTreeNodeROS()
    {
    }

}
