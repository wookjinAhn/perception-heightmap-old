//
// Created by wj on 22. 4. 19.
//

#ifndef CAMEL_PERCEPTION_HEIGHTMAP_MAPTREENODE_HPP
#define CAMEL_PERCEPTION_HEIGHTMAP_MAPTREENODE_HPP

//#include <ctime>
//#include <cmath>
#include <iostream>
#include <memory>	// unique_ptr

// STL
#include <algorithm>
#include <vector>

#include "Point2.hpp"
#include "Point3.hpp"
#include "BoundingBox.hpp"
#include "MapDataNode.hpp"

namespace camel
{

class MapTreeNode
{
public:
	MapTreeNode();
	MapTreeNode(MapDataNode* mapDataNode);
    MapTreeNode(MapDataNode* mapDataNode, const BoundingBox& boundingBox);
	MapTreeNode(const BoundingBox& boundingBox, int depth, int capacity);
	MapTreeNode(const BoundingBox& boundingBox, int depth);
	MapTreeNode(const BoundingBox& boundingBox);
	~MapTreeNode();

	BoundingBox GetBoundary() const;
	MapDataNode* GetMapDataNode() const;

	void SetMapDataNode(camel::MapDataNode* mapDataNode);
	void SetBoundary(BoundingBox& boundingBox);
	void SetBoundaryKey(float x, float z, float w, float h);
	void SetDepth(int depth);
	void SetCapacity(int capacity);

	void InsertMapTreeNode(std::vector<Point3>& points);
    void InsertMapTreeNode();

private:
	void subdivideNode();
	void insertNodeRecursive(Point3& point, MapDataNode* mapDataNode, int depth);

	MapDataNode* mMapDataNode = nullptr;
	BoundingBox mBoundindBox;
	int mDepth = 6;
	int mCapacity = 1;

	std::vector<Point3> mCapacityPoints;

	bool mbDivided = false;
	std::unique_ptr<MapTreeNode> mNW = nullptr;
	std::unique_ptr<MapTreeNode> mNE = nullptr;
	std::unique_ptr<MapTreeNode> mSW = nullptr;
	std::unique_ptr<MapTreeNode> mSE = nullptr;
};

}

#endif //CAMEL_PERCEPTION_HEIGHTMAP_MAPTREENODE_HPP
