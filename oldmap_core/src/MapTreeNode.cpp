//
// Created by wj on 22. 4. 19.
//

#include "oldmap_core/MapTreeNode.hpp"

namespace camel
{
    MapTreeNode::MapTreeNode()
    {
        MapDataNode heightmap;
        mMapDataNode = &heightmap;
        BoundingBox boundary(-1.0f, 1.0f, 2.0f);
        mBoundindBox = boundary;
        mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
        mMapDataNode->SetResolutionByBoundingBox(mDepth);
    }

    MapTreeNode::MapTreeNode(MapDataNode* mapDataNode)
        : mMapDataNode(mapDataNode)
    {
        BoundingBox boundary(-1.0f, 1.0f, 2.0f);
        mBoundindBox = boundary;
        mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
        mMapDataNode->SetResolutionByBoundingBox(mDepth);
    }

    MapTreeNode::MapTreeNode(MapDataNode* mapDataNode, const BoundingBox& boundingBox)
        : mMapDataNode(mapDataNode)
        , mBoundindBox(boundingBox)
    {
        mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
        mMapDataNode->SetResolutionByBoundingBox(mDepth);
    }

    MapTreeNode::MapTreeNode(const BoundingBox& boundingBox, int depth, int capacity)
        : mBoundindBox(boundingBox)
        , mDepth(depth)
        , mCapacity(capacity)
    {
        mCapacityPoints.reserve(mCapacity);
//        mMapDataNode->SetBoundingBox(mBoundindBox);
//        std::cout << "constructor End" << std::endl;
//		mMapDataNode->SetResolutionByBoundingBox(mDepth);
//        std::cout << "constructor End" << std::endl;
    }

    MapTreeNode::MapTreeNode(const BoundingBox& boundingBox, int depth)
        : mBoundindBox(boundingBox)
        , mDepth(depth)
    {
        mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
        mMapDataNode->SetResolutionByBoundingBox(mDepth);
    }

    MapTreeNode::MapTreeNode(const BoundingBox& boundingBox)
        : mBoundindBox(boundingBox)
        , mDepth(6)
    {
        mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
        mMapDataNode->SetResolutionByBoundingBox(mDepth);
    }

    MapTreeNode::~MapTreeNode()
    {
//		if (mMapDataNode != nullptr)
//		{
//			delete mMapDataNode;
//			mMapDataNode = nullptr;
//		}
    }

    BoundingBox MapTreeNode::GetBoundary() const
    {
        return mBoundindBox;
    }

    MapDataNode* MapTreeNode::GetMapDataNode() const
    {
        return mMapDataNode;
    }

    void MapTreeNode::SetMapDataNode(MapDataNode* mapDataNode)
    {
        if (!mMapDataNode)
        {
            mMapDataNode = mapDataNode;
        }
        else
        {
            delete mMapDataNode;
            mMapDataNode = mapDataNode;
        }
    }

    void MapTreeNode::SetBoundary(BoundingBox& boundingBox)
    {
        mBoundindBox = boundingBox;
    }

    void MapTreeNode::SetBoundaryKey(float x, float z, float w, float h)
    {
        mBoundindBox.SetBoundary(x, z, w, h);
    }

    void MapTreeNode::SetDepth(int depth)
    {
        mDepth = depth;
    }

    void MapTreeNode::SetCapacity(int capacity)
    {
        mCapacity = capacity;
        mCapacityPoints.reserve(mCapacity);
    }

    void MapTreeNode::Process(std::vector<Point3>& points)
    {
        std::cout << "[ Process ]" << std::endl;

        // changeDataCoordinateTo(mCurrentData, eCoordinate::WORLD);
        for (auto& iter : points)
        {
            float x = iter.GetZ();
            float y = -(iter.GetX());
            float z = -(iter.GetY());

            iter.SetXYZ(x, y, z);
        }

        // initializeMap();    // -10.0f;

        // rotateDefaultPosition();
        float defaultRotateAngle = 0.3263766f;

        float pitch00 = cos(defaultRotateAngle);
        float pitch01 = 0.0f;
        float pitch02 = sin(defaultRotateAngle);
        float pitch10 = 0.0f;
        float pitch11 = 1.0f;
        float pitch12 = 0.0f;
        float pitch20 = -sin(defaultRotateAngle);
        float pitch21 = 0.0f;
        float pitch22 = cos(defaultRotateAngle);

        for (int i = 0; i < points.size(); i++)
        {
            Point3 data = points[i];

            float resultX = pitch00 * data.GetX() + pitch01 * data.GetY() + pitch02 * data.GetZ() + mMapDataNode->GetDefaultCameraPosition()[1];
            float resultY = pitch10 * data.GetX() + pitch11 * data.GetY() + pitch12 * data.GetZ() + mMapDataNode->GetDefaultCameraPosition()[2];
            float resultZ = pitch20 * data.GetX() + pitch21 * data.GetY() + pitch22 * data.GetZ() + mMapDataNode->GetDefaultCameraPosition()[3];

            points[i].SetXYZ(resultX, resultY, resultZ);
        }

        float* currentOdom = mMapDataNode->GetCurrentOdom();
        float qxx = currentOdom[3] * currentOdom[3];
        float qyy = currentOdom[4] * currentOdom[4];
        float qzz = currentOdom[5] * currentOdom[5];
        float qxz = currentOdom[3] * currentOdom[5];
        float qxy = currentOdom[3] * currentOdom[4];
        float qyz = currentOdom[4] * currentOdom[5];
        float qwx = currentOdom[6] * currentOdom[3];
        float qwy = currentOdom[6] * currentOdom[4];
        float qwz = currentOdom[6] * currentOdom[5];

        float rotate00 = 1 - 2 * (qyy + qzz);
        float rotate01 = 2 * (qxy - qwz);
        float rotate02 = 2 * (qxz + qwy);

        float rotate10 = 2 * (qxy + qwz);
        float rotate11 = 1 - 2 * (qxx + qzz);
        float rotate12 = 2 * (qyz - qwx);

        float rotate20 = 2 * (qxz - qwy);
        float rotate21 = 2 * (qyz + qwx);
        float rotate22 = 1 - 2 * (qxx + qyy);

        for (int i = 0; i < points.size(); i++)
        {
            Point3 rotatedData = points[i];

            float resultX = rotate00 * rotatedData.GetX() + rotate01 * rotatedData.GetY() + rotate02 * rotatedData.GetZ() + currentOdom[0];
            float resultY = rotate10 * rotatedData.GetX() + rotate11 * rotatedData.GetY() + rotate12 * rotatedData.GetZ() + currentOdom[1];
            float resultZ = rotate20 * rotatedData.GetX() + rotate21 * rotatedData.GetY() + rotate22 * rotatedData.GetZ() + currentOdom[2];

            points[i].SetXYZ(resultX, resultY, resultZ);
        }

        for (auto& iter : points)
        {
            float x = -(iter.GetY());
            float y = -(iter.GetZ());
            float z = iter.GetX();

            iter.SetXYZ(x, y, z);
        }

        // insertDataToEachCell();
        insertMapTreeNode(points);
    }

    void MapTreeNode::insertMapTreeNode(std::vector<Point3>& points)
    {
        for (int i = 0; i < points.size(); i++)
        {
            int depth = 0;
            insertNodeRecursive(points[i], mMapDataNode, depth);
        }

        mMapDataNode->MakeMapToPoints();
    }

    void MapTreeNode::insertMapTreeNode()
    {
        std::vector<Point3> points = mMapDataNode->SamplingPoints(5000);

        for (int i = 0; i < points.size(); i++)
        {
            int depth = 0;
            insertNodeRecursive(points[i], mMapDataNode, depth);
        }

        mMapDataNode->MakeMapToPoints();
    }

    void MapTreeNode::subdivideNode()
    {
        float x = mBoundindBox.GetX();
        float z = mBoundindBox.GetZ();
        float w = mBoundindBox.GetW();
        float h = mBoundindBox.GetH();

        BoundingBox nw(x - w / 2, z + h / 2, w / 2, h / 2);
        BoundingBox ne(x + w / 2, z + h / 2, w / 2, h / 2);
        BoundingBox sw(x - w / 2, z - h / 2, w / 2, h / 2);
        BoundingBox se(x + w / 2, z - h / 2, w / 2, h / 2);

        mNW = std::make_unique<MapTreeNode>(nw, mDepth, mCapacity);
        mNE = std::make_unique<MapTreeNode>(ne, mDepth, mCapacity);
        mSW = std::make_unique<MapTreeNode>(sw, mDepth, mCapacity);
        mSE = std::make_unique<MapTreeNode>(se, mDepth, mCapacity);

        mbDivided = true;
    }

    void MapTreeNode::insertNodeRecursive(Point3& point, MapDataNode* mapDataNode, int depth)
    {
        if (mDepth == depth)
        {
            point.SetNodeKeyXZ(mBoundindBox.GetX(), mBoundindBox.GetZ());
            mapDataNode->MakeHeightMap(point);
            return;
        }

        mCapacityPoints.push_back(point);

        if (mCapacity < mCapacityPoints.size() && mDepth > depth)
        {
            subdivideNode();
        }

        if (mbDivided)
        {
            while (!mCapacityPoints.empty())
            {
                Point3 qPoint = mCapacityPoints.back();
                mCapacityPoints.pop_back();

                if (mNW->mBoundindBox.IsConstained(qPoint))
                {
                    mNW->insertNodeRecursive(qPoint, mapDataNode, ++depth);
                }
                else if (mNE->mBoundindBox.IsConstained(qPoint))
                {
                    mNE->insertNodeRecursive(qPoint, mapDataNode, ++depth);
                }
                else if (mSW->mBoundindBox.IsConstained(qPoint))
                {
                    mSW->insertNodeRecursive(qPoint, mapDataNode, ++depth);
                }
                else if (mSE->mBoundindBox.IsConstained(qPoint))
                {
                    mSE->insertNodeRecursive(qPoint, mapDataNode, ++depth);
                }
            }
        }
    }
}
