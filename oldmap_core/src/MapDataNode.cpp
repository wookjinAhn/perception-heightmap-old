//
// Created by wj on 22. 4. 19.
//

//#include "../include/heightmap_core/HeightmapNode.h"
#include "oldmap_core/MapDataNode.hpp"

namespace camel
{
    MapDataNode::MapDataNode()
    {
        mInputPoints.reserve(307200);
    }

    MapDataNode::~MapDataNode()
    {
//		for (int i = 0; i < mInputPoints.size(); i++) {
//			delete mInputPoints[i];
//		}
//		mInputPoints.clear();
//
//		for (int i = 0; i < mOutputPoints.size(); i++) {
//			delete mOutputPoints[i];
//		}
//		mOutputPoints.clear();
    }

    double MapDataNode::GetResolution() const
    {
        return mResolution;
    }

    void MapDataNode::SetResolution(double resolution)
    {
        mResolution = resolution;
    }

    void MapDataNode::SetResolutionByBoundingBox(const BoundingBox& boundingBox, int depth)
    {
        double gridWidth = boundingBox.GetW() * 2;

        for (int i = 0; i < depth; i++)
        {
            gridWidth = gridWidth / 2;
        }

        mResolution = gridWidth;
    }

    void MapDataNode::SetResolutionByBoundingBox(int depth)
    {
        double gridWidth = mBoundingBox.GetW() * 2;

        for (int i = 0; i < depth; i++)
        {
            gridWidth = gridWidth / 2;
        }

        mResolution = gridWidth;
    }

    BoundingBox MapDataNode::GetBoundingBox() const
    {
        return mBoundingBox;
    }

    void MapDataNode::SetBoundingBox(BoundingBox boundingBox)
    {
        mBoundingBox = boundingBox;
    }

    std::unordered_map<std::pair<float, float>, float, pair_hash> MapDataNode::GetMapDataPair() const
    {
        return mMapDataPair;
    }

    std::vector<Point3> MapDataNode::GetInputPoints() const
    {
        return mInputPoints;
    }

    std::vector<Point3> MapDataNode::GetOutputPoints() const
    {
        return mOutputPoints;
    }
    void MapDataNode::SetDefaultCameraPosition(float t265PoseZ, float d435PoseX, float d435PoseY, float d435PoseZ)
    {
        mCameraPosition[0] = t265PoseZ;
        mCameraPosition[1] = GetResolution();
        mCameraPosition[2] = d435PoseY;
        mCameraPosition[3] = d435PoseZ;
    }

    void MapDataNode::SetCurrentOdom(float poseX, float poseY, float poseZ, float quaternionX, float quaternionY, float quaternionZ, float quaternionW)
    {
        mCurrentOdom[0] = poseX;
        mCurrentOdom[1] = poseY;
        mCurrentOdom[2] = poseZ;
        mCurrentOdom[3] = quaternionX;
        mCurrentOdom[4] = quaternionY;
        mCurrentOdom[5] = quaternionZ;
        mCurrentOdom[6] = quaternionW;
    }

    float* MapDataNode::GetDefaultCameraPosition()
    {
        return mCameraPosition;
    }
    float* MapDataNode::GetCurrentOdom()
    {
        return mCurrentOdom;
    }

    void MapDataNode::SetInputPoint(Point3 point)
    {
        mInputPoints.push_back(point);
    }

    std::vector<Point3> MapDataNode::SamplingPointsWithRotate(int samplingNum, float rotationDegree)
    {
        if (mInputPoints.size() <= samplingNum)
        {
            return mInputPoints;
        }

        std::vector<Point3> samplingPoints;
        samplingPoints.reserve(samplingNum);

        std::vector<int> randomIndex(mInputPoints.size());
        std::iota(randomIndex.begin(), randomIndex.end(), 0);
        std::shuffle(randomIndex.begin(), randomIndex.end(), std::mt19937{ std::random_device{ }() });

        for (int i = 0; i < samplingNum; i++)
        {
            int pointIndex = randomIndex[i];
            mInputPoints[pointIndex].RotationRoll(rotationDegree);
            samplingPoints.push_back(mInputPoints[pointIndex]);
        }
        return samplingPoints;
    }

    std::vector<Point3> MapDataNode::SamplingPoints(int samplingNum)
    {
        if (mInputPoints.size() <= samplingNum)
        {
            return mInputPoints;
        }

        std::vector<Point3> samplingPoints;
        samplingPoints.reserve(samplingNum);

        std::vector<int> random(mInputPoints.size());
        std::iota(random.begin(), random.end(), 0);
        std::shuffle(random.begin(), random.end(), std::mt19937{ std::random_device{ }() });

        for (int i = 0; i < samplingNum; i++)
        {
            int randomIndex = random[i];
            samplingPoints.push_back(mInputPoints[randomIndex]);
        }

        return samplingPoints;
    }

    void MapDataNode::MakeHeightMap(Point3& point)        // <<float, float>, float> | < <x,z>, y >
    {
        if (mMapDataPair.find(std::make_pair(point.GetNodeKey().GetX(), point.GetNodeKey().GetZ())) == mMapDataPair.end())    // not exist
        {
            mMapDataPair.insert({ std::make_pair(point.GetNodeKey().GetX(), point.GetNodeKey().GetZ()), point.GetY() });
        }
        else    // exist
        {
            float beforeHeight = mMapDataPair.find(std::make_pair(point.GetNodeKey().GetX(), point.GetNodeKey().GetZ()))->second;
            if (beforeHeight > point.GetY())
            {
                mMapDataPair.find(std::make_pair(point.GetNodeKey().GetX(), point.GetNodeKey().GetZ()))->second = point.GetY();
            }
        }
    }

    void MapDataNode::MakeMapToPoints()                    // <float, float>, float> | <<x,z>, y>
    {
        for (auto iter = mMapDataPair.begin(); iter != mMapDataPair.end(); ++iter)
        {
            Point3 pointXYZ = { iter->first.second, -(iter->first.first) , -(iter->second) };
            mOutputPoints.push_back(pointXYZ);
        }
    }

    void MapDataNode::FromPCD(const std::string& inputPath)
    {
        mInputPoints.reserve(307200);

        std::ifstream fin;
        fin.open(inputPath);
        std::string line;

        if (fin.is_open())
        {
            int num = 1;
            while (!fin.eof())
            {
                getline(fin, line);
                if (num > 10)
                {
                    float x, y, z;
                    std::istringstream iss(line);
                    iss >> x >> y >> z;

                    Point3 pointXYZ = { x, y, z };
                    mInputPoints.push_back(pointXYZ);
                }
                num++;
            }
        }
        fin.close();
    }

    void MapDataNode::ToPCD(const std::string& outputPath)
    {
        MakeMapToPoints();

        time_t t;
        struct tm* timeinfo;
        time(&t);
        timeinfo = localtime(&t);

        std::string hour, min;

        if (timeinfo->tm_hour < 10)
        {
            hour = "0" + std::to_string(timeinfo->tm_hour);
        }
        else
        {
            hour = std::to_string(timeinfo->tm_hour);
        }

        if (timeinfo->tm_min < 10)
        {
            min = "0" + std::to_string(timeinfo->tm_min);
        }
        else
        {
            min = std::to_string(timeinfo->tm_min);
        }

        std::string filePath = outputPath + hour + min + ".pcd";

        std::ofstream fout;
        fout.open(filePath);

        fout << "VERSION" << std::endl;
        fout << "FIELDS x y z" << std::endl;
        fout << "SIZE 4 4 4" << std::endl;
        fout << "TYPE F F F" << std::endl;
        fout << "COUNT 1 1 1" << std::endl;
        fout << "WIDTH 1" << std::endl;
        fout << "HEIGHT " << mOutputPoints.size() << std::endl;
        fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
        fout << "POINTS " << mOutputPoints.size() << std::endl;
        fout << "DATA ascii" << std::endl;

        for (int i = 0; i < mOutputPoints.size(); i++)
        {
            fout << mOutputPoints[i].GetX() << " " << mOutputPoints[i].GetY() << " " << mOutputPoints[i].GetZ() << "\n";
        }

        fout.close();
    }


}
