//
// Created by wj on 22. 4. 19.
//

#ifndef OLDMAP_MAPDATANODE_HPP
#define OLDMAP_MAPDATANODE_HPP

#include <fstream>        // file oi (std::ifstream)
#include <iostream>
#include <random>        // sampling
#include <sstream>        // file oi (std::istringstream)
#include <string>        // data path

// STL
#include <algorithm>    // std::shuffle
#include <vector>
#include <map>
#include <unordered_map>

#include "Point3.hpp"
#include "BoundingBox.hpp"

namespace camel
{

    struct pair_hash
    {
        template<class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& pair) const
        {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    class MapDataNode
    {
    public:
        MapDataNode();
        ~MapDataNode();

        double GetResolution() const;
        void SetResolution(double resolution);
        void SetResolutionByBoundingBox(const BoundingBox& boundingBox, int depth);
        void SetResolutionByBoundingBox(int depth);

        BoundingBox GetBoundingBox() const;
        void SetBoundingBox(BoundingBox boundingBox);

        std::unordered_map<std::pair<float, float>, float, pair_hash> GetMapDataPair() const;

        std::vector<Point3> GetInputPoints() const;
        void SetInputPoint(Point3 point);

        std::vector<Point3> GetOutputPoints() const;

        void SetDefaultCameraPosition(float t265PoseZ, float d435PoseX, float d435PoseY, float d435PoseZ);
        void SetCurrentOdom(float poseX, float poseY, float poseZ, float quaternionX, float quaternionY, float quaternionZ, float quaternionW);
        float* GetDefaultCameraPosition();
        float* GetCurrentOdom();

        std::vector<Point3> SamplingPointsWithRotate(int samplingNum, float rotationDegree);
        std::vector<Point3> SamplingPoints(int samplingNum);
        void MakeHeightMap(Point3& point);
        void MakeMapToPoints();        // at ToPCD

        void FromPCD(const std::string& inputPath);
        void ToPCD(const std::string& outputPath);

    private:
        double mResolution;
        BoundingBox mBoundingBox;

        float mCurrentOdom[7];
        float mCameraPosition[4];

//	std::unordered_map<std::pair<float, float>, float> mMapDataPair; // not allowed. ref : https://www.techiedelight.com/use-std-pair-key-std-unordered_map-cpp/
        std::unordered_map<std::pair<float, float>, float, pair_hash> mMapDataPair;
//	std::map<std::pair<float, float>, float> mMapDataPair;

        std::vector<Point3> mInputPoints;
        std::vector<Point3> mOutputPoints;
    };

}

#endif //OLDMAP_MAPDATANODE_HPP
