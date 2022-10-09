//
// Created by wj on 22. 4. 19.
//

#ifndef OLDMAP_MAPDATANODEROS_HPP
#define OLDMAP_MAPDATANODEROS_HPP

//#include "../../../heightmap_core/include/heightmap_core/heightmap_core.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <oldmap_msgs/Oldmap.h>
#include "oldmap_core/oldmap_core.hpp"

namespace camel
{

class MapDataNodeROS : public camel::MapDataNode
{
public:
	MapDataNodeROS();
	~MapDataNodeROS();

	void SetRotateDegree(double rotateDegree);
	double GetRotateDegree() const;
	void SetRotateRadian(double rotateRadian);
	double GetRotateRadian() const;
    void SetDataRange(float w, float h);
    void SetDataRange(BoundingBox dataRange);
    BoundingBox GetDataRange() const;

    void MakeMapToPointsWithOldData();

    void FromMessage(sensor_msgs::PointCloud pointcloud_msg);
	void ToMessage(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud);
	void FromMessagePointCloud2(sensor_msgs::PointCloud2 pointcloud2_msgs);
	void ToMessagePointCloud2(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud2);
	void ToMessageHeightmapmsgs(std::string frame_id, oldmap_msgs::OldmapConstPtr& output_msgs);
	void ToPointCloud2Msgs();
	void fromPointCloud2Msgs();
	void ToHeightmapMsgs(oldmap_msgs::Oldmap& output_heightmap, float cameraHeight, camel::Point3& odom);
    void UpdateOldDataByOdom(camel::Point3& odomPosition);

private:

    std::unordered_map<std::pair<float, float>, float, pair_hash> mOldMapData;

    std::vector<Point3> mPastData;
    std::vector<Point3> mOutputPoints;
	float mRotateDegree;
    BoundingBox mPositionRange;
};

}

#endif //OLDMAP_MAPDATANODEROS_HPP
