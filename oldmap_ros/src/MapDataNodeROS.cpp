//
// Created by wj on 22. 4. 19.
//

#include "oldmap_ros/MapDataNodeROS.hpp"

namespace camel
{
    MapDataNodeROS::MapDataNodeROS()
        : camel::MapDataNode()
    {
        BoundingBox positionRange(0, 0, 1.0f, 1.0f);
        mPositionRange = positionRange;
        std::cout << "positionRange : " << mPositionRange.GetX() << ", " << mPositionRange.GetZ() << ", " << mPositionRange.GetH() << ", " << mPositionRange.GetW() << std::endl;
    }

    MapDataNodeROS::~MapDataNodeROS()
    {
    }

    void MapDataNodeROS::SetRotateDegree(double rotateDegree)
    {
        mRotateDegree = rotateDegree;
    }

    double MapDataNodeROS::GetRotateDegree() const
    {
        return mRotateDegree;
    }

    void MapDataNodeROS::SetRotateRadian(double rotateRadian)
    {
        mRotateDegree = rotateRadian * R2D;
    }

    double MapDataNodeROS::GetRotateRadian() const
    {
        return mRotateDegree * D2R;
    }

    void MapDataNodeROS::SetDataRange(float w, float h)
    {
        BoundingBox dataRange(0, 0, w, h);
        mPositionRange = dataRange;
    }

    void MapDataNodeROS::SetDataRange(BoundingBox dataRange)
    {
        mPositionRange = dataRange;
    }

    BoundingBox MapDataNodeROS::GetDataRange() const
    {
        return mPositionRange;
    }

    void MapDataNodeROS::FromMessage(sensor_msgs::PointCloud pointcloud_msg)
    {
        for (int i = 0; i < pointcloud_msg.points.size(); i++)
        {
            Point3 pointXYZ = { pointcloud_msg.points[i].x, pointcloud_msg.points[i].y, pointcloud_msg.points[i].z };
            SetInputPoint(pointXYZ);
        }
    }

    void MapDataNodeROS::ToMessage(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud)        //camera1_depth_optical_frame
    {
        MakeMapToPoints();

        output_pointcloud.header.frame_id = frame_id;           // header
        output_pointcloud.header.stamp = ros::Time::now();
        output_pointcloud.points.resize(GetOutputPoints().size());     // points			//************************

        for (int i = 0; i < output_pointcloud.points.size(); i++)
        {
            output_pointcloud.points[i].x = GetOutputPoints()[i].GetX();                    //************************
            output_pointcloud.points[i].y = GetOutputPoints()[i].GetY();                    //************************
            output_pointcloud.points[i].z = GetOutputPoints()[i].GetZ();                    //************************
        }

    }

    void MapDataNodeROS::FromMessagePointCloud2(sensor_msgs::PointCloud2 pointcloud2_msgs)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud2_msgs, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud2_msgs, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud2_msgs, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // Check if the point is invalid
            if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
            {
                Point3 pointXYZ = { *iter_x, *iter_y, *iter_z };
                SetInputPoint(pointXYZ);
            }
        }
    }

    void MapDataNodeROS::ToMessagePointCloud2(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud2)
    {
        std::cout << "OutputPoints : " << GetOutputPoints().size() << std::endl;

        sensor_msgs::PointCloud pointcloud_msgs;

        pointcloud_msgs.header.frame_id = frame_id;
        pointcloud_msgs.header.stamp = ros::Time::now();
        pointcloud_msgs.points.resize(GetOutputPoints().size());

        for (int i = 0; i < pointcloud_msgs.points.size(); i++)
        {
            Point3 outData = GetOutputPoints()[i];
            pointcloud_msgs.points[i].x = outData.GetX();
            pointcloud_msgs.points[i].y = outData.GetY();
            pointcloud_msgs.points[i].z = outData.GetZ();
        }
        sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msgs, output_pointcloud2);
    }

    void MapDataNodeROS::ToHeightmapMsgs(std::string frame_id, oldmap_msgs::Oldmap& heightmap_msgs, float cameraHeight)
    {
        std::cout << "ToHeightmapMsgs" << std::endl;
        std::vector<Point3> output = GetOutputPoints();
        for (auto& iter : output)
        {
            float x = -(iter.GetY());
            float y = -(iter.GetZ());
            float z = iter.GetX();

            iter.SetXYZ(x, y, z);
        }

        std::cout << "ToHeightmapMsgs : to msg" << std::endl;

        heightmap_msgs.header.frame_id = frame_id;           // header
        heightmap_msgs.header.stamp = ros::Time::now();
        heightmap_msgs.resolution = 0.03;
        heightmap_msgs.points.resize(output.size());

        for (int i = 0; i < output.size(); i++)
        {
            Point3 outData = output[i];
            heightmap_msgs.points[i].x = outData.GetX();
            heightmap_msgs.points[i].y = outData.GetY() - (GetDefaultCameraPosition()[0]);
            heightmap_msgs.points[i].z = outData.GetZ();
        }
    }
}
