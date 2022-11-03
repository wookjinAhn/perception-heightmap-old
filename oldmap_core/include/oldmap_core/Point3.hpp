//
// Created by wj on 22. 4. 19.
//

#ifndef OLDMAP_POINT3D_H
#define OLDMAP_POINT3D_H

#include <iostream>
#include <cmath>    // std::sin, std::cos
#include <camel-euclid/Vector.hpp>

#include "Point2.hpp"

const double PI = 3.14159265359;
const double D2R = PI / 180;
const double R2D = 180 / PI;

namespace camel
{

    class Point3 : public camelVector::Point3D
    {
    public:
        Point3();
        Point3(float x, float y, float z);

        Point2 GetNodeKey() const;
        Point2 GetCentroid() const;

        void SetNodeKey(const Point2& nodeKey);
        void SetNodeKeyXZ(float const x, float const z);
        void SetCentroid(const Point2& centroid);

        void RotationRoll(float degree);

        float DistanceBetween2D(const Point2& other);

    private:
        Point2 mNodeKey;
        Point2 mCentroid;
    };

}

#endif //OLDMAP_POINT3D_H
