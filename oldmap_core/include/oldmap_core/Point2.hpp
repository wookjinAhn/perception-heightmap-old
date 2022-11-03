//
// Created by wj on 22. 4. 19.
//

#ifndef OLDMAP_POINT2_HPP
#define OLDMAP_POINT2_HPP

#include <camel-euclid/Vector.hpp>

namespace camel
{

    class Point2 : public camelVector::Point2D
    {
    public:
        Point2();
        Point2(float x, float z);
    };

}

#endif //OLDMAP_POINT2_HPP
