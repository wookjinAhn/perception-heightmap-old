//
// Created by wj on 22. 4. 19.
//

//#include "../include/heightmap_core/Point2.h"
#include "heightmap_core/Point2.hpp"

namespace camel
{
    Point2::Point2()
        : camelVector::Point2D()
    {
    }

	Point2::Point2(float x, float y)
        : camelVector::Point2D(x, y)
    {
    }
}