//
// Created by wj on 22. 4. 19.
//

#ifndef OLDMAP_BOUNDINGBOX_HPP
#define OLDMAP_BOUNDINGBOX_HPP

#include "Point3.hpp"

namespace camel
{

class BoundingBox
{
public:
	BoundingBox();
	BoundingBox(float x, float z, float w, float h);
//        Boundary(float minX, float maxX, float y);
	BoundingBox(float minX, float maxX, float z);

	float GetX() const;
	float GetZ() const;
	float GetW() const;
	float GetH() const;

	void SetBoundary(float x, float z, float w, float h);

	bool IsConstained(const Point3& p) const;

private:
	float mX, mZ, mW, mH;
};

}

#endif //OLDMAP_BOUNDINGBOX_HPP
