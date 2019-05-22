#pragma once
#include <map>
#include "MathLib/src/Vector3d.h"

/**
	the application point is defined in local coordinates but the F vector is defined in wolrd coordinates
*/

struct Force_struct
{
	Point3d pt;
	Vector3d F;

	Force_struct() {
		pt = Point3d(0, 0, 0);
		F = Vector3d(0, 0, 0);
	}

};