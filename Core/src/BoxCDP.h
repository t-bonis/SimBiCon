#pragma once
#include "MathLib/src/mathLib.h"
#include "MathLib/src/Point3d.h"
#include "Utils/src/Utils.h"
#include "CollisionDetectionPrimitive.h"



/*========================================================================================================================================================================*
 * This class implements a rectangular box class that will be used as a collision detection primitive.                                                                    *
 * A box is represented by the position of two opposite corners.                                                                                                          *
 *========================================================================================================================================================================*/
class BoxCDP : public CollisionDetectionPrimitive
{
public:
	BoxCDP(Point3d& p1, Point3d& p2);
	BoxCDP(const BoxCDP& other);
	BoxCDP();

	~BoxCDP() = default;

	BoxCDP(BoxCDP&& other) = delete;
	BoxCDP& operator=(const BoxCDP& other) = delete;
	BoxCDP& operator=(BoxCDP&& other) = delete;
	

	//return the center of the box, expressed in local coordinates
	Point3d getCenter() const
	{
		return Point3d((m_p1.x + m_p2.x) / 2, (m_p1.y + m_p2.y) / 2, (m_p1.z + m_p2.z) / 2);
	}

	//returns the length in the x-direction
	double getXLen() const
	{
		return (fabs(m_p1.x - m_p2.x));
	}

	//returns the length in the y-direction
	double getYLen() const
	{
		return (fabs(m_p1.y - m_p2.y));
	}

	//returns the length in the z-direction
	double getZLen() const
	{
		return (fabs(m_p1.z - m_p2.z));
	}

private:
	//these are the two corners of the box, expressed in local coordinates.
	Point3d m_p1{ 0, 0, 0 }, m_p2{ 0, 0, 0 };
};
