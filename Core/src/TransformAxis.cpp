#include "TransformAxis.h"
#include "Constant.h"
#include "Simm_spline.h"
#include "Linear_function.h"


TransformAxis::TransformAxis()
{
	set_axis(0, 0, 0);

	set_function();
}

TransformAxis::~TransformAxis()
= default;

TransformAxis::TransformAxis(const TransformAxis& aTransformAxis)
{
	m_axis = aTransformAxis.m_axis;

	m_coordinate_dependance = aTransformAxis.m_coordinate_dependance;

	m_relation = aTransformAxis.m_relation;
}

void TransformAxis::set_coordinate(Coordinate* p_coordinate)
{
	m_coordinate_dependance = p_coordinate;
}

void TransformAxis::set_axis(const Vector3d& axis)
{
	m_axis = axis;
}

void TransformAxis::set_axis(double x, double y, double z)
{
	m_axis = {x, y, z};
}

void TransformAxis::set_function(TransformAxis_from_xml input_axe)
{
	if (input_axe.function == "constant")
	{
		m_relation = std::make_shared<Constant>(input_axe.value);
	}
	if (input_axe.function == "simm_spline")
	{
		m_relation = std::make_shared<Simm_spline>(input_axe.spline_x, input_axe.spline_y);
	}
	if (input_axe.function == "linear_function")
	{
		m_relation = std::make_shared<Linear_function>(input_axe.coef);
	}
}

void TransformAxis::set_function()
{
	m_relation = std::make_unique<Constant>(0.0);
}

Coordinate* TransformAxis::get_coordinate()
{
	return m_coordinate_dependance;
}
