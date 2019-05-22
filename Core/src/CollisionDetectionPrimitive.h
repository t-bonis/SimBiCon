#pragma once
#include "Model.h"

/*========================================================================================================================================================================*
 * This class implements an interface for collision detection primitives such as spheres, capsules and so on.                                                             *
 *========================================================================================================================================================================*/
class CollisionDetectionPrimitive
{
public:
	enum type { unknown_cdp, sphere_cdp, capsule_cdp, plane_cdp, box_cdp, tri_mesh_cdp };

public:
	CollisionDetectionPrimitive() = default;
	CollisionDetectionPrimitive(const CollisionDetectionPrimitive& other);

	virtual ~CollisionDetectionPrimitive() = default;

	CollisionDetectionPrimitive(CollisionDetectionPrimitive&& other) = delete;
	CollisionDetectionPrimitive& operator=(const CollisionDetectionPrimitive& other) = delete;
	CollisionDetectionPrimitive& operator=(CollisionDetectionPrimitive&& other) = delete;

	//	returns the type of this collision detection primitive.
	type get_type() const { return type; }

	std::shared_ptr<Model> get_model() const { return m_model; }

protected:
	type type{unknown_cdp};
	std::shared_ptr<Model> m_model{ nullptr };
};

inline CollisionDetectionPrimitive::CollisionDetectionPrimitive(const CollisionDetectionPrimitive& other)
{
	type = other.type;
	m_model = std::make_shared<Model>(*other.get_model());

}
