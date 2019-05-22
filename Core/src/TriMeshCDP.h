#pragma once
#include "CollisionDetectionPrimitive.h"
#include "Utils/src/Utils.h"
#include "Utils/src/SerializationUtils.h"

class Tri_mesh_cdp :
	public CollisionDetectionPrimitive
{
public:
	Tri_mesh_cdp(const std::string& file, const A_RigidBody& a_rigid_body);
	Tri_mesh_cdp(const Tri_mesh_cdp& other);
	Tri_mesh_cdp();

	~Tri_mesh_cdp() = default;
	
	Tri_mesh_cdp(Tri_mesh_cdp&& other) = delete;
	Tri_mesh_cdp& operator=(const Tri_mesh_cdp& other) = delete;
	Tri_mesh_cdp& operator=(Tri_mesh_cdp&& other) = delete;

private:
	bool load_from_file(const std::string& file, const A_RigidBody& a_rigid_body);
	void scale_meshes(const Vector3d& vector3_d);
	void translate_vertex(const Vector3d& vector);

public:
	std::vector<float> vertices;
	std::vector<unsigned int> indices;
};

