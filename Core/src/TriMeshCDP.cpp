#include "TriMeshCDP.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


Tri_mesh_cdp::Tri_mesh_cdp(const std::string& file, const A_RigidBody& a_rigid_body)
{
	type = tri_mesh_cdp;
	if (!load_from_file(file, a_rigid_body))
	{
		throw std::logic_error("Invalid file");
	}
	m_model = std::make_shared<Model>();
	m_model->load_from_file(file);
	m_model->scale_meshes(Vector3d(a_rigid_body.scale_factor[0], a_rigid_body.scale_factor[1],
		a_rigid_body.scale_factor[2]));
	m_model->translate_vertex(Vector3d(a_rigid_body.center_of_mass[0], a_rigid_body.center_of_mass[1],
		a_rigid_body.center_of_mass[2]));
}

Tri_mesh_cdp::Tri_mesh_cdp(const Tri_mesh_cdp& other) : CollisionDetectionPrimitive(other)
{
	vertices = other.vertices;
	indices = other.indices;
}

Tri_mesh_cdp::Tri_mesh_cdp()
{
	type = tri_mesh_cdp;
}

bool Tri_mesh_cdp::load_from_file(const std::string& file, const A_RigidBody& a_rigid_body)
{
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(file, 0);

	if (!scene)
	{
		return false;
	}

	if (scene->HasMeshes())
	{
		for (unsigned int i = 0; i < scene->mNumMeshes; i++)
		{
			for (unsigned int j = 0; j < scene->mMeshes[i]->mNumVertices; j++)
			{
				const auto position = scene->mMeshes[i]->mVertices[j];
				vertices.push_back(position.x);
				vertices.push_back(position.y);
				vertices.push_back(position.z);
			}

			for (unsigned int j = 0; j < scene->mMeshes[i]->mNumFaces; j++)
			{
				const auto face = scene->mMeshes[i]->mFaces[j];
				for (unsigned int k = 0; k < face.mNumIndices; k++)
				{
					indices.push_back(face.mIndices[k]);
				}
			}
		}
		scale_meshes(Vector3d(a_rigid_body.scale_factor[0], a_rigid_body.scale_factor[1],
			a_rigid_body.scale_factor[2]));
		translate_vertex(Vector3d(a_rigid_body.center_of_mass[0], a_rigid_body.center_of_mass[1],
			a_rigid_body.center_of_mass[2]));
	}
	return true;
}

void Tri_mesh_cdp::scale_meshes(const Vector3d& vector3_d)
{
	for (unsigned int i = 0; i < vertices.size(); i += 3)
	{
		vertices[i] *= vector3_d.x;
		vertices[i + 1] *= vector3_d.y;
		vertices[i + 2] *= vector3_d.z;
	}
}

void Tri_mesh_cdp::translate_vertex(const Vector3d& vector)
{
	for (unsigned int i = 0; i < vertices.size(); i += 3)
	{
		vertices[i] -= vector.x;
		vertices[i + 1] -= vector.y;
		vertices[i + 2] -= vector.z;
	}
}
