#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "Model.h"
#include "My_qopengl_widget.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>


Mesh::Mesh(const Mesh& other)
{
	this->vertices = other.vertices;
	this->indices = other.indices;
	this->textures = other.textures;
}

Mesh::Mesh(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, std::vector<Texture>& textures)
{
	this->vertices = vertices;
	this->indices = indices;
	this->textures = textures;
}

void Mesh::setup_mesh(My_qopengl_widget& q_opengl_widget)
{
	this->q_opengl_widget = &q_opengl_widget;
	vao.create();
	vao.bind();
	
	vbo.create();
	vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
	vbo.bind();
	vbo.allocate(vertices.data(), int(vertices.size() * sizeof(Vertex)));

	ibo.create();
	ibo.setUsagePattern(QOpenGLBuffer::StaticDraw);
	ibo.bind();
	ibo.allocate(indices.data(), int(indices.size() * sizeof(unsigned int)));
	

	q_opengl_widget.m_program.enableAttributeArray("vertexPosition");
	q_opengl_widget.m_program.setAttributeBuffer("vertexPosition", GL_FLOAT, 0,
	                                             3, sizeof(Vertex));

	q_opengl_widget.m_program.enableAttributeArray("vertexNormal");
	q_opengl_widget.m_program.setAttributeBuffer("vertexNormal", GL_FLOAT, 3 * sizeof(float),
	                                             3, sizeof(Vertex));

	q_opengl_widget.m_program.enableAttributeArray("texCoord2d");
	q_opengl_widget.m_program.setAttributeBuffer("texCoord2d", GL_FLOAT, 6 * sizeof(float),
												2, sizeof(Vertex));

	vao.release();
}

Model::Model(const Model& other)
{
	for (const auto& texture : other.textures_loaded)
	{
		textures_loaded.push_back(texture);
	}

	for (const auto& mesh : other.meshes)
	{
		auto temp_mesh(mesh);
		meshes.push_back(temp_mesh);
	}

	directory = other.directory;

	model_view_matrix = other.model_view_matrix;
	position = other.position;
	orientation = other.orientation;
}

void Model::draw()
{
	model_view_matrix.setToIdentity();
	model_view_matrix.translate(position);
	model_view_matrix.rotate(orientation);
	model_view_matrix.scale(scale);

	for(auto& mesh : meshes)
	{
		QMatrix4x4 mv = mesh.q_opengl_widget->m_view * model_view_matrix;

		mesh.q_opengl_widget->m_program.setUniformValue("modelViewMatrix", mv);
		mesh.q_opengl_widget->m_program.setUniformValue("normalMatrix", mv.normalMatrix());
		mesh.q_opengl_widget->m_program.setUniformValue("projectionMatrix", mesh.q_opengl_widget->m_projection);
		mesh.q_opengl_widget->m_program.setUniformValue("vertexColor", mesh.color);

		mesh.vao.bind();
		glDrawArrays(GL_TRIANGLES, 0, GLsizei(mesh.indices.size()));
		mesh.vao.release();
	}	
}

void Model::draw_wire()
{
	model_view_matrix.setToIdentity();
	model_view_matrix.translate(position);
	model_view_matrix.rotate(orientation);
	model_view_matrix.scale(scale);

	for (auto& mesh : meshes)
	{
		QMatrix4x4 mv = mesh.q_opengl_widget->m_view * model_view_matrix;

		mesh.q_opengl_widget->m_program.setUniformValue("modelViewMatrix", mv);
		mesh.q_opengl_widget->m_program.setUniformValue("normalMatrix", mv.normalMatrix());
		mesh.q_opengl_widget->m_program.setUniformValue("projectionMatrix", mesh.q_opengl_widget->m_projection);
		mesh.q_opengl_widget->m_program.setUniformValue("vertexColor", mesh.color);

		mesh.vao.bind();
		glDrawArrays(GL_LINE_STRIP, 0, GLsizei(mesh.indices.size()));
		mesh.vao.release();
	}
}

void Model::update(const Subject_interface* observable)
{
	if(const auto rigid_body = dynamic_cast<Rigid_body const*>(observable))
	{
		set_position(rigid_body->get_cm_position());
		set_rotation(rigid_body->get_orientation());
	}
}

std::vector<Mesh> Model::get_meshes() const
{
	return meshes;
}

void Model::add_meshes(std::vector<Mesh> meshes)
{
	for(auto& mesh : meshes)
	{
		this->meshes.emplace_back(mesh.vertices, mesh.indices, mesh.textures);
	}
}

void Model::add_box(Vector3d min, const Vector3d& max)
{
	// load a box of size (1,1,1)
	load_from_file("../data/models/box111.obj");
	// resize to max - min
	const auto scale_factor = max - min;
	scale_meshes(scale_factor);
	translate_vertex(-min);
	
}


void Model::add_sphere(float radius)
{
	// load a sphere of size (1)
	//resize it
}

void Model::add_cylinder(float radius, float length)
{
	//load cylinder of size (1) radius (1)
	//resize it
}

void Model::scale_meshes(const Vector3d& vector3_d)
{
	for (auto& mesh : meshes)
	{
		for (auto& vertex : mesh.vertices)
		{
			vertex.position[0] = vertex.position[0] * vector3_d.x;
			vertex.position[1] = vertex.position[1] * vector3_d.y;
			vertex.position[2] = vertex.position[2] * vector3_d.z;
		}
	}
}

void Model::load_from_file(const std::string& path)
{
	//gl_functions = QOpenGLContext::currentContext()->functions();
	// read file via ASSIMP
	Assimp::Importer importer;
	importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiComponent_NORMALS);
	const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace);
	// check for errors
	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
	{
		BOOST_LOG_TRIVIAL(error) << "ERROR::ASSIMP:: " << importer.GetErrorString();
		return;
	}
	// retrieve the directory path of the filepath
	directory = path.substr(0, path.find_last_of('/'));

	// process ASSIMP's root node recursively
	process_node(scene->mRootNode, scene);
}

void Model::process_node(aiNode *node, const aiScene *scene)
{
	// process all the node's meshes (if any)
	for (unsigned int i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
		process_mesh(mesh, scene);
	}
	// then do the same for each of its children
	for (unsigned int i = 0; i < node->mNumChildren; i++)
	{
		process_node(node->mChildren[i], scene);
	}
}

void Model::process_mesh(aiMesh *mesh, const aiScene *scene)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	std::vector<Texture> textures;

	for (unsigned int i = 0; i < mesh->mNumVertices; i++)
	{
		Vertex vertex;
		// positions
		vertex.position[0] = mesh->mVertices[i].x;
		vertex.position[1] = mesh->mVertices[i].y;
		vertex.position[2] = mesh->mVertices[i].z;
		// normals
		vertex.normal[0] = mesh->mNormals[i].x;
		vertex.normal[1] = mesh->mNormals[i].y;
		vertex.normal[2] = mesh->mNormals[i].z;
		// texture coordinates
		if (mesh->mTextureCoords[0]) // does the mesh contain texture coordinates?
		{
			// a vertex can contain up to 8 different texture coordinates. We thus make the assumption that we won't 
			// use models where a vertex can have multiple texture coordinates so we always take the first set (0).
			vertex.tex_coords[0] = mesh->mTextureCoords[0][i].x;
			vertex.tex_coords[1] = mesh->mTextureCoords[0][i].y;
		}
		else
			vertex.tex_coords = {0,0};
		vertices.push_back(vertex);
	}

	// now walk through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
	for (unsigned int i = 0; i < mesh->mNumFaces; i++)
	{
		const auto face = mesh->mFaces[i];
		// retrieve all indices of the face and store them in the indices vector
		for (unsigned int j = 0; j < face.mNumIndices; j++)
		{
			indices.push_back(face.mIndices[j]);
		}
	}



	meshes.emplace_back(vertices, indices, textures);
}

void Model::translate_vertex(const Vector3d& vector)
{
	for (auto& mesh : meshes)
	{
		for (auto& vertex : mesh.vertices)
		{
			vertex.position[0] -= vector.x;
			vertex.position[1] -= vector.y;
			vertex.position[2] -= vector.z;
		}
	}
}


void Model::set_position(const Point3d& position)
{
	this->position = QVector3D(position.x, position.y, position.z);
}

void Model::set_rotation(const Quaternion& orientation)
{
	this->orientation = QQuaternion(orientation.s, orientation.v.x, orientation.v.y, orientation.v.z);
}

void Model::set_color(const std::array<float, 3>& a_color)
{
	for (auto& mesh : meshes)
	{
		mesh.color = QVector3D(a_color[0], a_color[1], a_color[2]);
	}
}

void Model::set_scale(const std::array<float, 3>& a_scale)
{
	this->scale = QVector3D(a_scale[0], a_scale[1], a_scale[2]);
}
