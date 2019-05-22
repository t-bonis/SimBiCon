#pragma once
#include <array>
#include <assimp/scene.h>
#include <QtGui/QOpenGLShaderProgram>
#include <QtGui/QOpenGLVertexArrayObject>
#include <QtGui/QOpenGLBuffer>
#include "Observer_interface.h"

class My_qopengl_widget;
class Rigid_body;

struct Texture {
	unsigned int id{ 0 };
	std::string type;
	std::string path;
};

struct Vertex {
	std::array<float, 3> position{ 0,0,0 };
	std::array<float, 3> normal{ 0,0,0 };
	std::array<float, 2> tex_coords{ 0,0 };
};

class Mesh
{
public:
	Mesh() = default;
	Mesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, std::vector<Texture>& textures);
	Mesh(const Mesh& other);

	~Mesh() = default;

	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	std::vector<Texture> textures;

	My_qopengl_widget* q_opengl_widget{ nullptr };

	QVector3D color{ 1.0, 1.0, 1.0 };

	/*  Render data  */
	QOpenGLVertexArrayObject vao;
	QOpenGLBuffer vbo{ QOpenGLBuffer::VertexBuffer };
	QOpenGLBuffer ibo{ QOpenGLBuffer::IndexBuffer };

	/*  Functions    */
	void setup_mesh(My_qopengl_widget& q_opengl_widget);
};

class Model : public Observer_interface
{
public:
	Model() = default;
	Model(const Model& other);

	~Model() = default;


	Model(Model&& other) = delete;
	Model& operator=(const Model& other) = delete;
	Model& operator=(Model&& other) = delete;

	void add_box(Vector3d min, const Vector3d& max);
	void add_sphere(float radius);
	void add_cylinder(float radius, float length);
	void load_from_file(const std::string& path);
	void process_node(aiNode* node, const aiScene* scene);
	void process_mesh(aiMesh* mesh, const aiScene* scene);
	//std::vector<Texture> load_material_textures(aiMaterial* mat, aiTextureType type, std::string type_name);
	//unsigned texture_from_file(const char* path, const std::string& directory, bool gamma = false);
	void translate_vertex(const Vector3d& vector);
	void set_position(const Point3d& position);
	void set_rotation(const Quaternion& orientation);
	void set_color(const std::array<float, 3>& a_color);
	void set_scale(const std::array<float, 3>& a_scale);

	void scale_meshes(const Vector3d& vector3_d);
	void draw();
	void draw_wire();

	void update(const Subject_interface* observable);
	std::vector<Mesh> get_meshes() const;
	void add_meshes(std::vector<Mesh> meshes);
	std::vector<Texture> textures_loaded;
	std::vector<Mesh> meshes;
	std::string directory;

	Rigid_body* linked_body{nullptr};
	QMatrix4x4 model_view_matrix;
	QVector3D position;
	QQuaternion orientation;
	QVector3D scale = {1,1,1};

	bool displayed = true;
};
