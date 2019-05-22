#pragma once
#include <QtWidgets/QOpenGLWidget>
#include <QtCore/QTime>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLFramebufferObject>

#include "Model.h"
#include "ArticulatedRigidBody.h"
#include "QtGui.h"


class SimBiCon_framework;

class My_qopengl_widget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	My_qopengl_widget() = delete;
	void fill_meshes_storage();
	explicit My_qopengl_widget(Qt_gui& gui, SimBiCon_framework& conF, QWidget* parent = nullptr);

	~My_qopengl_widget();

	My_qopengl_widget(const My_qopengl_widget& other) = delete;
	My_qopengl_widget(My_qopengl_widget&& other) = delete;
	My_qopengl_widget& operator=(const My_qopengl_widget& other) = delete;
	My_qopengl_widget& operator=(My_qopengl_widget&& other) = delete;

	void add_drawable_objects(std::vector<std::shared_ptr<Articulated_rigid_body>> articulatedRBs);
	void add_desired_pose(std::vector<std::shared_ptr<Articulated_rigid_body>> articulatedRBs);

	void add_collision_primitive_box(std::vector<std::shared_ptr<Articulated_rigid_body>> articulatedRBs);
	void add_sphere(Vector3d pos, std::array<float, 3> color = { 0, 0, 0 }, std::array<float, 3> scale = { 0.01f,0.01f,0.01f });
	void add_arrow(Vector3d origin, Quaternion orientation, double length);
	void clear_temp_object();
	void add_ground(const std::shared_ptr<Rigid_body>& ground);
	void init_some_sphere();
	void init_some_arrow();
	void del_desired_pose();
	void del_skeletal();
	void del_collision_primitive_box();

private slots:
signals:
	void key_d_pressed();

protected:
	void initializeGL() override;
	void resizeGL(int w, int h) override;
	void paintGL() override;
	void keyPressEvent(QKeyEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;

public:
	SimBiCon_framework* con_f;

	Qt_gui* gui;

	std::vector<std::pair<std::string, std::vector<Mesh>>> meshes_storage;

	std::vector<Model*> m_skeletal;
	std::vector<Model*> m_CDPs;
	std::vector<Model*> m_desired_pose;
	std::vector<std::shared_ptr<Model>> m_temp_sphere;
	std::vector<std::shared_ptr<Model>> m_temp_arrows;
	Model* m_ground_plane{ nullptr };

	QOpenGLShaderProgram m_program;
	std::shared_ptr<QOpenGLFramebufferObject> m_fbo;

	QMatrix4x4 m_projection;

	QMatrix4x4 m_view;

	QPoint m_last_pos;
	QVector3D m_cam_eye{ 0, 4, 4 };
	QVector3D m_cam_center{ 0, 0, 0 };
	QVector3D m_cam_up{ 0, 1, 0 };



	int m_frames{ 0 };
	QTime m_time;
};
