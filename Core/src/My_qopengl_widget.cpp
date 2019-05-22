#include "My_qopengl_widget.h"
#include "SimBiCon_framework.h"
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <utility>
#include <QtCore/qmath.h>
#include "Ode_world.h"

static QString vertexShader =
"#version 450\n"
"attribute vec3 vertexPosition;\n"
"attribute vec3 vertexNormal;\n"
"attribute vec2 texCoord2d;\n"
"\n"
"uniform mat4 modelViewMatrix;\n"
"uniform mat3 normalMatrix;\n"
"uniform mat4 projectionMatrix;\n"
"uniform vec3 vertexColor;\n"
"\n"
"struct LightSource\n"
"{\n"
"    vec3 ambient;\n"
"    vec3 diffuse;\n"
"    vec3 position;\n"
"};\n"
"uniform LightSource lightSource;\n"
"\n"
"varying vec3 v_color;\n"
"varying vec2 v_texCoord2d;\n"
"\n"
"void main()\n"
"{\n"
"	vec3 normal		= normalize(normalMatrix * vertexNormal);\n"
"	vec3 position	= vec3(modelViewMatrix * vec4(vertexPosition, 1));\n"
"   float nDotVP	= dot(normal, normalize(lightSource.position));\n"
"\n"
"    vec3 ambient    = lightSource.ambient;\n"
"    vec3 diffuse    = lightSource.diffuse * nDotVP;\n"
"    v_color = clamp(ambient  * vertexColor +\n"
"                    diffuse  * vertexColor, 0.0 , 1.0 );\n"
"\n"
"    v_texCoord2d = texCoord2d;\n"
"\n"
"	gl_Position = projectionMatrix * modelViewMatrix * vec4(vertexPosition, 1);\n"
"}\n"
;

static QString fragmentShader =
"#version 450\n"
"\n"
"uniform sampler2D texUnit;\n"
"\n"
"varying vec3 v_color;\n"
"varying vec2 v_texCoord2d;\n"
"\n"
"void main()\n"
"{\n"
"	gl_FragColor = vec4(v_color, 1);\n"
"}\n"
;


void My_qopengl_widget::fill_meshes_storage()
{
	std::vector<std::string> meshes_list{ "box111", "cone111", "cylinder111", "sphere111", "arrow111" };
	for (auto& mesh_name : meshes_list)
	{
		Model temp_model;
		temp_model.load_from_file("../../data/models/" + mesh_name + ".obj");
		meshes_storage.emplace_back(mesh_name, temp_model.get_meshes());
	}
}

My_qopengl_widget::My_qopengl_widget(Qt_gui& gui, SimBiCon_framework& conF, QWidget* parent) :
	QOpenGLWidget(parent)
{
	con_f = &conF;
	this->gui = &gui;

	fill_meshes_storage();
}

My_qopengl_widget::~My_qopengl_widget()
{
	del_desired_pose();
	del_skeletal();
	del_collision_primitive_box();
	clear_temp_object();
}

void My_qopengl_widget::initializeGL()
{
	initializeOpenGLFunctions();

	// Application-specific initialization
	m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader);
	m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader);
	m_program.link();
	m_program.bind();
	m_program.setUniformValue("lightSource.diffuse", QVector3D(0.1f, 0.1f, 0.1f));
	m_program.setUniformValue("lightSource.ambient", QVector3D(0.7f, 0.7f, 0.7f));

	glEnable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE0);
	m_program.setUniformValue("texUnit", 0);

	add_drawable_objects(con_f->get_controlled_character()->get_arbs());
	add_collision_primitive_box(con_f->get_controlled_character()->get_arbs());
	add_desired_pose(con_f->get_reference_handler()->get_arbs());
	add_desired_pose(con_f->get_desired_pose_handler()->get_arbs());
	add_ground(dynamic_cast<Ode_world*>(con_f->get_physical_world())->get_rb_by_name("ground"));
	init_some_sphere();
	init_some_arrow();
	glClearColor(.4f, .4f, .4f, 1.f);
}

void My_qopengl_widget::add_drawable_objects(std::vector<std::shared_ptr<Articulated_rigid_body>> articulatedRBs)
{
	for (auto& articulatedRB : articulatedRBs)
	{
		for (auto& model : articulatedRB->get_models())
		{
			m_skeletal.push_back(model.get());
			for (auto& mesh : model->meshes)
			{
				mesh.setup_mesh(*this);
			}
			articulatedRB->add_observer(model.get());
			model->linked_body = articulatedRB.get();
			model->set_position(articulatedRB->get_cm_position());
			model->set_rotation(articulatedRB->get_orientation());
		}
	}
}

void My_qopengl_widget::add_desired_pose(std::vector<std::shared_ptr<Articulated_rigid_body>> articulatedRBs)
{
	for (auto& articulatedRB : articulatedRBs)
	{
		auto models = articulatedRB->get_models();
		for (auto& model : models)
		{
			model->set_color({ 0.0f,0.8f,0.0f });
			m_desired_pose.push_back(model.get());
			for (auto& mesh : model->meshes)
			{
				mesh.setup_mesh(*this);
			}
			articulatedRB->add_observer(model.get());
			model->linked_body = articulatedRB.get();
			model->set_position(articulatedRB->get_cm_position());
			model->set_rotation(articulatedRB->get_orientation());
		}
	}
}

void My_qopengl_widget::del_desired_pose()
{
	for (auto& model_ptr : m_desired_pose)
	{
		if (model_ptr->linked_body)
		{
			model_ptr->linked_body->del_observer(model_ptr);
		}
	}
	m_desired_pose.clear();
}

void My_qopengl_widget::del_skeletal()
{
	for (auto& model_ptr : m_skeletal)
	{
		if (model_ptr->linked_body)
		{
			model_ptr->linked_body->del_observer(model_ptr);
		}
	}
	m_skeletal.clear();
}

void My_qopengl_widget::add_collision_primitive_box(std::vector<std::shared_ptr<Articulated_rigid_body>> articulatedRBs)
{
	for (auto& articulatedRB : articulatedRBs)
	{
		for (auto& cdp : articulatedRB->get_cdps())
		{
			m_CDPs.push_back(cdp->get_model().get());
			cdp->get_model()->set_color({ 0.0f,0.0f,0.0f });
			for (auto& mesh : cdp->get_model()->meshes)
			{
				mesh.setup_mesh(*this);
			}
			articulatedRB->add_observer(cdp->get_model().get());
			cdp->get_model()->linked_body = articulatedRB.get();
			cdp->get_model()->set_position(articulatedRB->get_cm_position());
			cdp->get_model()->set_rotation(articulatedRB->get_orientation());
		}
	}
}

void My_qopengl_widget::del_collision_primitive_box()
{
	for (auto& cdp_ptr : m_CDPs)
	{
		if (cdp_ptr->linked_body)
		{
			cdp_ptr->linked_body->del_observer(cdp_ptr);
		}
	}
	m_CDPs.clear();
}

void My_qopengl_widget::add_ground(const std::shared_ptr<Rigid_body>& ground)
{
	for (auto& cdp : ground->get_cdps())
	{
		m_ground_plane = cdp->get_model().get();
		m_ground_plane->set_color({ 0.7f,0.7f,0.7f });
		for (auto& mesh : m_ground_plane->meshes)
		{
			mesh.setup_mesh(*this);
		}
		cdp->get_model()->set_position(ground->get_cm_position());
		cdp->get_model()->set_rotation(ground->get_orientation());
	}
}

void My_qopengl_widget::init_some_sphere()
{
	for (auto i = 0; i < 100; i++)
	{
		auto temp_model = std::make_shared<Model>();
		temp_model->add_meshes(meshes_storage[3].second);
		temp_model->scale_meshes({ 1, 1, 1 });
		temp_model->set_position(Point3d());
		for (auto& mesh : temp_model->meshes)
		{
			mesh.setup_mesh(*this);
		}
		temp_model->set_color({ 0.0f, 0.0f, 0.0f });
		temp_model->displayed = false;
		m_temp_sphere.push_back(std::move(temp_model));
	}
}


void My_qopengl_widget::init_some_arrow()
{
	for (auto i = 0; i < 100; i++)
	{
		auto temp_model = std::make_shared<Model>();
		temp_model->add_meshes(meshes_storage[4].second);
		temp_model->scale_meshes({ 1, 1, 1 });
		temp_model->set_position(Point3d());
		for (auto& mesh : temp_model->meshes)
		{
			mesh.setup_mesh(*this);
		}
		temp_model->set_color({ 0.0f, 0.0f, 0.0f });
		temp_model->displayed = false;
		m_temp_arrows.push_back(std::move(temp_model));
	}
}

void My_qopengl_widget::add_sphere(Vector3d pos, std::array<float, 3> color, std::array<float, 3> scale)
{
	for (auto& model : m_temp_sphere)
	{
		if (model->displayed)
		{
			continue;
		}
		else
		{
			model->set_position(pos);
			model->set_color(color);
			model->set_scale(scale);
			model->displayed = true;
			return;
		}
	}
}

void My_qopengl_widget::add_arrow(Vector3d origin, Quaternion orientation, double length)
{
	for (auto model : m_temp_arrows)
	{
		if (model->displayed)
		{
			continue;
		}
		else
		{
			model->set_position(origin);
			model->set_color({ 0,0,0 });
			model->set_rotation(orientation);
			model->set_scale({ 0.01f,  float(length), 0.01f });
			model->displayed = true;
			return;
		}
	}

}


void My_qopengl_widget::clear_temp_object()
{
	for (auto& model : m_temp_sphere)
	{
		model->displayed = false;
	}
	for (auto& model : m_temp_arrows)
	{
		model->displayed = false;
	}
}


void My_qopengl_widget::resizeGL(int w, int h)
{
	m_projection.setToIdentity();
	m_projection.perspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
	update();
}

void My_qopengl_widget::paintGL()
{
	QPainter painter;
	painter.begin(this);

	painter.beginNativePainting();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);

	m_view.setToIdentity();
	m_view.lookAt(m_cam_eye, m_cam_center, m_cam_up);

	m_program.bind();
	m_program.setUniformValue("lightSource.position", m_cam_eye);

	for (auto& object : m_skeletal)
	{
		object->draw();
	}

	for (auto& object : m_desired_pose)
	{
		object->draw();
	}

	for (auto& object : m_temp_sphere)
	{
		if (object->displayed)
		{
			object->draw();
		}
	}

	for (auto& object : m_temp_arrows)
	{
		if (object->displayed)
		{
			object->draw();
		}
	}

	m_ground_plane->draw();

	for (auto& object : m_CDPs)
	{
		object->draw_wire();
	}

	m_program.release();

	glDisable(GL_DEPTH_TEST);

	painter.endNativePainting();

	if (const int elapsed = m_time.elapsed())
	{
		QString framesPerSecond, stepPerSecond;
		framesPerSecond.setNum(m_frames / (elapsed / 1000.0), 'f', 2);
		painter.setPen(Qt::black);
		painter.drawText(20, 40, framesPerSecond + " paintGL calls / s");
	}


	painter.end();

	if (!(m_frames % 100))
	{
		m_time.start();
		m_frames = 0;
	}
	++m_frames;
}

void My_qopengl_widget::keyPressEvent(QKeyEvent* event)
{
	const auto norm_right = QVector3D::crossProduct(m_cam_up, m_cam_eye - m_cam_center).normalized();
	auto cam_focus_vector = m_cam_eye - m_cam_center;
	//auto dist_up = camFocusVector.distanceToLine(QVector3D(0, 0, 0), m_camUp);
	auto rot_up = QQuaternion::fromAxisAndAngle(norm_right, -1);
	auto rot_down = QQuaternion::fromAxisAndAngle(norm_right, 1);
	auto rot_left = QQuaternion::fromAxisAndAngle(m_cam_up, -1);
	auto rot_right = QQuaternion::fromAxisAndAngle(m_cam_up, 1);

	switch (event->key())
	{
	case Qt::Key_Up:
		if (rot_up.rotatedVector(cam_focus_vector).distanceToLine(QVector3D(0, 0, 0), m_cam_up) > 0.1f)
			m_cam_eye = rot_up.rotatedVector(cam_focus_vector) + m_cam_center;
		break;
	case Qt::Key_Down:
		if (rot_down.rotatedVector(cam_focus_vector).distanceToLine(QVector3D(0, 0, 0), m_cam_up) > 0.1f)
			m_cam_eye = rot_down.rotatedVector(cam_focus_vector) + m_cam_center;
		break;
	case Qt::Key_Right:
		m_cam_eye = rot_right.rotatedVector(cam_focus_vector) + m_cam_center;
		break;
	case Qt::Key_Left:
		m_cam_eye = rot_left.rotatedVector(cam_focus_vector) + m_cam_center;
		break;
	case Qt::Key_PageUp:
		m_cam_eye = cam_focus_vector - cam_focus_vector.normalized() * 0.01f + m_cam_center;
		break;
	case Qt::Key_PageDown:
		m_cam_eye = cam_focus_vector + cam_focus_vector.normalized() * 0.01f + m_cam_center;
		break;
	case Qt::Key_D:
		key_d_pressed();
		break;
	default:
		break;
	}
	update();
}


void My_qopengl_widget::mousePressEvent(QMouseEvent* event)
{
	m_last_pos = event->pos();
}

void My_qopengl_widget::mouseMoveEvent(QMouseEvent* event)
{
	const auto dx = event->x() - m_last_pos.x();
	const auto dy = event->y() - m_last_pos.y();

	const auto norm_right = QVector3D::crossProduct(m_cam_up, m_cam_eye - m_cam_center).normalized();
	auto cam_focus_vector = m_cam_eye - m_cam_center;
	//auto dist_up = camFocusVector.distanceToLine(QVector3D(0, 0, 0), m_camUp);
	auto rot_ud = QQuaternion::fromAxisAndAngle(norm_right, dy);
	auto rot_lr = QQuaternion::fromAxisAndAngle(m_cam_up, dx);

	if (event->buttons() & Qt::LeftButton)
	{
		m_cam_eye = rot_lr.rotatedVector(cam_focus_vector) + m_cam_center;
		cam_focus_vector = m_cam_eye - m_cam_center;
		m_cam_eye = rot_ud.rotatedVector(cam_focus_vector) + m_cam_center;
	}
	else if (event->buttons() & Qt::RightButton)
	{
		m_cam_center += norm_right * dx * 0.01f;
		m_cam_eye += norm_right * dx * 0.01f;

		m_cam_center += m_cam_up * dy * -0.01f;
		m_cam_eye += m_cam_up * dy * -0.01f;
	}
	m_last_pos = event->pos();
	update();
}

void My_qopengl_widget::wheelEvent(QWheelEvent* event)
{
	auto camFocusVector = m_cam_eye - m_cam_center;
	QPoint numDegrees = event->angleDelta() / 8;
	m_cam_eye = camFocusVector - camFocusVector.normalized() * numDegrees.y() * 0.01f + m_cam_center;
}