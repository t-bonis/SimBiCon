#pragma once

#include "mathLib.h"
#include "TransformationMatrix.h"
#include "Vector3d.h"
#include <QtGui\QQuaternion>

/*================================================================================================================================================================*
 *	This class provides an implementation for quaternions. Internally, each quaternion it will be stored in the form: q = s + v, where s is a scalar and v is a   |
 *	vector.                                                                                                                                                       |
 *================================================================================================================================================================*/
class Quaternion
{
public:
	//the scalar part of the quaternion
	double s{ 1 };
	//the vector part of the quaternion
	Vector3d v{ 0, 0, 0 };
public:

	Quaternion(double s, const Vector3d& v)
	{
		this->s = s;
		this->v = v;
	}

	Quaternion(const Quaternion& other)
	{
		this->s = other.s;
		this->v = other.v;
	}

	Quaternion()
	{
		this->s = 1;
		this->v = Vector3d(0, 0, 0);
	}

	//	Another constructor.
	Quaternion(double w, double x, double y, double z)
	{
		this->s = w;
		this->v = Vector3d(x, y, z);
	}	


	//those function are just renaming of other functions so that it is easier to read the code...
	Vector3d to_world_coordinate(const Vector3d& vect) const { return rotate(vect); }
	Vector3d to_local_coordinate(const Vector3d& vect) const { return get_conjugate().rotate(vect); }

	
	//	This method is used to set the current quaternion to the product of the two quaternions passed in as parameters.
	//	the bool parameters invA and invB indicate wether or not, the quaternion a or b should be inverted (well, complex conjugate really)
	//	for the multiplication
	void set_to_product_of(const Quaternion& a, const Quaternion& b, bool invA = false, bool invB = false);

	//	This method is used to return the rotation angle represented by this quaternion - in the range -pi to pi.
	//	Because you can always consider a rotation to be of x degrees around axis v, or by -x degrees around axis -v,
	//	we need to know the base rotation axis.
	double get_rotation_angle(const Vector3d& positive_rot_axis) const
	{
		int sinSign = SGN(positive_rot_axis.dotProductWith(v));
		double result = 2 * safeACOS(s);
		if (sinSign < 0)
			result = -result;
		if (result > M_PI) result -= 2 * M_PI;
		if (result < -M_PI) result += 2 * M_PI;
		return result;
	}

	~Quaternion() = default;

	//	Returns the complex conjugate of the current quaternion.
	Quaternion get_conjugate() const;
	bool operator==(Quaternion q) const;
	bool operator!=(Quaternion q) const;

	//Returns the inverse of the current quaternion: q * q^-1 = identity quaternion: s = 1, v = (0,0,0)
	Quaternion get_inverse() const;

	//	Returns the length of a quaternion.
	double get_length() const;

	//	Computes the dot product between the current quaternion and the one given as parameter.
	double dot_product_with(const Quaternion& other) const;

	/**
		This method returns a quaternion that is the result of linearly interpolating between the current quaternion and the one provided as a parameter.
		The value of the parameter t indicates the progress: if t = 0, the result will be *this. If it is 1, it will be other. If it is inbetween, then
		the result will be a combination of the two initial quaternions.
		Both quaternions that are used for the interpolation are assumed to have unit length!!!
	*/
	Quaternion linearly_interpolate_with(const Quaternion& other, double t) const;

	/**
		This method returns a quaternion that is the result of spherically interpolating between the current quaternion and the one provided as a parameter.
		The value of the parameter t indicates the progress: if t = 0, the result will be *this. If it is 1, it will be other. If it is inbetween, then
		the result will be a combination of the two initial quaternions.
		Both quaternions that are used for the interpolation are assumed to have unit length!!!
	*/
	Quaternion spherically_interpolate_with(const Quaternion& other, double t) const;

	//	This method will return a quaternion that represents a rotation of angle radians around the axis provided as a parameter.
	//	IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!! and angle in RADIAN

	static Quaternion get_rotation_quaternion(double angle,const Vector3d& axis);

	//	This method is used to rotate the vector that is passed in as a parameter by the current quaternion (which is assumed to be a
	//	unit quaternion).
	Vector3d rotate(const Vector3d& u) const;

	QQuaternion to_QQuaternion()
	{
		return QQuaternion(s, v.x, v.y, v.z);
	}

	//	This method is used to rotate the vector that is passed in as a parameter by the current quaternion (which is assumed to be a
	//	unit quaternion).
	Vector3d inverse_rotate(const Vector3d& u) const;


	//	This method populates the transformation matrix that is passed in as a parameter with the 
	//	4x4 matrix that represents an equivalent rotation as the current quaternion.
	void get_rotation_matrix(TransformationMatrix* m) const;


	//	This method fills the presumably 3x3 matrix so that it represents an equivalent rotation as the given quaternion.
	void get_rotation_matrix(Matrix* m) const;


	// This method return the axis of rotation and the angle (in radians)
	void get_axis_and_angle(Vector3d& axis, double& angle);

	// This method return the euler angle (in radians) with this assumption
	// Heading (y) applied first ; Attitude (z) applied second; Bank(x) applied last
	void get_euler_angles(Vector3d& angles);

	/**
		Returns the result of multiplying the current quaternion by rhs. NOTE: the product of two quaternions represents a rotation as well: q1*q2 represents
		a rotation by q2 followed by a rotation by q1!!!!
	*/
	Quaternion operator *(const Quaternion& other) const;

	/**
		This operator multiplies the current quaternion by the rhs one. Keep in mind the note RE quaternion multiplication.
	*/
	Quaternion& operator *=(const Quaternion& other);

	/**
		This method multiplies the current quaternion by a scalar.
	*/
	Quaternion& operator *=(double scalar);

	/**
		This method returns a copy of the current quaternion multiplied by a scalar.
	*/
	Quaternion operator *(double scalar) const;

	/**
		This method returns a quaternion that was the result of adding the quaternion rhs to the current quaternion.
	*/
	Quaternion operator +(const Quaternion& other) const;

	/**
		This method adds the rhs quaternion to the current one.
	*/
	Quaternion& operator +=(const Quaternion& other);

	//	This method transforms the current quaternion to a unit quaternion.
	Quaternion& to_unit();

	/**
		This method returns the scalar part of the current quaternion
	*/
	double getS() const { return this->s; }

	/**
		This method returns the vector part of the current quaternion
	*/
	Vector3d getV() const { return this->v; }

	/**
		Rotates the vector v by the quaternion and the result is placed in 'result'
	*/
	void fastRotate(const Vector3d& u, Vector3d* result) const
	{
		//uRot = q * (0, u) * q' = (s, v) * (0, u) * (s, -v)
		//working it out manually, we get:

		//		Vector3d t = u * s + v.crossProductWith(u);
		//		*result = v*u.dotProductWith(v) + t * s + v.crossProductWith(t);

		result->set_to_cross_product(v, u);
		result->add_scaled_vector(u, s);
		Vector3d tmp;
		tmp.set_to_cross_product(v, *result);
		result->multiply_by(s);
		result->add_vector(tmp);
		result->add_scaled_vector(v, u.dotProductWith(v));
	}

	/**
		sets the current quaternion to represent a rotation of theta radians about the (assumed) unit axis
	*/
	void set_to_rotation_quaternion(double angle, const Vector3d& axis)
	{
		s = cos(angle / 2);
		v.setValues(axis.x, axis.y, axis.z);
		v.multiply_by(sin(angle / 2));

		this->to_unit();
	}

	/**
		Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
		This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
		parameter, and another more arbitrary rotation.

		AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
		that is returned by this function.

		In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
		from T.

		It is assumed that vB is a unit vector!! rotAngle is the rotation angle around rotAxis (this gives the
		AqT transformation. To go from frame B to A, we then twist around the axis v by the amount returned
		by this function, and we then rotate around rotAxis by rotAngle.
	*/

	double decomposeRotation(const Vector3d vB, double *rotAngle, Vector3d* rotAxis) const;

	/**
		Assume that the current quaternion represents the relative orientation between two coordinate frames P and C (i.e. q
		rotates vectors from the child/local frame C into the parent/global frame P).

		With v specified in frame C's coordinates, this method decomposes the current relative rotation, such that:

		PqC = qA * qB, where qB represents a rotation about axis v.

		This can be thought of us as a twist about axis v - qB - and a more general rotation, and swing
		- qA - decomposition. Note that qB can be thought of as a rotation from the C frame into a tmp trame T,
		and qA a rotation from T into P.

		In the T coordinate frame, v is the same as in C, and qA is a rotation that aligns v from P to that
		from T.
	*/
	void decomposeRotation(Quaternion* qA, Quaternion* qB, const Vector3d& vC) const;

	/**
		Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
		This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
		parameter, and another more arbitrary rotation.

		AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
		that is returned by this function.

		In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
		from T.

		It is assumed that vB is a unit vector!! This method returns TqB, which represents a twist about
		the axis vB.
	*/
	Quaternion decomposeRotation(const Vector3d& vB) const;
};


/**
	This method is used to return a number that measures the distance between two rotation quaternion. The number returned
	is equal to |theta|, where theta is the min angle of rotations that can align the two coordinate frames
	represented by the two orientations.
*/
inline double distanceBetweenOrientations(const Quaternion& a, const Quaternion& b)
{
	double temp = (a.get_conjugate() * b).v.length();
	if (temp > 1)
		temp = 1;
	return 2 * asin(temp);
}
