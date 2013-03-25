/*
 * @summary: recast math
 * @date: 2013-03-20
 * @author: zl
 */

#ifndef DTMATH_H_
#define DTMATH_H_


#include "config.h"
#include <vector>


struct dtVec3
{
	/// Default constructor.
	dtVec3() : x(0.0f), y(0.0f), z(0.0f) {}

	/// Construct using coordinates.
	dtVec3(const float* v_) : x(v_[0]), y(v_[1]), z(v_[2]) {}

	/// Construct using coordinates.
	dtVec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

	/// Set this vector to all zeros.
	void setZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

	/// Set this vector to some specified coordinates.
	void set(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }

	/// Negate this vector.
	dtVec3 operator -() const { dtVec3 v; v.set(-x, -y, -z); return v; }

	/// Add a vector to this vector.
	void operator += (const dtVec3& v_)
	{
		x += v_.x; y += v_.y; z += v_.z;
	}

	/// Subtract a vector from this vector.
	void operator -= (const dtVec3& v_)
	{
		x -= v_.x; y -= v_.y; z -= v_.z;
	}

	/// Multiply this vector by a scalar.
	void operator *= (float s)
	{
		x *= s; y *= s; z *= s;
	}

	float x, y, z;
};


typedef std::vector<dtVec3> dtVec3List;


inline dtVec3 operator * (float s, const dtVec3& a)
{
	return dtVec3(s * a.x, s * a.y, s * a.z);
}

inline dtVec3 operator + (const dtVec3& a, const dtVec3& b)
{
	return dtVec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline dtVec3 operator - (const dtVec3& a, const dtVec3& b)
{
	return dtVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline dtVec3 operator * (const dtVec3& a, float s)
{
	return dtVec3(a.x * s, a.y * s, a.z * s);
}

inline dtVec3 operator / (const dtVec3& a, float s)
{
	return dtVec3(a.x / s, a.y / s, a.z / s);
}

inline dtVec3 operator / (float s, const dtVec3& a)
{
	return dtVec3(a.x / s, a.y / s, a.z / s);
}

inline dtVec3 operator /= (const dtVec3& a, float s)
{
	return dtVec3(a.x / s, a.y / s, a.z / s);
}

inline dtVec3 operator /= (float s, const dtVec3& a)
{
	return dtVec3(a.x / s, a.y / s, a.z / s);
}

inline bool operator == (const dtVec3& a, const dtVec3& b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

/// Perform the dot product on two vectors.
inline float dtDot(const dtVec3& a, const dtVec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
inline dtVec3 dtCross(const dtVec3& a, const dtVec3& b)
{
	return dtVec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

float frand_01();


#endif /* DTMATH_H_ */
