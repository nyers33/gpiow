#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/Dense>
#include <Eigen/Sparse>

typedef Eigen::Matrix<float, 2, 2> Mat2;
typedef Eigen::Matrix<float, 3, 3> Mat3;
typedef Eigen::Matrix<float, 4, 4> Mat4;
typedef Eigen::Matrix<float, 2, 1> Vec2;
typedef Eigen::Matrix<float, 3, 1> Vec3;

typedef Eigen::AlignedBox<float, 3> Bounds;
typedef Eigen::Quaternionf Quat;

inline Quat QuatCreate(Vec3 n, const float angleRadians)
{
	Quat q;

	const float halfAngleRadians = 0.5f * angleRadians;

	q.w() = cosf(halfAngleRadians);

	const float halfSine = sinf(halfAngleRadians);
	n.normalize();
	q.x() = n.x() * halfSine;
	q.y() = n.y() * halfSine;
	q.z() = n.z() * halfSine;

	return q;
}

inline Vec3 QuatRotatePoint(const Quat& q, const Vec3& rhs)
{
	Quat vector(0.0f, rhs.x(), rhs.y(), rhs.z());
	Quat rotP = q * vector * q.inverse();
	return rotP.vec();
}

inline Mat3 QuatToMat3(const Quat& q)
{
	Mat3 mat = Mat3::Identity();
	
	mat.row(0) = QuatRotatePoint(q, mat.row(0));
	mat.row(1) = QuatRotatePoint(q, mat.row(1));
	mat.row(2) = QuatRotatePoint(q, mat.row(2));

	return mat;
}

// to interface gpiow math structs
#define Cross cross
#define Dot dot
#define Normalize normalize
#define Transpose transpose
#define Inverse inverse
#define GetMagnitude norm
#define GetLengthSqr squaredNorm
#define Zero setZero
#define Identity setIdentity
#define Expand extend
#define mins min()
#define maxs max()