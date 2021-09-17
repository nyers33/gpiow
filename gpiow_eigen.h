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
typedef Eigen::Matrix<float, 4, 1> Vec4;

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

inline Quat QuatCreate(float x, float y, float z, float w)
{
	// eigen style quat init
	// Quaternion (const Scalar &w, const Scalar &x, const Scalar &y, const Scalar &z)
	return Quat(w, x, y, z);
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

inline bool Vec3IsValid(const Vec3& self)
{
	if (self.x() * 0.0f != self.x() * 0.0f) {
		return false;
	}

	if (self.y() * 0.0f != self.y() * 0.0f) {
		return false;
	}

	if (self.z() * 0.0f != self.z() * 0.0f) {
		return false;
	}

	return true;
}

inline void Vec3GetOrtho(const Vec3& self, Vec3& u, Vec3& v)
{
	Vec3 n = self;
	n.normalize();

	const Vec3 w = (n.z() * n.z() > 0.9f * 0.9f) ? Vec3(1, 0, 0) : Vec3(0, 0, 1);
	u = w.cross(n);
	u.normalize();

	v = n.cross(u);
	v.normalize();
	u = v.cross(n);
	u.normalize();
}

inline Mat3 Mat4Minor(const Mat4& mat, const int i, const int j)
{
	Mat3 minor;

	int yy = 0;
	for (int y = 0; y < 4; y++) {
		if (y == j) {
			continue;
		}

		int xx = 0;
		for (int x = 0; x < 4; x++) {
			if (x == i) {
				continue;
			}

			minor(xx,yy) = mat(x,y);
			xx++;
		}

		yy++;
	}

	return minor;
}

inline float Mat4Cofactor(const Mat4& mat, const int i, const int j)
{
	const Mat3 minor = Mat4Minor(mat, i, j);
	const float C = float(pow(-1, i + 1 + j + 1)) * minor.determinant();
	return C;
}

inline void Mat4Orient(Mat4& mat, Vec3 pos, Vec3 fwd, Vec3 up)
{
	Vec3 left = up.cross(fwd);

	// For our coordinate system where:
	// +x-axis = fwd
	// +y-axis = left
	// +z-axis = up
	mat.row(0) = Vec4(fwd.x(), left.x(), up.x(), pos.x());
	mat.row(1) = Vec4(fwd.y(), left.y(), up.y(), pos.y());
	mat.row(2) = Vec4(fwd.z(), left.z(), up.z(), pos.z());
	mat.row(3) = Vec4(0, 0, 0, 1);
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