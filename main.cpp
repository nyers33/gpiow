#include <chrono>
#include <thread>
#include <limits>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

#include "Scene.h"

Scene* m_scene = nullptr;

// helper for file naming increment padding for leading 0ss
std::string formatInt(int i, int width, char pad)
{
	std::stringstream ss;
	ss << std::setfill(pad) << std::setw(width) << std::fixed << i;

	return ss.str();
}

void Initialize()
{
	// only req for book01
	FillDiamond();

	m_scene = new Scene;
	m_scene->Initialize();
	m_scene->Reset();
}

void MainLoop()
{
	static int numSamples = 0;
	static float avgTime_us = 0.0f;
	static float maxTime_us = 0.0f;

	{
		bool runPhysics = true;
		float dt_us = 16000;
		float dt_sec = dt_us * 0.001f * 0.001f;

		printf("\ndt_ms: %.1f    ", dt_us * 0.001f);

		// Run Update
		if (runPhysics)
		{
			auto start = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < 2; i++)
			{
				m_scene->Update(dt_sec * 0.5f);
			}
			auto end = std::chrono::high_resolution_clock::now();

			std::chrono::duration<float> diff = end - start;

			// https://www.modernescpp.com/index.php/time-duration
			typedef std::chrono::duration<float, std::ratio<1, 1000000>> MyMicroSecondTick;
			dt_us = MyMicroSecondTick(diff).count();

			if (dt_us > maxTime_us)
			{
				maxTime_us = dt_us;
			}

			avgTime_us = (avgTime_us * float(numSamples) + dt_us) / float(numSamples + 1);
			numSamples++;

			printf("frame dt_ms: %.2f %.2f %.2f", avgTime_us * 0.001f, maxTime_us * 0.001f, dt_us * 0.001f);
		}
	}
}

void MyFillCubeTessellated(int numDivisions, std::vector< Vec3 >& m_vertices, std::vector< unsigned int >& m_indices) {
	if (numDivisions < 1) {
		numDivisions = 1;
	}

	const int numnum = (numDivisions + 1) * (numDivisions + 1);
	Vec3* v = (Vec3*)malloc(numnum * sizeof(Vec3));
	Vec2* st = (Vec2*)malloc(numnum * sizeof(Vec2));
	for (int y = 0; y < numDivisions + 1; y++) {
		for (int x = 0; x < numDivisions + 1; x++) {
			float xf = (((float)x / (float)numDivisions) - 0.5f) * 2.0f;
			float yf = (((float)y / (float)numDivisions) - 0.5f) * 2.0f;
			v[y * (numDivisions + 1) + x] = Vec3(xf, yf, 1.0f);

			float sf = (float)x / (float)numDivisions;
			float tf = (float)y / (float)numDivisions;
			st[y * (numDivisions + 1) + x] = Vec2(sf, tf);
		}
	}

	const int numFaces = numDivisions * numDivisions;

	int faceIdx = 0;
	int* faceIdxs = (int*)malloc(3 * 2 * numFaces * sizeof(int));
	for (int y = 0; y < numDivisions; y++) {
		for (int x = 0; x < numDivisions; x++) {
			int y0 = y;
			int y1 = y + 1;
			int x0 = x;
			int x1 = x + 1;

			faceIdxs[faceIdx * 6 + 1] = y0 * (numDivisions + 1) + x0;
			faceIdxs[faceIdx * 6 + 0] = y1 * (numDivisions + 1) + x0;
			faceIdxs[faceIdx * 6 + 2] = y1 * (numDivisions + 1) + x1;

			faceIdxs[faceIdx * 6 + 4] = y0 * (numDivisions + 1) + x0;
			faceIdxs[faceIdx * 6 + 3] = y1 * (numDivisions + 1) + x1;
			faceIdxs[faceIdx * 6 + 5] = y0 * (numDivisions + 1) + x1;

			faceIdx++;
		}
	}

	Mat3 matOrients[6];
	for (int i = 0; i < 6; i++) {
		matOrients[i].Identity();
	}
	// px
	matOrients[0].row(0) = Vec3(0.0f, 0.0f, 1.0f);
	matOrients[0].row(1) = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[0].row(2) = Vec3(0.0f, 1.0f, 0.0f);
	// nx
	matOrients[1].row(0) = Vec3(0.0f, 0.0f, -1.0f);
	matOrients[1].row(1) = Vec3(-1.0f, 0.0f, 0.0f);
	matOrients[1].row(2) = Vec3(0.0f, 1.0f, 0.0f);

	// py
	matOrients[2].row(0) = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[2].row(1) = Vec3(0.0f, 0.0f, 1.0f);
	matOrients[2].row(2) = Vec3(0.0f, -1.0f, 0.0f);
	// ny
	matOrients[3].row(0) = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[3].row(1) = Vec3(0.0f, 0.0f, -1.0f);
	matOrients[3].row(2) = Vec3(0.0f, 1.0f, 0.0f);

	// pz
	matOrients[4].row(0) = Vec3(1.0f, 0.0f, 0.0f);
	matOrients[4].row(1) = Vec3(0.0f, 1.0f, 0.0f);
	matOrients[4].row(2) = Vec3(0.0f, 0.0f, 1.0f);
	// nz
	matOrients[5].row(0) = Vec3(-1.0f, 0.0f, 0.0f);
	matOrients[5].row(1) = Vec3(0.0f, 1.0f, 0.0f);
	matOrients[5].row(2) = Vec3(0.0f, 0.0f, -1.0f);

	const int numIdxs = 3 * 2 * 6 * numFaces;
	const int numVerts = 4 * 6 * numFaces;
	Vec3* cubeVerts = (Vec3*)malloc(numVerts * sizeof(Vec3));
	int* cubeIdxs = (int*)malloc(numIdxs * sizeof(int));

	memset(cubeVerts, 0, sizeof(Vec3) * numVerts);

	for (int side = 0; side < 6; side++) {
		const Mat3& mat = matOrients[side];

		const Vec3 tang = mat * Vec3(1.0f, 0.0f, 0.0f);
		const Vec3 norm = mat * Vec3(0.0f, 0.0f, 1.0f);

		for (int vid = 0; vid < numnum; vid++) {
			const Vec3 xyz = mat * v[vid];
			const Vec2 uv = st[vid];

			cubeVerts[side * numnum + vid] = xyz;

			//cubeVerts[side * numnum + vid].xyz[0] = xyz[0];
			//cubeVerts[side * numnum + vid].xyz[1] = xyz[1];
			//cubeVerts[side * numnum + vid].xyz[2] = xyz[2];
			//
			//cubeVerts[side * numnum + vid].st[0] = uv[0];
			//cubeVerts[side * numnum + vid].st[1] = uv[1];
			//
			//cubeVerts[side * numnum + vid].norm[0] = FloatToByte_n11(norm[0]);
			//cubeVerts[side * numnum + vid].norm[1] = FloatToByte_n11(norm[1]);
			//cubeVerts[side * numnum + vid].norm[2] = FloatToByte_n11(norm[2]);
			//cubeVerts[side * numnum + vid].norm[3] = FloatToByte_n11(0.0f);
			//
			//cubeVerts[side * numnum + vid].tang[0] = FloatToByte_n11(tang[0]);
			//cubeVerts[side * numnum + vid].tang[1] = FloatToByte_n11(tang[1]);
			//cubeVerts[side * numnum + vid].tang[2] = FloatToByte_n11(tang[2]);
			//cubeVerts[side * numnum + vid].tang[3] = FloatToByte_n11(0.0f);
		}

		for (int idx = 0; idx < 3 * 2 * numFaces; idx++) {
			const int offset = 3 * 2 * numFaces * side;
			cubeIdxs[idx + offset] = faceIdxs[idx] + numnum * side;
		}
	}

	for (int i = 0; i < numVerts; i++) {
		m_vertices.push_back(cubeVerts[i]);
	}

	for (int i = 0; i < numIdxs; i++) {
		m_indices.push_back(cubeIdxs[i]);
	}

	free(v);
	free(st);
	free(faceIdxs);
	free(cubeVerts);
	free(cubeIdxs);
}

void MyFillSphere(const float radius, std::vector< Vec3 >& m_vertices, std::vector< unsigned int >& m_indices) {
	float t = radius;
	if (t < 0.0f) {
		t = 0.0f;
	}
	if (t > 100.0f) {
		t = 100.0f;
	}
	t /= 100.0f;
	float min = 5;
	float max = 30;
	float s = min * (1.0f - t) + max * t;
	MyFillCubeTessellated((int)s, m_vertices, m_indices);

	// Project the tessellated cube onto a sphere
	for (int i = 0; i < m_vertices.size(); i++) {
		Vec3 xyz = m_vertices[i];
		xyz.Normalize();

		//model.m_vertices[i].xyz[0] = xyz[0];
		//model.m_vertices[i].xyz[1] = xyz[1];
		//model.m_vertices[i].xyz[2] = xyz[2];

		m_vertices[i] = xyz;

		//model.m_vertices[i].norm[0] = FloatToByte_n11(xyz[0]);
		//model.m_vertices[i].norm[1] = FloatToByte_n11(xyz[1]);
		//model.m_vertices[i].norm[2] = FloatToByte_n11(xyz[2]);
		//model.m_vertices[i].norm[3] = FloatToByte_n11(0.0f);
		//
		//Vec3 tang = Byte4ToVec3(model.m_vertices[i].norm);
		//Vec3 bitang = xyz.Cross(tang);
		//bitang.Normalize();
		//tang = bitang.Cross(xyz);
		//
		//model.m_vertices[i].tang[0] = FloatToByte_n11(tang[0]);
		//model.m_vertices[i].tang[1] = FloatToByte_n11(tang[1]);
		//model.m_vertices[i].tang[2] = FloatToByte_n11(tang[2]);
		//model.m_vertices[i].tang[3] = FloatToByte_n11(0.0f);
	}
}

bool MyBuildFromShape(Shape* shape, std::vector< Vec3 >& m_vertices, std::vector< unsigned int >& m_indices) {
	if (NULL == shape) {
		return false;
	}

	if (shape->GetType() == Shape::SHAPE_BOX) {
		const ShapeBox* shapeBox = (const ShapeBox*)shape;

		m_vertices.clear();
		m_indices.clear();

		MyFillCubeTessellated(0, m_vertices, m_indices);
		Vec3 halfdim = (shapeBox->m_bounds.maxs - shapeBox->m_bounds.mins) * 0.5f;
		Vec3 center = (shapeBox->m_bounds.maxs + shapeBox->m_bounds.mins) * 0.5f;
		for (int v = 0; v < m_vertices.size(); v++) {
			m_vertices[v].x() *= halfdim.x();
			m_vertices[v].y() *= halfdim.y();
			m_vertices[v].z() *= halfdim.z();
			m_vertices[v] += center;
		}
	}
	else if (shape->GetType() == Shape::SHAPE_SPHERE) {
		const ShapeSphere* shapeSphere = (const ShapeSphere*)shape;

		m_vertices.clear();
		m_indices.clear();

		MyFillSphere(shapeSphere->m_radius, m_vertices, m_indices);
		for (int v = 0; v < m_vertices.size(); v++) {
			m_vertices[v] *= shapeSphere->m_radius;
		}
	}
	else if (shape->GetType() == Shape::SHAPE_CONVEX) {
		const ShapeConvex* shapeConvex = (const ShapeConvex*)shape;

		m_vertices.clear();
		m_indices.clear();

		// Build the connected convex hull from the points
		std::vector< Vec3 > hullPts;
		std::vector< tri_t > hullTris;
		BuildConvexHull(shapeConvex->m_points, hullPts, hullTris);

		// Calculate smoothed normals
		std::vector< Vec3 > normals;
		normals.reserve(hullPts.size());
		for (int i = 0; i < hullPts.size(); i++) {
			Vec3 norm(0.0f, 0.0f, 0.0f);

			for (int t = 0; t < hullTris.size(); t++) {
				const tri_t& tri = hullTris[t];
				if (i != tri.a && i != tri.b && i != tri.c) {
					continue;
				}

				const Vec3& a = hullPts[tri.a];
				const Vec3& b = hullPts[tri.b];
				const Vec3& c = hullPts[tri.c];

				Vec3 ab = b - a;
				Vec3 ac = c - a;
				norm += ab.Cross(ac);
			}

			norm.Normalize();
			normals.push_back(norm);
		}

		m_vertices.reserve(hullPts.size());
		for (int i = 0; i < hullPts.size(); i++) {
			Vec3 vert;

			//vert.xyz[0] = hullPts[i].x;
			//vert.xyz[1] = hullPts[i].y;
			//vert.xyz[2] = hullPts[i].z;

			vert = hullPts[i];

			//Vec3 norm = normals[i];
			//norm.Normalize();
			//
			//vert.norm[0] = FloatToByte_n11(norm[0]);
			//vert.norm[1] = FloatToByte_n11(norm[1]);
			//vert.norm[2] = FloatToByte_n11(norm[2]);
			//vert.norm[3] = FloatToByte_n11(0.0f);

			m_vertices.push_back(vert);
		}

		m_indices.reserve(hullTris.size() * 3);
		for (int i = 0; i < hullTris.size(); i++) {
			m_indices.push_back(hullTris[i].a);
			m_indices.push_back(hullTris[i].b);
			m_indices.push_back(hullTris[i].c);
		}
	}

	return true;
}

void SaveSceneMeshData(Scene* scene, std::vector< Vec4 >& all_points, std::vector< unsigned int >& all_indices)
{
	all_points.clear();
	all_indices.clear();

	for (const Body& currBody : scene->m_bodies)
	{
		std::vector< Vec3 > m_vertices;
		std::vector< unsigned int > m_indices;

		Shape::shapeType_t myShape = currBody.m_shape->GetType();
		MyBuildFromShape(currBody.m_shape, m_vertices, m_indices);

		Vec3 fwd = QuatRotatePoint(currBody.m_orientation, Vec3(1, 0, 0));
		Vec3 up = QuatRotatePoint(currBody.m_orientation, Vec3(0, 0, 1));

		Mat4 matOrient;
		Mat4Orient(matOrient, currBody.m_position, fwd, up);

		size_t indices_offset = all_points.size();
		for (int v = 0; v < m_vertices.size(); v++) {
			all_points.emplace_back(matOrient * Vec4(m_vertices[v].x(), m_vertices[v].y(), m_vertices[v].z(), 1.0f));
		}

		for (int idx = 0; idx < m_indices.size(); idx++) {
			all_indices.emplace_back(m_indices[idx] + indices_offset);
		}
	}
}

void SavePLY(const std::string& fileName, const std::vector< Vec4 >& all_points, const std::vector< unsigned int >& all_indices)
{
	std::ofstream myfile;
	myfile.open(fileName);

	typedef std::numeric_limits< float > flt_limits;
	myfile.precision(flt_limits::max_digits10);

	myfile << "ply" << '\n';
	myfile << "format ascii 1.0" << '\n';
	myfile << "element vertex " << all_points.size() << '\n';
	myfile << "property float x" << '\n';
	myfile << "property float y" << '\n';
	myfile << "property float z" << '\n';
	myfile << "property uint8 red" << '\n';
	myfile << "property uint8 green" << '\n';
	myfile << "property uint8 blue" << '\n';
	myfile << "element face " << all_indices.size() / 3 << '\n';
	myfile << "property list uchar uint vertex_indices" << '\n';
	myfile << "end_header" << '\n';

	for (const Vec4& currPoint : all_points)
	{
		myfile << currPoint.x() << ' ' << currPoint.y() << ' ' << currPoint.z() << ' ' << 127 << ' ' << 127 << ' ' << 127 << '\n';
	}

	for (int idx = 0; idx < all_indices.size() / 3; idx++) {
		myfile << "3 " << all_indices[3 * idx + 0] << ' ' << all_indices[3 * idx + 1] << ' ' << all_indices[3 * idx + 2] << '\n';
	}

	myfile.close();

}

int main( int argc, char * argv[] )
{
	int iterCurr = 0;
	const int iterMax = 2*1024;

	std::vector< Vec4 > all_points;
	std::vector< unsigned int > all_indices;

	// Book00 scene has 36 dynamic speheres dropping on 9 static speheres
	// Book01 scene has one convex object + one spehere + an arena made from 5 boxes

	Initialize();

	// gen mesh data and write to ply file for debug viz
	{
		SaveSceneMeshData(m_scene, all_points, all_indices);

		std::string iterStr = formatInt(iterCurr, 6, '0');
		std::string fileName = std::string("gpiow_") + m_scene->GetName() + "_" + iterStr + std::string(".ply");
		SavePLY(fileName, all_points, all_indices);
	}

	do
	{
		MainLoop();
		++iterCurr;
	} while (iterCurr < iterMax);
	
	// gen mesh data and write to ply file for debug viz
	{
		SaveSceneMeshData(m_scene, all_points, all_indices);

		std::string iterStr = formatInt(iterCurr, 6, '0');
		std::string fileName = std::string("gpiow_") + m_scene->GetName() + "_" + iterStr + std::string(".ply");
		SavePLY(fileName, all_points, all_indices);
	}

	delete m_scene;
	m_scene = nullptr;

	return 0;
}
