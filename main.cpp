#include <chrono>
#include <thread>
#include <limits>

#include <fstream>
#include <iostream>

#include "Scene.h"

Scene* m_scene = nullptr;

void Initialize()
{
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

int main( int argc, char * argv[] )
{
	Initialize();

	int iterCurr = 0;
	const int iterMax = 4*1024;

	do
	{
		MainLoop();
		++iterCurr;
	} while (iterCurr < iterMax);

	std::ofstream myfile;
	myfile.open("Book00_SPEHERES.txt");

	typedef std::numeric_limits< float > flt_limits;
	myfile.precision(flt_limits::max_digits10);

	// Book00 scene has 36 dynamic speheres dropping on 9 static speheres
	for (const Body& currBody : m_scene->m_bodies)
	{
		Shape::shapeType_t myShape = currBody.m_shape->GetType();
		if (myShape == Shape::SHAPE_SPHERE)
		{
			const ShapeSphere* myShapeSphere = (const ShapeSphere*)currBody.m_shape;
			myfile << currBody.m_position.x() << ' ' << currBody.m_position.y() << ' ' << currBody.m_position.z() << ' ' << myShapeSphere->m_radius << '\n';
		}
	}

	myfile.close();

	delete m_scene;
	m_scene = nullptr;

	return 0;
}
