extern "C" {
#include "openGJK/openGJK.h"
}

#include "ccd/ccd.h"
#include "ccd/quat.h"

#include "Physics/Intersections.h"

int main( int argc, char * argv[] )
{
	// libccd
	{
		size_t i;
		ccd_t ccd;

		CCD_INIT(&ccd);
	}

	Vec3 userPdat[9];
	userPdat[0] = Vec3(0.0f, 5.5f, 0.0f);
	userPdat[1] = Vec3(2.3f, 1.0f, -2.0f);
	userPdat[2] = Vec3(8.1f, 4.0f, 2.4f);
	userPdat[3] = Vec3(4.3f, 5.0f, 2.2f);
	userPdat[4] = Vec3(2.5f, 1.0f, 2.3f);
	userPdat[5] = Vec3(7.1f, 1.0f, 2.4f);
	userPdat[6] = Vec3(1.0f, 1.5f, 0.3f);
	userPdat[7] = Vec3(3.3f, 0.5f, 0.3f);
	userPdat[8] = Vec3(6.0f, 1.4f, 0.2f);

	Vec3 userQdat[9];
	userQdat[0] = Vec3(-0.0f, -5.5f, 0.0f);
	userQdat[1] = Vec3(-2.3f, -1.0f, 2.0f);
	userQdat[2] = Vec3(-8.1f, -4.0f, -2.4f);
	userQdat[3] = Vec3(-4.3f, -5.0f, -2.2f);
	userQdat[4] = Vec3(-2.5f, -1.0f, -2.3f);
	userQdat[5] = Vec3(-7.1f, -1.0f, -2.4f);
	userQdat[6] = Vec3(-1.0f, -1.5f, -0.3f);
	userQdat[7] = Vec3(-3.3f, -0.5f, -0.3f);
	userQdat[8] = Vec3(-6.0f, -1.4f, -0.2f);

	// openGJK
	{
		// Squared distance computed by openGJK.
		double dd;
		// Structure of simplex used by openGJK.
		struct simplex  s;
		// Number of vertices defining body 1 and body 2, respectively.
		int    nvrtx1, nvrtx2;
		// Structures of body 1 and body 2, respectively.
		struct bd       bd1;
		struct bd       bd2;

		int npoints = 9;

		/* Coordinates of object 1. */
		{
			nvrtx1 = npoints;
			double** arr = (double**)malloc(npoints * sizeof(double*));
			for (int i = 0; i < npoints; i++)
				arr[i] = (double*)malloc(3 * sizeof(double));

			bd1.coord = arr;
			bd1.numpoints = npoints;

			for (int idx = 0; idx < npoints; idx++)
			{
				bd1.coord[idx][0] = userPdat[idx].x();
				bd1.coord[idx][1] = userPdat[idx].y();
				bd1.coord[idx][2] = userPdat[idx].z();
			}
		}

		/* Coordinates of object 2. */
		{
			nvrtx2 = npoints;
			double** arr = (double**)malloc(npoints * sizeof(double*));
			for (int i = 0; i < npoints; i++)
				arr[i] = (double*)malloc(3 * sizeof(double));

			bd2.coord = arr;
			bd2.numpoints = npoints;

			for (int idx = 0; idx < npoints; idx++)
			{
				bd2.coord[idx][0] = userQdat[idx].x();
				bd2.coord[idx][1] = userQdat[idx].y();
				bd2.coord[idx][2] = userQdat[idx].z();
			}
		}

		/* Initialise simplex as empty */
		s.nvrtx = 0;

		/* For importing openGJK this is Step 3: invoke the GJK procedure. */
		/* Compute squared distance using GJK algorithm. */
		dd = gjk(bd1, bd2, &s);

		/* Print distance between objects. */
		printf("Distance between bodies %f\n", dd);

		/* Free memory */
		for (int i = 0; i < bd1.numpoints; i++)
			free(bd1.coord[i]);
		free(bd1.coord);
		for (int i = 0; i < bd2.numpoints; i++)
			free(bd2.coord[i]);
		free(bd2.coord);
	}

	return 0;
}
