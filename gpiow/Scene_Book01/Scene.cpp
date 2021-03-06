//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"
#include <cmath>
#include <random>

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
AddStandardSandBox
====================================================
*/
void AddStandardSandBox( std::vector< Body > & bodies ) {
	Body body;

	body.m_position = Vec3( 0, 0, 0 );
	body.m_orientation = QuatCreate( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeBox( g_boxGround, sizeof( g_boxGround ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3( 50, 0, 0 );
	body.m_orientation = QuatCreate( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall0, sizeof( g_boxWall0 ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3(-50, 0, 0 );
	body.m_orientation = QuatCreate( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall0, sizeof( g_boxWall0 ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3( 0, 25, 0 );
	body.m_orientation = QuatCreate( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall1, sizeof( g_boxWall1 ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3( 0,-25, 0 );
	body.m_orientation = QuatCreate( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall1, sizeof( g_boxWall1 ) / sizeof( Vec3 ) );
	bodies.push_back( body );
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;

	if (0)
	{
		body.m_position = Vec3(10, 0, 3);
		body.m_orientation = QuatCreate(0, 0, 0, 1);
		body.m_linearVelocity = Vec3(-100, 0, 0);
		body.m_angularVelocity = Vec3(0.0f, 0.0f, 0.0f);
		body.m_invMass = 1.0f;
		body.m_elasticity = 0.5f;
		body.m_friction = 0.5f;
		body.m_shape = new ShapeSphere(0.5f);
		m_bodies.push_back(body);

		body.m_position = Vec3(-10, 0, 3);
		body.m_orientation = QuatCreate(0, 0, 0, 1);
		body.m_linearVelocity = Vec3(100, 0, 0);
		body.m_angularVelocity = Vec3(0, 10, 0);
		body.m_invMass = 1.0f;
		body.m_elasticity = 0.5f;
		body.m_friction = 0.5f;
		body.m_shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
		m_bodies.push_back(body);
	}
	else
	{
		body.m_linearVelocity = Vec3(0, 0, 0);
		body.m_angularVelocity = Vec3(0, 0, 0);
		body.m_invMass = 1.0f;
		body.m_elasticity = 0.5f;
		body.m_friction = 0.5f;

		std::random_device rnd;
		std::default_random_engine rnd_engine(rnd());
		std::uniform_real_distribution<float> distr(0., 1.);

		int nRows = 6;
		int nOuterRad = 8;
		int nInnerRad = 6;
		float hRow = 2.0f;

		for (int j = 0; j < nRows; ++j)
		{
			for (int i = 0; i < 8; ++i)
			{
				float coneAngle = 0.25f * (float)(M_PI);
				float z = distr(rnd_engine) * (1.0f - cos(coneAngle)) + cos(coneAngle);
				float phi = distr(rnd_engine) * 2.0f * (float)(M_PI);
				float x = sqrt(1.0f - z * z) * cos(phi);
				float y = sqrt(1.0f - z * z) * sin(phi);
				Vec3 randomConeDir(x, y, z);

				body.m_position = Vec3(
					4.0f * cosf((float)(M_PI) * i / ((float)(nOuterRad) / 2.0f) + 0.5f * (j % 2) * (float)(M_PI) / ((float)(nOuterRad) / 2.0f)),
					4.0f * sinf((float)(M_PI) * i / ((float)(nOuterRad) / 2.0f) + 0.5f * (j % 2) * (float)(M_PI) / ((float)(nOuterRad) / 2.0f)),
					6.0f + j * hRow
				);

				body.m_orientation = Quat(Eigen::AngleAxisf((0.5f * distr(rnd_engine) - 0.25f) * (float)(M_PI), randomConeDir));

				body.m_shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
				m_bodies.push_back(body);
			}
		}

		for (int j = 0; j < nRows; ++j)
		{
			for (int i = 0; i < 6; ++i)
			{
				float coneAngle = 0.25f * (float)(M_PI);
				float z = distr(rnd_engine) * (1.0f - cos(coneAngle)) + cos(coneAngle);
				float phi = distr(rnd_engine) * 2.0f * (float)(M_PI);
				float x = sqrt(1.0f - z * z) * cos(phi);
				float y = sqrt(1.0f - z * z) * sin(phi);
				Vec3 randomConeDir(x,y,z);

				body.m_position = Vec3(
					2.0f * cosf((float)(M_PI) * i / ((float)(nInnerRad) / 2.0f) + 0.5f * (j % 2) * (float)(M_PI) / ((float)(nInnerRad) / 2.0f)),
					2.0f * sinf((float)(M_PI) * i / ((float)(nInnerRad) / 2.0f) + 0.5f * (j % 2) * (float)(M_PI) / ((float)(nInnerRad) / 2.0f)),
					6.0f + j * hRow
				);

				body.m_orientation = Quat(Eigen::AngleAxisf((0.5f * distr(rnd_engine) - 0.25f) * (float)(M_PI), randomConeDir));

				body.m_shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
				m_bodies.push_back(body);
			}
		}
	}
	AddStandardSandBox(m_bodies);
}

/*
====================================================
CompareContacts
====================================================
*/
int CompareContacts( const void * p1, const void * p2 ) {
	contact_t a = *(contact_t*)p1;
	contact_t b = *(contact_t*)p2;

	if ( a.timeOfImpact < b.timeOfImpact ) {
		return -1;
	}

	if ( a.timeOfImpact == b.timeOfImpact ) {
		return 0;
	}

	return 1;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	// Gravity impulse
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		Body * body = &m_bodies[ i ];
		float mass = 1.0f / body->m_invMass;
		Vec3 impulseGravity = Vec3( 0, 0, -10 ) * mass * dt_sec;
		body->ApplyImpulseLinear( impulseGravity );
	}

	//
	// Broadphase
	//
	std::vector< collisionPair_t > collisionPairs;
	BroadPhase( m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec );

	//
	//	NarrowPhase (perform actual collision detection)
	//
	int numContacts = 0;
	const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t * contacts = (contact_t *)alloca( sizeof( contact_t ) * maxContacts );
	for ( int i = 0; i < collisionPairs.size(); i++ ) {
		const collisionPair_t & pair = collisionPairs[ i ];
		Body * bodyA = &m_bodies[ pair.a ];
		Body * bodyB = &m_bodies[ pair.b ];

		// Skip body pairs with infinite mass
		if ( 0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass ) {
			continue;
		}

		contact_t contact;
		if ( Intersect( bodyA, bodyB, dt_sec, contact ) ) {
			contacts[ numContacts ] = contact;
			numContacts++;
		}
	}

	// Sort the times of impact from first to last
	if ( numContacts > 1 ) {
		qsort( contacts, numContacts, sizeof( contact_t ), CompareContacts );
	}

	//
	// Apply ballistic impulses
	//
	float accumulatedTime = 0.0f;
	for ( int i = 0; i < numContacts; i++ ) {
		contact_t & contact = contacts[ i ];
		const float dt = contact.timeOfImpact - accumulatedTime;

		// Position update
		for ( int j = 0; j < m_bodies.size(); j++ ) {
			m_bodies[ j ].Update( dt );
		}

		ResolveContact( contact );
		accumulatedTime += dt;
	}

	// Update the positions for the rest of this frame's time
	const float timeRemaining = dt_sec - accumulatedTime;
	if ( timeRemaining > 0.0f ) {
		for ( int i = 0; i < m_bodies.size(); i++ ) {
			m_bodies[ i ].Update( timeRemaining );
		}
	}
}