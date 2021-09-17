//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

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
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	// Tweak: aligned sphere around world origin
	Body body;

	// Dynamic Bodies
	int nGridDynamic = 6;
	for ( int x = 0; x < nGridDynamic; x++ ) {
		for ( int y = 0; y < nGridDynamic; y++ ) {
			float radius = 0.5f;
			float xx = -0.5f * float(nGridDynamic - 1 ) * radius * 1.5f + x * radius * 1.5f;
			float yy = -0.5f * float(nGridDynamic - 1 ) * radius * 1.5f + y * radius * 1.5f;
			body.m_position = Vec3( xx, yy, 10.0f );
			body.m_orientation = QuatCreate( 0, 0, 0, 1 );
			body.m_linearVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere( radius );
			m_bodies.push_back( body );
		}
	}

	// Static "floor"
	int nGridStatic = 3;
	for (int x = 0; x < nGridStatic; x++) {
		for (int y = 0; y < nGridStatic; y++) {
			float radius = 80.0f;
			float xx = -0.5f * float(nGridStatic - 1) * radius * 0.25f + x * radius * 0.25f;
			float yy = -0.5f * float(nGridStatic - 1) * radius * 0.25f + y * radius * 0.25f;
			body.m_position = Vec3( xx, yy, -radius );
			body.m_orientation = QuatCreate( 0, 0, 0, 1 );
			body.m_linearVelocity.Zero();
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere( radius );
			m_bodies.push_back( body );
		}
	}
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