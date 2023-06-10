//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Broadphase.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"

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
Scene::~Scene()
{
    for (int i = 0; i < m_bodies.size(); i++) {
        delete m_bodies[i].m_shape;
    }
    m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset()
{
    for (int i = 0; i < m_bodies.size(); i++) {
        delete m_bodies[i].m_shape;
    }
    m_bodies.clear();

    Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize()
{
    Body body;
    body.m_position = Vec3(0, 0, 10);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_invMass = 1.0f;
    body.m_shape = new ShapeSphere(1.0f);
    m_bodies.push_back(body);

    // Add a ” ground ”spherethat won ’ tf a l lunder theinfluenceofg r a v i t y
    body.m_position = Vec3(0, 0, -1000);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_invMass = 0.0f;
    body.m_shape = new ShapeSphere(1000.0f);
    m_bodies.push_back(body);
}

bool Intersect(const Body* bodyA, const Body* bodyB)
{
    const Vec3 ab = bodyB->m_position - bodyA->m_position;
    const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
    const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

    const float radiusAB = sphereA->m_radius + sphereB->m_radius;
    const float lengthSquare = ab.GetLengthSqr();
    if (lengthSquare <= (radiusAB * radiusAB)) {
        return true;
    }
    return false;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
    for (int i = 0; i < m_bodies.size(); i++) {
        Body* body = &m_bodies[i];
        // Gravityneeds to be an impulse
        // I = dp ,F = dp/ dt => dp = F*dt => I = F*dt
        //  F = mgs
        float mass = 1.0f / body->m_invMass;
        Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
        body->ApplyImpulseLinear(impulseGravity);
    }

    for (int i = 0; i < m_bodies.size(); i++) {
        for (int j = i + 1; j < m_bodies.size(); j++) {
            Body* bodyA = &m_bodies[i];
            Body* bodyB = &m_bodies[j];
            if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass) {
                continue;
            }
            if (Intersect(bodyA, bodyB)) {
                bodyA->m_linearVelocity.Zero();
                bodyB->m_linearVelocity.Zero();
            }
        }
    }

    for (int i = 0; i < m_bodies.size(); i++) {
        // Positionupdate
        m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
    }
}