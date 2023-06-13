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
    // body.m_position = Vec3(0, 0, 50);
    // body.m_orientation = Quat(0, 0, 0, 1);
    // body.m_invMass = 10.0f;
    // body.m_elasticity = 0.9f;
    // body.m_shape = new ShapeSphere(1.0f);
    // m_bodies.push_back(body);

    body.m_position = Vec3(0, 0, 10);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_invMass = 1.0f;
    body.m_elasticity = 0.9f;
    body.m_shape = new ShapeSphere(1.0f);
    m_bodies.push_back(body);

    // Add a ” ground ”spherethat won ’ tf a l lunder theinfluenceofg r a v i t y
    body.m_position = Vec3(0, 0, -1000);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_invMass = 0.0f;
    body.m_elasticity = 1.0f;
    body.m_shape = new ShapeSphere(1000.0f);
    m_bodies.push_back(body);
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
            // 跳过无限重量的物体
            if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass) {
                continue;
            }
            contact_t contact;
            if (Intersect(bodyA, bodyB, contact)) {
                ResolveContact(contact);
            }
        }
    }

    for (int i = 0; i < m_bodies.size(); i++) {
        // Positionupdate
        m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
    }
}