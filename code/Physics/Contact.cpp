//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact(contact_t& contact)
{
    Body* body1 = contact.bodyA;
    Body* body2 = contact.bodyB;

    const Vec3 ptOnA = body1->BodySpaceToWorldSpace(contact.ptOnA_LocalSpace);
    const Vec3 ptOnB = body2->BodySpaceToWorldSpace(contact.ptOnB_LocalSpace);

    const float invMass1 = body1->m_invMass;
    const float invMass2 = body2->m_invMass;

    const float elasticity1 = body1->m_elasticity;
    const float elasticity2 = body2->m_elasticity;
    const float elasticity = elasticity1 * elasticity2;

    const Mat3 invWorldInertiaA = body1->GetInverseInertiaTensorWorldSpace();
    const Mat3 invWorldInertiaB = body2->GetInverseInertiaTensorWorldSpace();

    const Vec3 n = contact.normal;

    const Vec3 ra = ptOnA - body1->GetCenterOfMassWorldSpace();
    const Vec3 rb = ptOnB - body2->GetCenterOfMassWorldSpace();

    const Vec3 angularJA = (invWorldInertiaA * ra.Cross(n)).Cross(ra);
    const Vec3 angularJB = (invWorldInertiaB * rb.Cross(n)).Cross(rb);
    const float angularFactor = (angularJA + angularJB).Dot(n);

    // Get the world space velocity of the motion and rotation
    const Vec3 velA = body1->m_linearVelocity + body1->m_angularVelocity.Cross(ra);
    const Vec3 velB = body2->m_linearVelocity + body2->m_angularVelocity.Cross(rb);

    // Calculate the collision impulse
    const Vec3 vab = velA - velB;
    const float ImpulseJ = (1.0f + elasticity) * vab.Dot(n) / (invMass1 + invMass2 + angularFactor);
    const Vec3 vectorImpulseJ = n * ImpulseJ;

    body1->ApplyImpulse(ptOnA, vectorImpulseJ * -1.0f);
    body2->ApplyImpulse(ptOnB, vectorImpulseJ * 1.0f);

    //
    // Calculate the impulse caused by friction
    //

    const float frictionA = body1->m_friction;
    const float frictionB = body2->m_friction;
    const float friction = frictionA * frictionB;

    // Find the normal direction of the velocity with respect to the normal of the collision
    const Vec3 velNorm = n * n.Dot(vab);

    // Find the tangent direction of the velocity with respect to the normal of the collision
    const Vec3 velTang = vab - velNorm;

    // Get the tangential velocities relative to the other body
    Vec3 relativeVelTang = velTang;
    relativeVelTang.Normalize();

    const Vec3 inertiaA = (invWorldInertiaA * ra.Cross(relativeVelTang)).Cross(ra);
    const Vec3 inertiaB = (invWorldInertiaB * rb.Cross(relativeVelTang)).Cross(rb);
    const float invInertia = (inertiaA + inertiaB).Dot(relativeVelTang);

    // Calculate the tangential impulse for friction
    const float reducedMass = 1.0f / (body1->m_invMass + body2->m_invMass + invInertia);
    const Vec3 impulseFriction = velTang * reducedMass * friction;

    body1->ApplyImpulse(ptOnA, impulseFriction * -1.0f);
    body2->ApplyImpulse(ptOnB, impulseFriction * 1.0f);

    if (contact.timeOfImpact == 0.0f) {
        const Vec3 ds = ptOnB - ptOnA;

        // p33
        const float tA = invMass1 / (invMass1 + invMass2);
        const float tB = invMass2 / (invMass1 + invMass2);

        body1->m_position += ds * tA;
        body1->m_position -= ds * tB;
    }
}