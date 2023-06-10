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

    body1->m_linearVelocity.Zero();
    body2->m_linearVelocity.Zero();

    // p33
    const float tA = body1->m_invMass / (body1->m_invMass + body2->m_invMass);
    const float tB = body1->m_invMass / (body1->m_invMass + body2->m_invMass);

    const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    body1->m_position += ds * tA;
    body1->m_position -= ds * tB;
}