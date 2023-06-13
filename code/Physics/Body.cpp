//
//  Body.cpp
//
#include "Body.h"
#include "../Math/Quat.h"
/*
====================================================
Body::Body
====================================================
*/
Body::Body()
    : m_position(0.0f)
    , m_orientation(0.0f, 0.0f, 0.0f, 1.0f)
    , m_shape(NULL)
{
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
    const Vec3 centerOfMass = m_shape->GetCenterOfMass();
    const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
    return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
    const Vec3 centerOfMass = m_shape->GetCenterOfMass();
    return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const
{
    Vec3 tmp = worldPt - GetCenterOfMassWorldSpace();
    Quat inverseOrient = m_orientation.Inverse();
    Vec3 bodySpace = inverseOrient.RotatePoint(tmp);
    return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& worldPt) const
{
    Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(worldPt);
    return worldSpace;
}

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
    Mat3 inertiaTensor = m_shape->InertiaTensor();
    Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
    return invInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
    Mat3 inertiaTensor = m_shape->InertiaTensor();
    Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
    Mat3 orient = m_orientation.ToMat3();
    //这里涉及到坐标空间转换
    //world -> local-> world
    invInertiaTensor = orient * invInertiaTensor * orient.Transpose();
    return invInertiaTensor;
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
    if (0.0f == m_invMass) {
        return;
    }

    // p = mv
    // dp = m dv = J
    // => dv = J / m
    m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyImpulseAngular( const Vec3 & impulse ) {
	if ( 0.0f == m_invMass ) {
		return;
	}

	// L = I w = r x p
	// dL = I dw = r x J 
	// => dw = I^-1 * ( r x J )
	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float maxAngularSpeed = 30.0f; // 30 rad/s is fast enough for us. But feel free to adjust.
	if ( m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed ) {
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}