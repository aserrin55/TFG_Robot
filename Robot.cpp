#include "Robot.h"

Robot::Robot()
{
	m_force = btVector3(0.0, 0.0, 0.0);
}

void Robot::getAppliedForce(btVector3 &applied_force)
{
	applied_force = m_force;
}

void Robot::setAppliedForce(btVector3 &applied_force)
{
	m_force = applied_force;
}