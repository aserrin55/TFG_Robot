#pragma once

#include "../../bullet3-2.86/examples/CommonInterfaces/CommonRigidBodyBase.h"

class Robot {
public:
	Robot();
	virtual ~Robot() { }

	void setAppliedForce(btVector3 &applied_force);
	void getAppliedForce(btVector3 &applied_force);


protected:
	btVector3 m_force;
};