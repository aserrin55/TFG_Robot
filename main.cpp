#include "main.h"
#include "../../bullet3-2.86/src/btBulletDynamicsCommon.h"
#include "../../bullet3-2.86/examples/CommonInterfaces/CommonRigidBodyBase.h"
#include "Robot.h"
#pragma comment(lib,"opengl32.lib")

#define GLEW_STATIC

#define DT 1.f / 60.f
#define CNT_FORCE -5.0
double force_Robot2X = CNT_FORCE;
double force_Robot2Y = CNT_FORCE;
btScalar theta = -0.22;


struct MoveBox : public CommonRigidBodyBase
{
	Robot *m_pRobot1;
	Robot *m_pRobot2;
	MoveBox(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~MoveBox() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 25;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3] = { 0,0,0 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void MoveBox::initPhysics()
{
	m_guiHelper->setUpAxis(1);
	
	///Con este método hacemos la inicialización de la configuración de colisiones, del dispacher y del dynamicsWorld
	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	/// inicializamos las formas, la primera el suelo
	{
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	///creamos la box dinámica obj1
	{

		btBoxShape* colShape = createBoxShape(btVector3(1, 1, 1));
		m_collisionShapes.push_back(colShape);
		btTransform startTransform;
		startTransform.setIdentity();
		btScalar	mass(1.5f);
		bool isDynamic = (mass != 0.f);
		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);
		startTransform.setOrigin(btVector3(5.0, 0.5, 0.0));
		createRigidBody(mass, startTransform, colShape);
	}

	///creating a kinematic robot obj2 
	//{
	//	m_pRobot1 = new Robot();
	//	btCollisionShape* robot1Shape = new btSphereShape(btScalar(1));
	//	m_collisionShapes.push_back(robot1Shape);

	//	btTransform robot1startTransform;
	//	robot1startTransform.setIdentity();
	//	robot1startTransform.setOrigin(btVector3(0.5, 0.0, 0.0));

	//	// if mass != 0.f object is dynamic
	//	btScalar robot1mass(1.1f);
	//	btVector3 robot1localInertia(0, 0, 0);
	//	robot1Shape->calculateLocalInertia(robot1mass, robot1localInertia);

	//	btDefaultMotionState* robot1MotionState = new btDefaultMotionState(robot1startTransform);
	//	btRigidBody::btRigidBodyConstructionInfo Robot1rbInfo(robot1mass, robot1MotionState
	//		, robot1Shape, robot1localInertia);
	//	btRigidBody* Robot1body = new btRigidBody(Robot1rbInfo);

	//	m_dynamicsWorld->addRigidBody(Robot1body);
	//}

	///creating a kinematic robot obj3
	{
		m_pRobot2 = new Robot();
		btCollisionShape* robot2Shape = new btSphereShape(btScalar(1));
		m_collisionShapes.push_back(robot2Shape);

		btTransform robot2startTransform;
		robot2startTransform.setIdentity();
		robot2startTransform.setOrigin(btVector3(3, 0.0, 2.0));

		// if mass != 0.f object is dynamic
		btScalar robot2mass(1.1f);
		btVector3 robot2localInertia(0, 0, 0);
		robot2Shape->calculateLocalInertia(robot2mass, robot2localInertia);

		btDefaultMotionState* robot2MotionState = new btDefaultMotionState(robot2startTransform);
		btRigidBody::btRigidBodyConstructionInfo Robot2rbInfo(robot2mass, robot2MotionState
			, robot2Shape, robot2localInertia);
		btRigidBody* Robot2body = new btRigidBody(Robot2rbInfo);

		m_dynamicsWorld->addRigidBody(Robot2body);
	}


	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void MoveBox::renderScene()
{
	m_dynamicsWorld->stepSimulation(DT);
	int numCollision = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int j = m_dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;
		btVector3 applied_force1;
		btScalar omega;
		btVector3 applied_force2;
		if (body && body->getMotionState())
		{
			switch (j)
			{
			case 0: //floor, no motion state needed because it won't move
				trans = obj->getWorldTransform(); break;
			case 1: //box, we have to take the transformation from the motion state
					//because it may have been moved
				break;

			case 2:	//robot, we have to set ourselves the transformation and communicate it to Bullet
					//through the motion state
				//m_pRobot1->setAppliedForce(btVector3(25, 0, 0));
				//m_pRobot1->getAppliedForce(applied_force1);
				//body->applyCentralForce(applied_force1);
			{
				omega = body->getAngularVelocity().getZ();
				//theta = theta + omega.angle*DT;
				theta = theta + omega*DT;
				force_Robot2X = CNT_FORCE * cos(theta);
				force_Robot2Y = CNT_FORCE * sin(theta);
				body->applyCentralForce(btVector3(force_Robot2X, 0, force_Robot2Y));
				break;
			}
				
			case 3:
				//omega = body->getAngularVelocity();
				////theta = theta + omega.angle*DT;
				//btScalar omegaAngle = omega.angle(omega);
				//theta = theta + omegaAngle*DT;
				//force_Robot2X = force_Robot2X + cos(theta);
				//force_Robot2Y = force_Robot2Y + sin(theta);
				//body->applyCentralForce(btVector3(force_Robot2X,0, force_Robot2Y));
				break;

			}
		}
	}

	CommonRigidBodyBase::renderScene();

}

CommonExampleInterface*    StandaloneExampleCreateFunc(CommonExampleOptions& options)
{
	return new MoveBox(options.m_guiHelper);

}

B3_STANDALONE_EXAMPLE(StandaloneExampleCreateFunc)



