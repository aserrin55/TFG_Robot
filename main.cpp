#include "main.h"
#include "../../bullet3-2.86/src/btBulletDynamicsCommon.h"
#include "../../bullet3-2.86/examples/CommonInterfaces/CommonRigidBodyBase.h"
#include "Robot.h"
#include <math.h>
#pragma comment(lib,"opengl32.lib")

#define GLEW_STATIC

#define DT 1.f / 20.f
#define CNT_FORCE 2.0;
double force_Robot2X = CNT_FORCE;
double force_Robot2Y = CNT_FORCE;
btScalar theta = 2.0;
btRigidBody* Robot2body;

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
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints
		+ btIDebugDraw::DBG_DrawNormals);

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
		btScalar	mass(0.5f);
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
		btScalar robot2mass(2.1f);
		btVector3 robot2localInertia(0, 0, 0);
		robot2Shape->calculateLocalInertia(robot2mass, robot2localInertia);

		btDefaultMotionState* robot2MotionState = new btDefaultMotionState(robot2startTransform);
		btRigidBody::btRigidBodyConstructionInfo Robot2rbInfo(robot2mass, robot2MotionState
			, robot2Shape, robot2localInertia);
		Robot2body = new btRigidBody(Robot2rbInfo);
		Robot2body->setActivationState(DISABLE_DEACTIVATION);
		m_dynamicsWorld->addRigidBody(Robot2body);
	}


	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void MoveBox::renderScene()
{
	double omega = 0.2;

	theta =  theta + omega*DT;

	if (theta > SIMD_2_PI)
		theta -= SIMD_2_PI;

	force_Robot2X = cos(theta)*CNT_FORCE;
	force_Robot2Y = sin(theta)*CNT_FORCE;

	Robot2body->setAngularVelocity(btVector3(0.0, omega, 0.0));
	Robot2body->setLinearVelocity(btVector3(force_Robot2X, 0.0, force_Robot2Y));

	m_dynamicsWorld->stepSimulation(DT,20);

	CommonRigidBodyBase::renderScene();

}

CommonExampleInterface*    StandaloneExampleCreateFunc(CommonExampleOptions& options)
{
	return new MoveBox(options.m_guiHelper);

}

B3_STANDALONE_EXAMPLE(StandaloneExampleCreateFunc)



