
#ifndef DEFRAC_DEMO_H
#define DEFRAC_DEMO_H

#include <string>
#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class btDefracDynamicsWorld;
class btSpring;
class btMaterial;
class btDefracBody;


class DefracDemo : public GlutDemoApplication
{
public:

	
	bool								m_autocam;
	bool								m_cutting;
	bool								m_raycast;
	btScalar							m_animtime;
	btClock								m_clock;
	int									m_lastmousepos[2];
	btVector3							m_impact;
	btVector3							m_goal;
	bool								m_drag;
	std::string							m_meshFilename;


	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btCollisionAlgorithmCreateFunc*	m_boxBoxCF;
	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btDefracBody* m_body;
	btMaterial* m_material;
	btSpring* m_spring;
	btScalar m_young;
	int m_cgMaxIter;

	float m_ms;
	int m_frameCount;
	int m_framesPerUpdate;

	btScalar m_oldPickingDist;
	btVector3 m_pickPos;

public:

	void	initPhysics();

	void	exitPhysics();

	DefracDemo() : 
		m_drag(false),	
		m_spring(NULL), 
		m_young(3000),
		m_cgMaxIter(20),
		m_ms(0.f), 
		m_frameCount(0),
		m_framesPerUpdate(1),
		m_oldPickingDist(0)
	{
		setTexturing(true);
		setShadows(true);
	}

	DefracDemo(const char* filename, btScalar youngModulus=20000, int cgMaxIter=20) : 
		m_drag(false), 
		m_meshFilename(filename), 
		m_young(youngModulus),
		m_cgMaxIter(cgMaxIter),
		m_spring(NULL), m_ms(0.f), 
		m_frameCount(0), 
		m_framesPerUpdate(1),
		m_oldPickingDist(0)
	{
		setTexturing(true);
		setShadows(true);
	}

	virtual ~DefracDemo()
	{
		exitPhysics();
	}

	void initDemo();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	static DemoApplication* Create()
	{
		DefracDemo* demo = new DefracDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual const btDefracDynamicsWorld*	getDefracDynamicsWorld() const
	{
		return (btDefracDynamicsWorld*) m_dynamicsWorld;
	}

	virtual btDefracDynamicsWorld*	getDefracDynamicsWorld()
	{
		return (btDefracDynamicsWorld*) m_dynamicsWorld;
	}

	const std::string& getMeshFilename() const { return m_meshFilename; }

	//
	void	clientResetScene();
	void	renderme();
	void	keyboardCallback(unsigned char key, int x, int y);
	void	mouseFunc(int button, int state, int x, int y);
	void	mouseMotionFunc(int x,int y);

};

#endif
