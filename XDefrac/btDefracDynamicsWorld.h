
#ifndef _BT_DEFRAC_DYNAMICS_WORLD_H
#define _BT_DEFRAC_DYNAMICS_WORLD_H

#include "LinearMath/btHashMap.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

class btDefracBody;
class btDefracBodyComponent;
class btSpring;

class btDefracDynamicsWorld : public btDiscreteDynamicsWorld
{
public:
	enum ODESolver
	{
		ODE_EXPLICIT_EULER,
		ODE_IMPLICIT_EULER
	};

private:
	btHashMap<btHashKey<btDefracBodyComponent*>, btDefracBody*> m_componentToBody;
	btAlignedObjectArray<btDefracBody*> m_defracBodies;
	btAlignedObjectArray<btSpring*> m_springs;
	unsigned int m_cgMaxIter;
	unsigned int m_lastNumIter;

	virtual void internalSingleStepSimulation(btScalar timeStep);
	void integrateMotionImplicitEuler(btDefracBodyComponent* component, btScalar timeStep);
	void integrateMotionExplicitEuler(btDefracBodyComponent* component, btScalar timeStep);

	ODESolver odeSolver;

public:
	btDefracDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,
						  btConstraintSolver* constraintSolver,
						  btCollisionConfiguration* collisionConfiguration);
	~btDefracDynamicsWorld();

	void addDefracBody(btDefracBody* body, short int collisionFilterGroup=btBroadphaseProxy::DefaultFilter,
					   short int collisionFilterMask=btBroadphaseProxy::AllFilter);
	void removeDefracBody(btDefracBody* body);
	void removeDefracBodyComponent(btDefracBodyComponent* component);

	const btDefracBody* getDefracBody(int i) const { return m_defracBodies[i]; }
	 btDefracBody* getDefracBody(int i) { return m_defracBodies[i]; }
	int getNumDefracBodies() { return m_defracBodies.size(); }

	virtual void removeCollisionObject(btCollisionObject* collisionObject);
	
	void addSpring(btSpring* spring);
	void removeSpring(btSpring* spring);

	void setCGMaxIter(unsigned int maxIter) { m_cgMaxIter = maxIter; }
	unsigned int getCGMaxIter() { return m_cgMaxIter; }

	unsigned int getLastNumIter() { return m_lastNumIter; }

	virtual void debugDrawWorld();
	void setODESolver(ODESolver solver) { odeSolver = solver; }
	ODESolver getODESolver() { return odeSolver; }
};

#endif