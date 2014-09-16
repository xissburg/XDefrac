
#include "btDefracDynamicsWorld.h"
#include "LinearMath/btIDebugDraw.h"
#include "btDefracBody.h"
#include "btDefracBodyComponent.h"
#include "btSpring.h"

#include "btSparseMatrix.h"

#include <boost/timer.hpp>


btDefracDynamicsWorld::btDefracDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,
											 btConstraintSolver* constraintSolver,
											 btCollisionConfiguration* collisionConfiguration)
	:btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration),
	m_cgMaxIter(10),
	odeSolver(ODE_IMPLICIT_EULER)
{

}

btDefracDynamicsWorld::~btDefracDynamicsWorld()
{

}

void btDefracDynamicsWorld::addDefracBody(btDefracBody* body, 
										 short int collisionFilterGroup,
										 short int collisionFilterMask)
{
	m_defracBodies.push_back(body);

	for(int i=0; i<body->getComponentCount(); ++i)
	{
		btDefracBodyComponent* c = body->getComponent(i);
		btCollisionWorld::addCollisionObject(c, collisionFilterGroup, collisionFilterMask);
		btHashKey<btDefracBodyComponent*> key((long)c);
		m_componentToBody.insert(key, body);
	}
}

void btDefracDynamicsWorld::removeDefracBody(btDefracBody* body)
{
	m_defracBodies.remove(body);

	for(int i=0; i<body->getComponentCount(); ++i)
		btCollisionWorld::removeCollisionObject(body->getComponent(i));
}

void btDefracDynamicsWorld::removeDefracBodyComponent(btDefracBodyComponent* component)
{
	btHashKey<btDefracBodyComponent*> key((long)component);
	btDefracBody** pbody = m_componentToBody.find(key);

	if(pbody != NULL)
	{
		btDefracBody* body = *pbody;
		body->removeComponent(component);

		if(body->getComponentCount() == 0)
			removeDefracBody(body);
	}
}

void btDefracDynamicsWorld::removeCollisionObject(btCollisionObject* collisionObject)
{
	btDefracBodyComponent* component = btDefracBodyComponent::upcast(collisionObject);

	if (component)
	{
		//find which defracBody owns this component, remove it from the body, then if the
		//body has 0 components, remove the body from the simulation too
		btHashKey<btDefracBodyComponent*> key((long)component);
		btDefracBody** pbody = m_componentToBody.find(key);
		btAssert(pbody != NULL);
		btDefracBody* body = *pbody;
		body->removeComponent(component);

		if(body->getComponentCount() == 0)
			removeDefracBody(body);
	}
	else
		btDiscreteDynamicsWorld::removeCollisionObject(collisionObject);
}

void btDefracDynamicsWorld::addSpring(btSpring* spring)
{
	m_springs.push_back(spring);
}

void btDefracDynamicsWorld::removeSpring(btSpring* spring)
{
	m_springs.remove(spring);
}

void btDefracDynamicsWorld::debugDrawWorld()
{
	btDiscreteDynamicsWorld::debugDrawWorld();

	if (getDebugDrawer())
	{
		for (int i=0;i<m_defracBodies.size();i++)
		{
			btDefracBody* body = m_defracBodies[i];

			for(int j=0; j<body->getTetrahedronCount(); ++j)
			{
				const btTetrahedron* t = body->getTetrahedron(j);

				const btVector3 p[] = {t->getNode(0)->getPosition(),
									   t->getNode(1)->getPosition(),
									   t->getNode(2)->getPosition(),
									   t->getNode(3)->getPosition()};

				if(!(m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawWireframe))
				{
					for(int k=0; k<4; ++k) //for each face
					{
						m_debugDrawer->drawTriangle(p[(k+0)%4],
													p[(k+1)%4],
													p[(k+2)%4], 
													btVector3(0.45, 0.95, 1), 1);
					}
				}
				else
				{
					for(int k=0; k<4; ++k) //for each node
					{
						for(int m=0; m<4; ++m)
						{
							if(m != k)
							{
								m_debugDrawer->drawLine(p[k], p[m], btVector3(0.45, 0.95, 1));
							}
						}
					}
				}

				if(m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawAabb)
				{
					btVector3 min, max;
					t->getAABB(min, max);
					m_debugDrawer->drawAabb(min, max, btVector3(0.9, 0.9, 0.9));
				}
			}
		}
	}	
}

/** Conjugate Gradient **/

int pcg_solve(const btSparseMatrix& A, btVector3n& x, const btVector3n& b, const size_t maxiter = 10, const double rTOL = 1e-6, const double aTOL = 1e-14)
{
    const int size = x.size();
    int iteration = 0;
    
    btVector3n resid(size);
    btVector3n g(size);
    btVector3n d1(size);
    float alpha, beta, norm, h1, h2, norm_0;
    
    resid = b - A*x;
    
    g = resid;
    
    norm = resid.dot(resid);
    norm_0 = norm;
    ++iteration;
    
    d1 = resid;
    h2 = resid.dot(d1);
    
    while ((iteration < maxiter) && (norm > (aTOL*aTOL)) && ((norm/norm_0) > (rTOL * rTOL)) ) {
        h1 = h2;
        
        d1 = A*g; 
        
        h2 = g.dot(d1);
        
        alpha = h1/h2;
        
        x += alpha*g;
        resid -= alpha*d1; 
        
        d1 = resid;
        h2 = resid.dot(d1);      
        
        beta = h2/h1; 
        g = beta*g + d1;
        
        norm = resid.dot(resid);
        
        ++iteration;
    }
    return iteration;
}

void btDefracDynamicsWorld::integrateMotionImplicitEuler(btDefracBodyComponent* component, 
														 btScalar timeStep)
{
	btSparseMatrix& K1 = component->getK1();
	btSparseMatrix& K2 = component->getK2();

    K1.setZero();
    K2.setZero();

	//boost::timer t;
	//t.restart();

	for(int t=0; t<component->getTetrahedronCount(); ++t)
	{
		const btTetrahedron* pt = component->getTetrahedron(t);
        const btMatrix3x3 r = pt->getRotation();

		for(int i=0; i<4; ++i)
		{
			int ii = component->getNodeIndex(t*4 + i);

			for(int j=0; j<4; ++j)
			{
				int jj = component->getNodeIndex(t*4 + j);
                btMatrix3x3 kij = pt->getStiffnessBlock(i*4 + j);
                btMatrix3x3 k2ij = r * kij;
                btMatrix3x3 k1ij = k2ij * r.transpose();
                
                K1(ii, jj) += k1ij;
                K2(ii, jj) += k2ij;
			}
		}
	}
	
	//std::cout << "Assembly time: " << t.elapsed() << std::endl;

	btVector3n w = component->getPositionVector();
	btVector3n v = component->getPosition0Vector();
	btVector3n f = component->getForceVector();
	btVector3n x = component->getVelocityVector();

	btScalar alpha = 0.1f;
	btScalar beta = 0.1f;
    
	btSparseMatrix A(btSparseMatrix::addDiagonal(btSparseMatrix::multiplyDiagonalLeft(K1, component->getInvMassVector()) * (timeStep*(alpha + timeStep)), timeStep*beta + 1));

    btVector3n b =  x + ((component->getInvMassVector() * (f - (K1*w) + (K2*v))) * timeStep);
    
    m_lastNumIter = pcg_solve(A, x, b, m_cgMaxIter, 1e-3, 1e-6);

	for(int i=0; i<component->getNodeCount(); ++i)
	{
		const btVector3& velocity = x[i];
		component->setNodeVelocity(i, velocity);
		component->displaceNode(i, velocity*timeStep);
	}
}

void btDefracDynamicsWorld::integrateMotionExplicitEuler(btDefracBodyComponent* component, 
														 btScalar timeStep)
{
	btSparseMatrix& K1 = component->getK1();
	btSparseMatrix& K2 = component->getK2();

	static const btMatrix3x3 mZero(0,0,0,0,0,0,0,0,0);
    
	for(int t=0; t<component->getTetrahedronCount(); ++t)
	{
		for(int i=0; i<4; ++i)
		{
			int ii = component->getNodeIndex(t*4 + i);
            
			for(int j=0; j<4; ++j)
			{
				int jj = component->getNodeIndex(t*4 + j);
                
				K1(ii, jj) = mZero;
				K2(ii, jj) = mZero;
			}
		}
	}

	for(int t=0; t<component->getTetrahedronCount(); ++t)
	{
		const btTetrahedron* pt = component->getTetrahedron(t);
        const btMatrix3x3 r = pt->getRotation();
        
		for(int i=0; i<4; ++i)
		{
			int ii = component->getNodeIndex(t*4 + i);
            
			for(int j=0; j<4; ++j)
			{
				int jj = component->getNodeIndex(t*4 + j);
                btMatrix3x3 kij = pt->getStiffnessBlock(i*4 + j);
                btMatrix3x3 k2ij = r * kij;
                btMatrix3x3 k1ij = k2ij * r.transpose();
                
                K1(ii, jj) += k1ij;
                K2(ii, jj) += k2ij;
			}
		}
	}

    btVector3n w = component->getPositionVector();
	btVector3n v = component->getPosition0Vector();
	btVector3n f = component->getForceVector();
	f += K2*v - K1*w;

	for(int i=0; i<component->getNodeCount(); ++i)
		component->integrateNodeMotion(i, f[i], timeStep);
}

void btDefracDynamicsWorld::internalSingleStepSimulation(btScalar timeStep)
{
	btDiscreteDynamicsWorld::internalSingleStepSimulation(timeStep);
	//const static int nIterations = 1;
	//timeStep /= nIterations;

	//for(int k=0; k<nIterations; ++k)

	for(int i=0; i<m_springs.size(); ++i)
		m_springs[i]->applyForces();

	if(odeSolver == ODE_IMPLICIT_EULER)
		for(int i=0; i<m_defracBodies.size(); ++i)
		{
			btDefracBody* body = m_defracBodies[i];

			for(int j=0; j<body->getComponentCount(); ++j)
			{
				btDefracBodyComponent* c = body->getComponent(j);
				c->applyAcceleration(m_gravity);
				integrateMotionImplicitEuler(c, timeStep);
				c->zeroOutForces();
			}
		}
	else if(odeSolver == ODE_EXPLICIT_EULER)
		for(int i=0; i<m_defracBodies.size(); ++i)
		{
			btDefracBody* body = m_defracBodies[i];

			for(int j=0; j<body->getComponentCount(); ++j)
			{
				btDefracBodyComponent* c = body->getComponent(j);
				c->applyAcceleration(m_gravity);
				integrateMotionExplicitEuler(c, timeStep);
				c->zeroOutForces();
			}
		}
}