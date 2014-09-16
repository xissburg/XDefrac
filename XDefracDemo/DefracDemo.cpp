



#include "btBulletDynamicsCommon.h"

#include "DefracDemo.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"


#include "BulletDefrac/btDefracDynamicsWorld.h"
#include "BulletDefrac/btDefracBodyComponent.h"
#include "BulletDefrac/btDefracBody.h"
#include "BulletDefrac/btMaterial.h"
#include "BulletDefrac/btSpring.h"
#include "BulletDefrac/btDefracUtils.h"

#include "GL_ShapeDrawer.h"
#include "GLDebugFont.h"
#include "GlutStuff.h"

//#include "SDL.h"

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static btBvhTriangleMeshShape* trimeshShape =0;
static btRigidBody* staticBody = 0;
static float waveheight = 5.f;

const float TRIANGLE_SIZE=8.f;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f


////////////////////////////////////


void DefracDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT); 

	float ms = getDeltaTimeMicroseconds();
	float dt = ms / 1000000.f;//1.0/60.;	

	if (m_dynamicsWorld)
	{
		
		

//#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum, otherwise 4 at max
		int maxSimSubSteps = m_idle ? 1 : 4;
		//if (m_idle)
		//	dt = 1.0/420.f;

		const btScalar maxTimeStep = 1.f/30;
		
		if(dt > maxTimeStep)
			dt = maxTimeStep;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt, 0, 1.0/30);
		//numSimSteps = m_dynamicsWorld->stepSimulation(dt,10,1./240.f);
		std::cout << "numIter: " << getDefracDynamicsWorld()->getLastNumIter() << std::endl;

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif		

	}

#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 

	/*++m_frameCount;
	if(m_frameCount > m_framesPerUpdate)
	{
		m_ms = m_clock.getTimeMicroseconds()*0.001f;///m_frameCount;
		m_clock.reset();
		//m_frameCount = 0;
	}*/

	renderme(); 

	//render the graphics objects, with center of mass shift

	updateCamera();



#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	
#endif //PRINT_CONTACT_STATISTICS


	glutSwapBuffers();

}


void DefracDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	glFlush();
	glutSwapBuffers();
}

//
// Random
//

static inline btScalar	UnitRand()
{
	return(rand()/(btScalar)RAND_MAX);
}

static inline btScalar	SignedUnitRand()
{
	return(UnitRand()*2-1);
}

static inline btVector3	Vector3Rand()
{
	const btVector3	p=btVector3(SignedUnitRand(),SignedUnitRand(),SignedUnitRand());
	return(p.normalized());
}

void DefracDemo::initDemo()
{
	//build a tetra cube
	btAlignedObjectArray<btVector3> cubeNodes;
	cubeNodes.push_back(btVector3( -2, -2, -2));
	cubeNodes.push_back(btVector3( -2, -2,  2));
	cubeNodes.push_back(btVector3( -2,  2,  2));
	cubeNodes.push_back(btVector3( -2,  2, -2));
	cubeNodes.push_back(btVector3(  2, -2, -2));
	cubeNodes.push_back(btVector3(  2, -2,  2));
	cubeNodes.push_back(btVector3(  2,  2,  2));
	cubeNodes.push_back(btVector3(  2,  2, -2));
	cubeNodes.push_back(btVector3(  6, -2, -2));
	cubeNodes.push_back(btVector3(  6, -2,  2));
	cubeNodes.push_back(btVector3(  6,  2,  2));
	cubeNodes.push_back(btVector3(  6,  2, -2));
	cubeNodes.push_back(btVector3( 10, -2, -2));
	cubeNodes.push_back(btVector3( 10, -2,  2));
	cubeNodes.push_back(btVector3( 10,  2,  2));
	cubeNodes.push_back(btVector3( 10,  2, -2));


	int ci[] = {
		0, 1, 3, 4,
		1, 2, 3, 4, 
		2, 7, 3, 4, 
		1, 5, 2, 4,
		2, 5, 6, 4,
		2, 6, 7, 4,
		4, 5, 7, 8,
		5, 6, 7, 8, 
		6, 11, 7, 8, 
		5, 9, 6, 8,
		6, 9, 10, 8,
		6, 10, 11, 8,
		8, 9, 11, 12,
		9, 10, 11, 12, 
		10, 15, 11, 12, 
		9, 13, 10, 12,
		10, 13, 14, 12,
		10, 14, 15, 12
	};

	btAlignedObjectArray<int> cubeIndices;
	int size = sizeof(ci)/sizeof(int);
	cubeIndices.initializeFromBuffer(ci, size, size);


	btAlignedObjectArray<btVector3> tetraNodes;
	tetraNodes.push_back(btVector3(0, 4, 0));
	tetraNodes.push_back(btVector3(-btSqrt(3)/2*4, 0, -2));
	tetraNodes.push_back(btVector3(0, 0, 4));
	tetraNodes.push_back(btVector3(btSqrt(3)/2*4, 0, -2));
	tetraNodes.push_back(btVector3(0, 0, 4));
	tetraNodes.push_back(btVector3(0, -4, 0));

	btAlignedObjectArray<int> tetraIndices;
	int ti[] = {
		0, 1, 2, 3,
		4, 2, 1, 3
	};
	size = sizeof(ti)/sizeof(int);
	tetraIndices.initializeFromBuffer(ti, size, size);
	
	btDefracDynamicsWorld* world = getDefracDynamicsWorld();
	world->setCGMaxIter(m_cgMaxIter);
	m_material = new btMaterial(m_young, 0.3);

	if(m_meshFilename.size() > 0)
	{
		m_body = btDefracUtils::CreateFromTetgenFile(m_meshFilename, 10, m_material);
	}
	else
	{
		m_body = new btDefracBody(tetraNodes, tetraIndices, 2000, m_material);
	}

	m_body->getComponent(0)->setNodeMass(0, 0);
	//m_body->getComponent(0)->setNodeMass(400, 0);

	world->addDefracBody(m_body);
	//world->setGravity(btVector3(0, 10, 0));

	//heterogeneous material
	/*btMaterial* material = new btMaterial(1000, 0.3);

	for(int i=0; i<400; ++i)
		m_body->getTetrahedron(i)->setMaterial(material);*/

	//stiffness matrix rendering code with SDL
	/*
	SDL_Surface* surface = SDL_CreateRGBSurface(SDL_SWSURFACE, body->getK().size2(), 
		body->getK().size1(), 8, 0xff, 0xff, 0xff, 0);
	SDL_LockSurface(surface);
	char* pixels = (char*)surface->pixels;

	btDefracUtils::CreateMatrixImage(body->getK(), pixels);

	SDL_UnlockSurface(surface);
	SDL_SaveBMP(surface, "K.bmp");
	SDL_FreeSurface(surface);
	*/

	std::cout << "numNodes: " << m_body->getNodeCount() << std::endl;
	std::cout << "numTetrahedrons: " << m_body->getTetrahedronCount() << std::endl;
}

void DefracDemo::clientResetScene()
{
	m_azi = 0;
	m_cameraDistance = 30.f;
	m_cameraTargetPosition.setValue(0,0,0);

	DemoApplication::clientResetScene();
	
	m_body->reset();

	m_autocam						=	false;
	m_raycast						=	false;
	m_cutting						=	false;
}

void DefracDemo::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();

    glEnable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();
	const btVector3 position = m_body->getComponent(0)->getNode(0)->getPosition();
	const btVector3 unit(1,1,1);
	glDisable(GL_LIGHTING);
	idraw->drawBox(position-unit*0.05, position+unit*0.05, btVector3(1,0,0));
	glEnable(GL_LIGHTING);
	
	DemoApplication::renderme();
}



void DefracDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
	case	',':	m_raycast=!m_raycast;break;
	case	';':	m_autocam=!m_autocam;break;
	case	'q':	
		{
			btDefracDynamicsWorld* ddw = getDefracDynamicsWorld();
			if(ddw->getODESolver() == btDefracDynamicsWorld::ODE_EXPLICIT_EULER)
			{
				ddw->setODESolver(btDefracDynamicsWorld::ODE_IMPLICIT_EULER);
				printf("ODE_IMPLICIT_EULER\n");
			}
			else if(ddw->getODESolver() == btDefracDynamicsWorld::ODE_IMPLICIT_EULER)
			{
				ddw->setODESolver(btDefracDynamicsWorld::ODE_EXPLICIT_EULER);
				printf("ODE_EXPLICIT_EULER\n");
			}
		}
	//case	'c':	getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()^fDrawFlags::Clusters);break;
		break;
	default:		DemoApplication::keyboardCallback(key,x,y);
	}
}

//
void DefracDemo::mouseMotionFunc(int x,int y)
{
	if(m_spring)
	{
		btVector3 newRayTo = getRayTo(x,y);
		btVector3 rayFrom;
		btVector3 oldPivotInB = m_spring->getSourcePosition();
		btVector3 newPivotB;

		if (m_ortho)
		{
			newPivotB = oldPivotInB;
			newPivotB.setX(newRayTo.getX());
			newPivotB.setY(newRayTo.getY());
		} 
		else
		{
			rayFrom = m_cameraPosition;
			btVector3 dir = newRayTo-rayFrom;
			dir.normalize();
			dir *= m_oldPickingDist;
			newPivotB = rayFrom + dir;
		}

		m_spring->setSourcePosition(newPivotB);
	}
	else
	{
		DemoApplication::mouseMotionFunc(x,y);
	}
}

//
void DefracDemo::mouseFunc(int button, int state, int x, int y)
{
	if(button == 0)//left button
	{
		if(state == 0) //pressed
		{
			btVector3 rayTo = getRayTo(x,y);
			btVector3 rayFrom;
			if (m_ortho)
			{
				rayFrom = rayTo;
				rayFrom.setZ(-100.f);
			} else
			{
				rayFrom = m_cameraPosition;
			}
	
			btDefracDynamicsWorld* world = getDefracDynamicsWorld();
			btScalar minTime(BT_LARGE_FLOAT);

			for(int b=0; b < world->getNumDefracBodies(); ++b)
			{
				btDefracBody* body = world->getDefracBody(b);

				for(int i=0; i < body->getTetrahedronCount(); ++i)
				{
					btTetrahedron* t = body->getTetrahedron(i);
					btVector3 min, max;
					t->getAABB(min, max);
					btScalar time(0);

					if(btDefracUtils::SegmentAABBIntersect(rayFrom, rayTo, min, max, &time)
						&& time < minTime)
					{
						minTime = time;
						btVector3 pickPos = rayFrom + (rayTo-rayFrom)*time;
						m_oldPickingDist = (pickPos - rayFrom).length();
						
						if(m_spring == NULL)
							m_spring = new btSpring(t, btVector4(0.25,0.25,0.25,0.25), 1000, 0.2);
						else
							m_spring->setTetrahedron(t);

						m_spring->setSourcePosition(pickPos);
					}
				}
			}

			if(m_spring != NULL)
				world->addSpring(m_spring);
		}
		else if(state == 1) //released
		{
			btDefracDynamicsWorld* world = getDefracDynamicsWorld();
			world->removeSpring(m_spring);
			delete m_spring;
			m_spring = NULL;
		}
	}
	
	if(m_spring == NULL)
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}


void DefracDemo::initPhysics()
{
	m_dispatcher=0;
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	
	m_solver = solver;
	
	btDiscreteDynamicsWorld* world = new btDefracDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create concave ground mesh
	m_azi = 0;
	btCollisionShape* groundShape = 0;
	{
		int i;
		int j;

		const int NUM_VERTS_X = 30;
		const int NUM_VERTS_Y = 30;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		gGroundVertices = new btVector3[totalVerts];
		gGroundIndices = new int[totalTriangles*3];

		btScalar offset(-50);

		for ( i=0;i<NUM_VERTS_X;i++)
		{
			for (j=0;j<NUM_VERTS_Y;j++)
			{
				gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);

		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++)
		{
			for (int j=0;j<NUM_VERTS_Y-1;j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = j*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
		groundShape->setMargin(0.5);
	}

	m_collisionShapes.push_back(groundShape);

	btCollisionShape* groundBox = new btBoxShape (btVector3(100,CUBE_HALF_EXTENTS,100));
	m_collisionShapes.push_back(groundBox);

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-30.f,0));
	localCreateRigidBody(0, tr, groundBox);
	
	//	clientResetScene();

	initDemo();
	clientResetScene();
}


void	DefracDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	delete m_spring;
	delete m_body;
	delete m_material;

}







