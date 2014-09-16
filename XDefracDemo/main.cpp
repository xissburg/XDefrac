
#include "DefracDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer gDebugDrawer;

int main(int argc,char** argv)
{
	DefracDemo* defracDemo = 0;

	switch(argc)
	{
	case 2:
		defracDemo = new DefracDemo(argv[1]);
		break;
	case 3:
		defracDemo = new DefracDemo(argv[1], atof(argv[2]));
		break;
	case 4:
		defracDemo = new DefracDemo(argv[1], atof(argv[2]), atoi(argv[3]));
		break;
	default:
		defracDemo = new DefracDemo();
	}

	
	defracDemo->initPhysics();
	defracDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
	//defracDemo->setDebugMode(defracDemo->getDebugMode() | btIDebugDraw::DBG_NoHelpText);

	glutmain(1, argv,800,600,"Bullet Physics Demo. http://bulletphysics.com", defracDemo);

	delete defracDemo;
	return 0;

}
