
#ifndef BT_SPRING_H
#define BT_SPRING_H

#include "LinearMath/btVector3.h"
#include "btElement.h"


class btSpring
{
private:
	btTetrahedron* m_tetrahedron;
	btVector4 m_volumeCoords;
	btVector3 m_source;
	btScalar m_stiffness;
	btScalar m_restLength;

public:
	btSpring(btTetrahedron* tetrahedron, const btVector4& volCoord, btScalar stiffness, btScalar restLength);
	
	void setTetrahedron(btTetrahedron* tetrahedron) { m_tetrahedron = tetrahedron; }
	void setSourcePosition(const btVector3& p) { m_source = p; }
	const btVector3& getSourcePosition() const { return m_source; }
	
	void applyForces();
};

#endif