
#include "btSpring.h"
#include "btElement.h"

btSpring::btSpring(btTetrahedron* tetrahedron, const btVector4 &volCoords, btScalar stiffness, btScalar restLength):
	m_tetrahedron(tetrahedron),
	m_volumeCoords(volCoords),
	m_stiffness(stiffness),
	m_restLength(restLength)
{
	m_source = m_tetrahedron->getWorldCoordinates(m_volumeCoords);
}

void btSpring::applyForces()
{
	const btVector3 p1 = m_tetrahedron->getWorldCoordinates(m_volumeCoords);
	const btVector3 L = m_source - p1;
	btScalar l = L.length();

	if(l-m_restLength > 0.001)
	{
		const btVector3 f = L*(1/l)*(l-m_restLength)*m_stiffness;
		m_tetrahedron->applyForce(f, m_volumeCoords);
	}
}