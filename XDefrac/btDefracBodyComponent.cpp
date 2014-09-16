
#include "btDefracBodyComponent.h"
#include "btDefracUtils.h"
#include <boost/timer.hpp>


btDefracBodyComponent::btDefracBodyComponent(const btAlignedObjectArray<btNode*>& nodes, 
											 const btAlignedObjectArray<btTetrahedron*>& tetrahedrons, 
											 const btAlignedObjectArray<int>& indices):
	m_K1(NULL),
	m_K2(NULL),
    m_invMassVector(nodes.size())
{
	btCollisionObject::m_internalType = CO_USER_TYPE;
	m_collisionShape = new btDefracCollisionShape(this);
	m_collisionShape->setMargin(0.25);

	btAssert(indices.size() == tetrahedrons.size()*4);//4 indices for each tetrahedron

	//copy each array
	m_nodes.reserve(nodes.size());
	for(int i=0; i<nodes.size(); ++i)
		m_nodes.push_back(nodes[i]);

	m_tetrahedrons.reserve(tetrahedrons.size());
	for(int i=0; i<tetrahedrons.size(); ++i)
		m_tetrahedrons.push_back(tetrahedrons[i]);

	m_indices.reserve(indices.size());
	for(int i=0; i<indices.size(); ++i)
		m_indices.push_back(indices[i]);

    std::set<btMatrixIndex> matrixIndices;
    
    for (int t=0; t<m_tetrahedrons.size(); ++t) {
        for (int i=0; i<4; ++i) {
            int ii = m_indices[t*4 + i];
            for (int j=0; j<4; ++j) {
                int jj = m_indices[t*4 + j];
                btMatrixIndex mi = {ii, jj};
                matrixIndices.insert(mi);
            }
        }
    }
    
    m_K1 = new btSparseMatrix(m_nodes.size(), matrixIndices);
    m_K2 = new btSparseMatrix(m_nodes.size(), matrixIndices);
    
	//const int kSize = 3*m_nodes.size();
	//m_RKR_1.resize(kSize, kSize, 0);
	//m_RK.resize(kSize, kSize, 0);
	assembleMassVector();
}

btDefracBodyComponent::~btDefracBodyComponent()
{
    delete m_K1;
    delete m_K2;
}

btVector3n btDefracBodyComponent::getPositionVector()
{
	btVector3n r(m_nodes.size());

	for(int i=0; i<m_nodes.size(); ++i)
	{
		r[i] = m_nodes[i]->getPosition();
	}

	return r;
}

btVector3n btDefracBodyComponent::getPosition0Vector()
{
	btVector3n r(m_nodes.size());

	for(int i=0; i<m_nodes.size(); ++i)
	{
		r[i] = m_nodes[i]->getPosition0();
	}

	return r;
}

btVector3n btDefracBodyComponent::getVelocityVector()
{
	btVector3n r(m_nodes.size());

	for(int i=0; i<m_nodes.size(); ++i)
	{
		r[i] = m_nodes[i]->getVelocity();

	}

	return r;
}

btVector3n btDefracBodyComponent::getForceVector()
{
	btVector3n r(m_nodes.size());

	for(int i=0; i<m_nodes.size(); ++i)
	{
		r[i] = m_nodes[i]->getForce();
	}

	return r;
}

void btDefracBodyComponent::assembleMassVector()
{
	for(int i=0; i<m_nodes.size(); ++i)
	{
		m_invMassVector[i] = m_nodes[i]->getInvMass();
	}
}

void btDefracBodyComponent::applyForce(const btVector3 &force)
{
	for(int i=0; i<m_nodes.size(); ++i)
		m_nodes[i]->applyForce(force);
}

void btDefracBodyComponent::applyAcceleration(const btVector3 &acc)
{
	for(int i=0; i<m_nodes.size(); ++i)
		m_nodes[i]->applyAcceleration(acc);
}
void btDefracBodyComponent::zeroOutForces()
{
	for(int i=0; i<m_nodes.size(); ++i)
		m_nodes[i]->setForce(btVector3(0,0,0));
}

