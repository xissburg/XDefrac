
#include "btDefracBody.h"
#include "btElement.h"
#include "btDefracBodyComponent.h"

btDefracBody::btDefracBody(const btAlignedObjectArray<btVector3>& nodePosition, 
						   const btAlignedObjectArray<int>& indices, btScalar mass, 
						   btMaterial *material)
{
	btAssert(indices.size()%4 == 0);//4 indices(nodes) for each tetrahedron
	
	const int numNodes = nodePosition.size();
	const int numTets = indices.size()/4;

	m_nodes.reserve(numNodes);
	m_tetrahedrons.reserve(numTets);

	const btScalar nodeMass = mass/numNodes;

	for(int i=0; i<numNodes; ++i)
	{
		void* mem = btAlignedAlloc(sizeof(btNode), 16);
		btNode* n = new (mem) btNode(nodePosition[i], nodeMass);
		m_nodes.push_back(n);
	}

	for(int i=0; i<numTets; ++i)
	{
		void* mem = btAlignedAlloc(sizeof(btTetrahedron), 16);
		btNode* nodes[] = {m_nodes[indices[i*4+0]], 
						   m_nodes[indices[i*4+1]], 
						   m_nodes[indices[i*4+2]],
						   m_nodes[indices[i*4+3]]};

		btTetrahedron* t = new (mem) btTetrahedron(nodes, material);
		m_tetrahedrons.push_back(t);
	}

	//by now assume the body is initially made of only one component
	void* mem = btAlignedAlloc(sizeof(btDefracBodyComponent), 16);
	btDefracBodyComponent* c = new (mem) btDefracBodyComponent(m_nodes, m_tetrahedrons, indices);
	m_components.push_back(c);
}

btDefracBody::~btDefracBody()
{
	for(int i=0; i<m_components.size(); ++i)
		btAlignedFree(m_components[i]);

	for(int i=0; i<m_tetrahedrons.size(); ++i)
		btAlignedFree(m_tetrahedrons[i]);

	for(int i=0; i<m_nodes.size(); ++i)
		btAlignedFree(m_nodes[i]);
}

void btDefracBody::removeComponent(btDefracBodyComponent* c)
{
	m_components.remove(c);
	btAlignedFree(c);
}

void btDefracBody::reset()
{
	for(int i=0; i<m_nodes.size(); ++i)
	{
		btNode* n = m_nodes[i];
		n->setPosition(n->getPosition0());
		n->setVelocity(btVector3(0, 0, 0));
		n->setForce(btVector3(0, 0, 0));
	}
}