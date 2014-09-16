
#ifndef BT_DEFRAC_BODY_H
#define BT_DEFRAC_BODY_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"


class btNode;
class btTetrahedron;
class btDefracBodyComponent;
class btMaterial;

class btDefracBody
{
private:
	btAlignedObjectArray<btNode*> m_nodes;
	btAlignedObjectArray<btTetrahedron*> m_tetrahedrons;
	btAlignedObjectArray<btDefracBodyComponent*> m_components;

public:
	btDefracBody(const btAlignedObjectArray<btVector3>& nodePosition, 
		const btAlignedObjectArray<int>& indices, btScalar mass, btMaterial* material);
	~btDefracBody();

	void reset();//resets all nodes to original position with zero velocity and force

	const btNode* getNode(int index) const { return m_nodes[index]; }

	const btTetrahedron* getTetrahedron(int index) const { return m_tetrahedrons[index]; }
	btTetrahedron* getTetrahedron(int index) { return m_tetrahedrons[index]; }

	const btDefracBodyComponent* getComponent(int index) const { return m_components[index]; }
	btDefracBodyComponent* getComponent(int index) { return m_components[index]; }
	void removeComponent(btDefracBodyComponent* c);

	int getNodeCount() const { return m_nodes.size(); }
	int getTetrahedronCount() const { return m_tetrahedrons.size(); }
	int getComponentCount() const { return m_components.size(); }
};

#endif