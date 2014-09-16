
#ifndef _BT_DEFRAC_BODY_COMPONENT_H
#define _BT_DEFRAC_BODY_COMPONENT_H

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"

#include "btElement.h"
#include "LinearMath/btPoolAllocator.h"

#include "btSparseMatrix.h"


class btMaterial;

//A btDefracBodyComponent contains a set of nodes and tetrahedrons where, considering that two tetrahedrons
//are adjacent iff they share a btNode, its adjacency graph is a connected graph
class btDefracBodyComponent : public btCollisionObject
{
private:
	btAlignedObjectArray<btNode*> m_nodes;
	btAlignedObjectArray<btTetrahedron*> m_tetrahedrons;
	btAlignedObjectArray<int> m_indices;
    btSparseMatrix* m_K1;//assembled co-rotated stiffness
	btSparseMatrix* m_K2;//like the one above
    std::vector<btScalar> m_invMassVector;
	void assembleMassVector();

public:
	btDefracBodyComponent(const btAlignedObjectArray<btNode*>& nodes, 
		const btAlignedObjectArray<btTetrahedron*>& tetrahedrons, 
		const btAlignedObjectArray<int>& indices);//build from lists of nodes and indices
	~btDefracBodyComponent();

	void reset();
	
	void applyForce(const btVector3& force);
	void applyAcceleration(const btVector3& acc);
	void zeroOutForces();

	void setNodeVelocity(int i, const btVector3& v)
	{
		m_nodes[i]->setVelocity(v);
	}

	void displaceNode(int i, const btVector3& displacement)
	{
		btNode* n = m_nodes[i];
		n->setPosition(n->getPosition() + displacement);
	}

	void applyNodeForce(int i, const btVector3& f)
	{
		m_nodes[i]->applyForce(f);
	}

	void integrateNodeMotion(int i, const btVector3& f, btScalar timeStep)
	{
		btNode* n = m_nodes[i];
		n->applyForce(f);
		n->setVelocity(n->getVelocity() + n->getForce()*n->getInvMass()*timeStep);
		n->setPosition(n->getPosition() + n->getVelocity()*timeStep);
	}

	void setNodeMass(int i, btScalar mass)
	{
		m_nodes[i]->setMass(mass);
        m_invMassVector[i] = m_nodes[i]->getInvMass();
	}

	const std::vector<btScalar>& getInvMassVector() const { return m_invMassVector; }
	btSparseMatrix& getK1() { return *m_K1; }
	btSparseMatrix& getK2() { return *m_K2; }

	const btNode* getNode(int index) const { return m_nodes[index]; }
	btTetrahedron* getTetrahedron(int index) { return m_tetrahedrons[index]; }
	const btTetrahedron* getTetrahedron(int index) const { return m_tetrahedrons[index]; }
	int getNodeIndex(int index) const { return m_indices[index]; }

	int getNodeCount() const { return m_nodes.size(); }
	int getTetrahedronCount() const { return m_tetrahedrons.size(); }

    btVector3n getPositionVector();
    btVector3n getPosition0Vector();
    btVector3n getVelocityVector();
	btVector3n getForceVector();

	static const btDefracBodyComponent* upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType() == CO_USER_TYPE)
			return (const btDefracBodyComponent*)colObj;

		return 0;
	}

	static btDefracBodyComponent* upcast(btCollisionObject* colObj)
	{
		if (colObj->getInternalType() == CO_USER_TYPE)
			return (btDefracBodyComponent*)colObj;

		return 0;
	}
};


class btDefracCollisionShape : public btConcaveShape
{
public:
	btDefracBodyComponent* m_body;

	btDefracCollisionShape(btDefracBodyComponent* body)
	{
		m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE;
		m_body = body;
	}

	virtual ~btDefracCollisionShape(){}

	void	processAllTriangles(btTriangleCallback* /*callback*/,const btVector3& /*aabbMin*/,const btVector3& /*aabbMax*/) const
	{
		//not yet
		//btAssert(0);
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
	{
		//const btAlignedObjectArray<btNode>& nodes = m_body->getNodeArray();
		aabbMin.setValue(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		aabbMax = -aabbMin;

		for(int i=0; i<m_body->getNodeCount(); ++i)
		{
			const btVector3& p = m_body->getNode(i)->getPosition();
			btVector3 tp = t.getBasis()*p + t.getOrigin();
			aabbMin.setMin(tp);
			aabbMax.setMax(tp);
		}
	}


	virtual void setLocalScaling(const btVector3& /*scaling*/){}
	virtual const btVector3& getLocalScaling() const
	{
		static const btVector3 dummy(1,1,1);
		return dummy;
	}
	virtual void calculateLocalInertia(btScalar /*mass*/,btVector3& /*inertia*/) const
	{
		///not yet
		//btAssert(0);
	}
	virtual const char*	getName()const
	{
		return "DefracBody";
	}

};

#endif