
#ifndef _BT_ELEMENT_H
#define _BT_ELEMENT_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"

#include "btDefracUtils.h"


class btTetrahedron;
class btMaterial;

class btNode
{
private:
	btVector3 m_position0;//position in reference configuration
	btVector3 m_position;//position in deformed state
	btVector3 m_velocity;//btNode velocity
	btVector3 m_force;//force acting on btNode
	btScalar  m_invMass;//inverse of btNode mass kg^-1
	btAlignedObjectArray<btTetrahedron*> m_adjacentTetrahedrons;//array with references to every tet which contains this btNode

public:
	btNode();
	btNode(const btVector3& position, btScalar mass);

	const btVector3& getPosition() const { return m_position; }
	void setPosition(const btVector3& position) { m_position = position; }

	const btVector3& getPosition0() const { return m_position0; }
	void setPosition0(const btVector3& position0) { m_position0 = position0; }

	const btVector3& getVelocity() const { return m_velocity; }
	void setVelocity(const btVector3& velocity) { m_velocity = velocity; }

	const btVector3& getForce() const { return m_force; }
	void setForce(const btVector3& force) { if(m_invMass > 0) m_force = force; }
	void applyForce(const btVector3& force) { if(m_invMass > 0) m_force += force; }
	void applyAcceleration(const btVector3& acc) { if(m_invMass > 0) m_force += acc/m_invMass; }

	btScalar getInvMass() const { return m_invMass; }

	void setMass(btScalar mass)
	{
		if(mass > 0)
			m_invMass = 1/mass;
		else
			m_invMass = 0;
	}
	
	int getNumAdjacentTetrahedrons() { return m_adjacentTetrahedrons.size(); }
	btTetrahedron* getAdjacentTetrahedron(int index) { return m_adjacentTetrahedrons[index]; }
	void removeAdjacentTetrahedron(btTetrahedron* const& t) { m_adjacentTetrahedrons.remove(t); }
	void addAdjacentTetrahedron(btTetrahedron* const t) { m_adjacentTetrahedrons.push_back(t); }
};


class btTetrahedron
{
private:
	btNode* m_nodes[4];
	btMatrix3x3 m_invV;//element basis matrix, it times a vector computes the vector coordinates in the btTetrahedron's aereal coordinates
	btMatrix3x3_12x12 m_k;//element stiffness matrix
    btMaterial* m_material;

	btTetrahedron();//no default constructor
	void computeBasisMatrix();
	void computeStiffnessMatrix();
	void computeStiffnessMatrix2();

public:
	btTetrahedron(btNode* nodes[4], btMaterial* material);//A tet can only exist given its nodes

    btMatrix3x3 getRotation() const;
	void getCorotatedStiffnessMatrices(btMatrix3x3_12x12& rk, btMatrix3x3_12x12& rkr_1) const;//computes and returns the corotated stifness matrices matrix of this tetrahedron. It is not stored since its very likely that they will change every step

	btScalar getStiffness(int i, int j) const { return m_k.get(i, j); }
	const btMatrix3x3& getStiffnessBlock(int index) const { return m_k.get(index); }

	void getAABB(btVector3& min, btVector3& max) const;

	const btNode* getNode(int index) const { return m_nodes[index]; }

	btVector4 getVolumeCoordinates(const btVector3& p) const;
	btVector3 getWorldCoordinates(const btVector4& p) const;

	void applyForce(const btVector3& f, const btVector3& p);
	void applyForce(const btVector3& f, const btVector4& p);//apply force at volume coordinates p

	const btMaterial* getMaterial() { return m_material; }
	void setMaterial(btMaterial* material)
	{
		m_material = material;
		computeStiffnessMatrix();
	}

	void nodesPosition0Changed()//must be called whenever any of its nodes change position0, recompute stiffness and basis matrices
	{
		computeBasisMatrix();
		computeStiffnessMatrix();
	}
};

#endif