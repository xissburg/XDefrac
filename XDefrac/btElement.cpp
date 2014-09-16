
#include "btElement.h"
#include "btMaterial.h"
#include <Eigen/Dense>


btNode::btNode():
	m_position0(0,0,0),
	m_position(0,0,0),
	m_velocity(0,0,0),
	m_force(0,0,0)
{
}

btNode::btNode(const btVector3& position, btScalar mass):
	m_position0(position),
	m_position(position),
	m_velocity(0,0,0),
	m_force(0,0,0)
{
	setMass(mass);
}

btTetrahedron::btTetrahedron(btNode* nodes[4], btMaterial* material):
	m_material(material)
{
	for(int i=0; i<4; ++i)
	{
		m_nodes[i] = nodes[i];
		m_nodes[i]->addAdjacentTetrahedron(this);
	}

	computeBasisMatrix();
	computeStiffnessMatrix();
}


void btTetrahedron::computeBasisMatrix()
{
	const btVector3& v0 = m_nodes[0]->getPosition0();
	const btVector3& v1 = m_nodes[1]->getPosition0();
	const btVector3& v2 = m_nodes[2]->getPosition0();
	const btVector3& v3 = m_nodes[3]->getPosition0();

	btVector3 vv1(v1 - v0);
	btVector3 vv2(v2 - v0);
	btVector3 vv3(v3 - v0);
	
	btMatrix3x3 V(vv1.x(), vv2.x(), vv3.x(),
				  vv1.y(), vv2.y(), vv3.y(),
				  vv1.z(), vv2.z(), vv3.z());

	m_invV = V.inverse();
}

void btTetrahedron::computeStiffnessMatrix()
{
	const btVector3& v0 = m_nodes[0]->getPosition0();
	const btVector3& v1 = m_nodes[1]->getPosition0();
	const btVector3& v2 = m_nodes[2]->getPosition0();
	const btVector3& v3 = m_nodes[3]->getPosition0();
	btScalar volume6 = ((v1-v2).cross(v0-v1)).dot(v3-v0);
	btScalar v = 1/volume6;

	//gradient of the shape functions
	btVector3 dN0 = (v1-v2).cross(v2-v3)*v;
	btVector3 dN1 = (v3-v0).cross(v2-v3)*v;
	btVector3 dN2 = (v3-v0).cross(v0-v1)*v;
	btVector3 dN3 = (v1-v2).cross(v0-v1)*v;

    Eigen::Matrix<btScalar, 6, 12, Eigen::RowMajor> B;

	B(0,0)=dN0.x(); B(0,1)=0; B(0,2)=0; B(0,3)=dN1.x(); B(0,4)=0; B(0,5)=0; B(0,6)=dN2.x(); B(0,7)=0; B(0,8)=0; B(0,9)=dN3.x(); B(0,10)=0; B(0,11)=0;
	B(1,0)=0, B(1,1)=dN0.y(), B(1,2)=0, B(1,3)=0, B(1,4)=dN1.y(), B(1,5)=0, B(1,6)=0, B(1,7)=dN2.y(), B(1,8)=0, B(1,9)=0, B(1,10)=dN3.y(), B(1,11)=0;
	B(2,0)=0, B(2,1)=0, B(2,2)=dN0.z(), B(2,3)=0, B(2,4)=0, B(2,5)=dN1.z(), B(2,6)=0, B(2,7)=0, B(2,8)=dN2.z(), B(2,9)=0, B(2,10)=0, B(2,11)=dN3.z();
	B(3,0)=dN0.y(), B(3,1)=dN0.x(), B(3,2)=0, B(3,3)=dN1.y(), B(3,4)=dN1.x(), B(3,5)=0, B(3,6)=dN2.y(), B(3,7)=dN2.x(), B(3,8)=0, B(3,9)=dN3.y(), B(3,10)=dN3.x(), B(3,11)=0;
	B(4,0)=0, B(4,1)=dN0.z(), B(4,2)=dN0.y(), B(4,3)=0, B(4,4)=dN1.z(), B(4,5)=dN1.y(), B(4,6)=0, B(4,7)=dN2.z(), B(4,8)=dN2.y(), B(4,9)=0, B(4,10)=dN3.z(), B(4,11)=dN3.y();
	B(5,0)=dN0.z(), B(5,1)=0, B(5,2)=dN0.x(), B(5,3)=dN1.z(), B(5,4)=0, B(5,5)=dN1.x(), B(5,6)=dN2.z(), B(5,7)=0, B(5,8)=dN2.x(), B(5,9)=dN3.z(), B(5,10)=0, B(5,11)=dN3.x();

	Eigen::Matrix<btScalar, 6, 12, Eigen::RowMajor> m_EB = m_material->getE()*B;
	Eigen::Matrix<btScalar, 12, 12, Eigen::RowMajor> k = B.transpose()*m_EB;
	k *= fabsf(volume6)/6;

	for(int i=0; i<12; ++i)
		for(int j=0; j<12; ++j)
			m_k.set(i, j, k(i,j));
}

void btTetrahedron::computeStiffnessMatrix2()
{
	const btVector3& v0 = m_nodes[0]->getPosition0();
	const btVector3& v1 = m_nodes[1]->getPosition0();
	const btVector3& v2 = m_nodes[2]->getPosition0();
	const btVector3& v3 = m_nodes[3]->getPosition0();
	btScalar volume6 = ((v1-v2).cross(v0-v1)).dot(v3-v0);
	btScalar v = 1/volume6;

	//gradient of the shape functions
	btVector3 dN[] = {  (v1-v2).cross(v2-v3)*v,
						(v3-v0).cross(v2-v3)*v,
						(v3-v0).cross(v0-v1)*v,
						(v1-v2).cross(v0-v1)*v };

	const btScalar e = m_material->getYoungModulus();
	const btScalar nu = m_material->getPoissonRatio();
	const btScalar u = 0.5f*e/(1+nu);
	const btScalar l = (e*nu)/((1+nu)*(1-2*nu));

	for(int i=0; i<4; ++i)
		for(int j=0; j<4; ++j)
		{
			btMatrix3x3& kij = m_k.get(i*4 + j);
			kij = btMatrix3x3::getIdentity();
			kij[0][0] = kij[1][1] = kij[2][2] = u*dN[i].dot(dN[j]);
			
			for(int r=0; r<3; ++r)
				for(int s=0; s<3; ++s)
				{
					kij[r][s] += l*dN[i][r]*dN[j][s] + u*dN[j][r]*dN[i][s];
				}

			kij[0] *= abs(volume6)/6;
			kij[1] *= abs(volume6)/6;
			kij[2] *= abs(volume6)/6;
		}
}

btMatrix3x3 btTetrahedron::getRotation() const
{
    //assume that m_invV and m_k are up to date
	const btVector3& w0 = m_nodes[0]->getPosition();
	const btVector3 ww1(m_nodes[1]->getPosition() - w0);
	const btVector3 ww2(m_nodes[2]->getPosition() - w0);
	const btVector3 ww3(m_nodes[3]->getPosition() - w0);
    
	const btMatrix3x3 W(ww1.x(), ww2.x(), ww3.x(),
					    ww1.y(), ww2.y(), ww3.y(),
					    ww1.z(), ww2.z(), ww3.z());
    
	const btMatrix3x3 W_invV(W*m_invV);
    btMatrix3x3 R(btDefracUtils::OrthonormalizeColumns(W_invV));
    
    return R;
}

void btTetrahedron::getCorotatedStiffnessMatrices(btMatrix3x3_12x12& rk, btMatrix3x3_12x12& rkr_1) const
{
	//assume that m_invV and m_k are up to date
	const btVector3& w0 = m_nodes[0]->getPosition();
	const btVector3 ww1(m_nodes[1]->getPosition() - w0);
	const btVector3 ww2(m_nodes[2]->getPosition() - w0);
	const btVector3 ww3(m_nodes[3]->getPosition() - w0);

	const btMatrix3x3 W(ww1.x(), ww2.x(), ww3.x(),
					    ww1.y(), ww2.y(), ww3.y(),
					    ww1.z(), ww2.z(), ww3.z());

	const btMatrix3x3 W_invV(W*m_invV);
	const btMatrix3x3 R(btDefracUtils::OrthonormalizeColumns(W_invV));

	for(int i=0; i<4; ++i)
		for(int j=0; j<4; ++j)
		{
			int i4j = i*4+j;
			rk.get(i4j) = R*m_k.get(i4j);
			rkr_1.get(i4j) = rk.get(i4j).timesTranspose(R);
		}
}

btVector4 btTetrahedron::getVolumeCoordinates(const btVector3& p) const
{
	const btVector3& w1 = m_nodes[0]->getPosition();
	const btVector3& w2 = m_nodes[1]->getPosition();
	const btVector3& w3 = m_nodes[2]->getPosition();
	const btVector3& w4 = m_nodes[3]->getPosition();

	btScalar v6 = ((w1-w2).cross(w2-w3)).dot(w4-w1);
	btScalar v = 1/v6;

	//value of the shape functions at p
	return btVector4(((w3-w4).cross(w2-w3)).dot(p-w2)*v,
					 ((w3-w4).cross(w4-w1)).dot(p-w3)*v, 
					 ((w1-w2).cross(w4-w1)).dot(p-w4)*v,
					 ((w1-w2).cross(w2-w3)).dot(p-w1)*v);
}

btVector3 btTetrahedron::getWorldCoordinates(const btVector4& p) const
{
	const btVector3& w1 = m_nodes[0]->getPosition();
	const btVector3& w2 = m_nodes[1]->getPosition();
	const btVector3& w3 = m_nodes[2]->getPosition();
	const btVector3& w4 = m_nodes[3]->getPosition();

	return btVector3(p.x()*w1 + p.y()*w2 + p.z()*w3 + p.w()*w4);
}

void btTetrahedron::applyForce(const btVector3& f, const btVector3& p)
{
	btVector4 N(getVolumeCoordinates(p));

	//apply forces at nodes
	for(int i=0; i<4; ++i)
		m_nodes[i]->applyForce(f*N[i]);
}

void btTetrahedron::applyForce(const btVector3& f, const btVector4& p)
{
	//apply forces at nodes
	m_nodes[0]->applyForce(f*p.x());
	m_nodes[1]->applyForce(f*p.y());
	m_nodes[2]->applyForce(f*p.z());
	m_nodes[3]->applyForce(f*p.w());
}

void btTetrahedron::getAABB(btVector3& min, btVector3& max) const
{
	min.setValue(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
	max = -min;

	for(int i=0; i<4; ++i)
	{
		const btVector3& p = m_nodes[i]->getPosition();

		for(int j=0; j<3; ++j)
		{
			if(p[j] < min[j])
				min[j] = p[j];
			if(p[j] > max[j])
				max[j] = p[j];
		}
	}
}