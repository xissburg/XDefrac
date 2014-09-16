
#include "btMaterial.h"


btMaterial::btMaterial(btScalar youngModulus, btScalar poissonRatio):
	m_e(youngModulus),
	m_nu(poissonRatio),
	m_E(6, 6)
{
	computeE();
}

void btMaterial::computeE()
{
	const btScalar c = m_e/((1+m_nu)*(1-2*m_nu));
    m_E.setZero();

	m_E(0,0)=c*(1-m_nu), m_E(0,1)=c*m_nu,     m_E(0,2)=c*m_nu,     m_E(0,3)=0, m_E(0,4)=0, m_E(0,5)=0;
	m_E(1,0)=c*m_nu,     m_E(1,1)=c*(1-m_nu), m_E(1,2)=c*m_nu,     m_E(1,3)=0, m_E(1,4)=0, m_E(1,5)=0;
	m_E(2,0)=c*m_nu,     m_E(2,1)=c*m_nu,     m_E(2,2)=c*(1-m_nu), m_E(2,3)=0, m_E(2,4)=0, m_E(2,5)=0;
	m_E(3,0)=0, m_E(3,1)=0, m_E(3,2)=0, m_E(3,3)=c*(0.5f-m_nu), m_E(3,4)=0, m_E(3,5)=0;
	m_E(4,0)=0, m_E(4,1)=0, m_E(4,2)=0, m_E(4,3)=0, m_E(4,4)=c*(0.5f-m_nu), m_E(4,5)=0;
	m_E(5,0)=0, m_E(5,1)=0, m_E(5,2)=0, m_E(5,3)=0, m_E(5,4)=0, m_E(5,5)=c*(0.5f-m_nu);
}

void btMaterial::setYoungModulus(btScalar e)
{
	m_e = e;
	computeE();
}

void btMaterial::setPoissonRatio(btScalar nu)
{
	m_nu = nu;
	computeE();
}

void btMaterial::setYoungModulusAndPoissonRatio(btScalar youngModulus, btScalar poissonRatio)
{
	m_e = youngModulus;
	m_nu = poissonRatio;
	computeE();
}