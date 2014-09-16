
#ifndef BT_MATERIAL_H
#define BT_MATERIAL_H

#include "LinearMath/btScalar.h"
#include <Eigen/Dense>

class btMaterial
{
private:
	btScalar m_e;//Young modulus
	btScalar m_nu;//Poisson ratio
    Eigen::Matrix<btScalar, 6, 6, Eigen::RowMajor> m_E;
	void computeE();

public:
	btMaterial(btScalar youngModulus, btScalar poissonRatio);
	
	Eigen::Matrix<btScalar, 6, 6, Eigen::RowMajor>& getE() { return m_E; }
	btScalar getYoungModulus() const { return m_e; }
	btScalar getPoissonRatio() const { return m_nu; }
	void setYoungModulus(btScalar e);
	void setPoissonRatio(btScalar nu);
	void setYoungModulusAndPoissonRatio(btScalar youngModulus, btScalar poissonRatio);
};


#endif