
#include "btDefracUtils.h"
#include "btDefracBody.h"

#include <sstream>
#include <fstream>

btDefracBody* btDefracUtils::CreateFromTetgenFile(const std::string &baseFilename, btScalar mass, btMaterial* material)
{
	std::stringstream ss0;
	ss0 << baseFilename << ".node";
	std::ifstream nodeFile(ss0.str().c_str());

	if(nodeFile.fail()) 
		return NULL;

	std::stringstream ss1;
	ss1 << baseFilename << ".ele";
	std::ifstream eleFile(ss1.str().c_str());

	if(eleFile.fail())
		return NULL;

	//Nodes
	int nNodes = 0;
	int nDimensions = 0;
	int nAttributes = 0;
	int hasBounds = 0;

	nodeFile >> nNodes;
	nodeFile >> nDimensions;
	nodeFile >> nAttributes;
	nodeFile >> hasBounds;

	btAlignedObjectArray<btVector3> nodePosition;
	nodePosition.reserve(nNodes);

	for(int i=0; i<nNodes; ++i)
	{
		int index = 0;
		btScalar x, y, z;
		nodeFile >> index;
		nodeFile >> x;
		nodeFile >> y;
		nodeFile >> z;
		nodePosition.push_back(btVector3(x, y, z));
	}

	//Tetrahedrons
	int nTetrahedrons = 0;

	eleFile >> nTetrahedrons;
	eleFile >> nDimensions;
	eleFile >> nAttributes;

	btAlignedObjectArray<int> nodeIndex;
	nodeIndex.reserve(nTetrahedrons*4);

	for(int i=0; i<nTetrahedrons; ++i)
	{	
		int index = 0;
		int ni = 0;
		eleFile >> index;

		for(int j=0; j<4; ++j)
		{
			eleFile >> ni;
			nodeIndex.push_back(ni-1);
		}
	}

	btDefracBody* body = new btDefracBody(nodePosition, nodeIndex, mass, material);

	return body;
}

btMatrix3x3 btDefracUtils::OrthonormalizeColumns(const btMatrix3x3& m)
{
	btMatrix3x3 r;
	r[0] = btVector3(m[0].x(), m[1].x(), m[2].x()).normalized();
	btVector3 mr1(m[0].y(), m[1].y(), m[2].y());
	r[1] = mr1 - r[0].dot(mr1)*r[0];
	r[1].normalize();
	r[2] = r[0].cross(r[1]);
	return r.transpose();
}

//From: http://www.torquepowered.com/community/blogs/view/309
bool btDefracUtils::SegmentAABBIntersect(const btVector3 &start, const btVector3 &end, 
						  const btVector3 &min, const btVector3 &max, btScalar *time)  
{  
	btScalar st,et,fst = 0,fet = 1;

	for (int i = 0; i < 3; i++) 
	{  
		if (start[i] < end[i])
		{  
			if (start[i] > max[i] || end[i] < min[i])  
				return false;  
			btScalar di = end[i] - start[i];  
			st = (start[i] < min[i])? (min[i] - start[i]) / di: 0;  
			et = (end[i] > max[i])? (max[i] - start[i]) / di: 1;  
		}  
		else 
		{  
			if (end[i] > max[i] || start[i] < min[i])  
				return false;
			btScalar di = end[i] - start[i];  
			st = (start[i] > max[i])? (max[i] - start[i]) / di: 0;  
			et = (end[i] < min[i])? (min[i] - start[i]) / di: 1;  
		}  

		if (st > fst) fst = st;  
		if (et < fet) fet = et;  

		if (fet < fst)  
			return false;
	}  

	*time = fst;  
	return true;  
}  