
#ifndef _BT_DEFRAC_HELPERS_H
#define _BT_DEFRAC_HELPERS_H

#include <string>
#include "LinearMath/btMatrix3x3.h"

class btDefracBody;
class btMaterial;

class btMatrix3x3_12x12//very application specific, stores a 12x12 matrix organized as 16 btMatrix3x3 blocks in row major order
{
private:
	btMatrix3x3 m[16];

public:
	btMatrix3x3_12x12() {}
	btScalar& operator ()(int i, int j)
	{
		int I = i/3;//block index
		int J = j/3;
		return m[I*4 + J][i-I*3][j-J*3]; 
	}

	btMatrix3x3& get(int i)//returns a reference to the i-th block
	{
		return m[i];
	}

	const btMatrix3x3& get(int i) const //returns a const reference to the i-th block
	{
		return m[i];
	}

	btScalar get(int i, int j) const
	{
		int I = i/3;//block index
		int J = j/3;
		return m[I*4 + J][i-I*3][j-J*3]; 
	}

	void set(int i, int j, btScalar s)
	{
		int I = i/3;//block index
		int J = j/3;
		m[I*4 + J][i-I*3][j-J*3] = s;
	}
};

class btDefracUtils
{
public:
	static btDefracBody* CreateFromTetgenFile(const std::string& baseFilename, btScalar mass, btMaterial* material);
	
	static btMatrix3x3 OrthonormalizeColumns(const btMatrix3x3& m);//orthonormalizes lines of m

	static bool SegmentAABBIntersect(const btVector3 &start, const btVector3 &end, 
						  const btVector3 &min, const btVector3 &max, btScalar *time);

	template <class Matrix>
	static void CreateMatrixImage(const Matrix& M, char* pixels)
	{
		//find the value of the elements different zero with smallest and biggest value in M
		typedef typename Matrix::value_type value_type;
		value_type min=0, max=0;
		int i=0, j=0;

		//initialize min and max to any value in M different zero
		for(; i<M.size1(); ++i)
		{
			for(; j<M.size2(); ++j)
			{
				value_type k = abs(M(i,j));
				
				if(k > 0)
				{
					min = max = k;
					break;//if it breaks here, j != M.size2()
				}
			}

			if(j != M.size2())
				break;
		}

		//find min and max in M
		for(i=0; i<M.size1(); ++i)
		{
			for(j=0; j<M.size2(); ++j)
			{
				value_type k = abs(M(i,j));

				if(k > 0)
				{
					if(k > max)
						max = k;

					if(k < min)
						min = k;
				}
			}
		}

		//fill the image pixels
		for(i=0; i<M.size1(); ++i)
		{
			for(j=0; j<M.size2(); ++j)
			{
				value_type k = abs(M(i,j));

				if(k > 0)
				{
					value_type m = (k - min)/(max - min);//in [0,1] interval
					pixels[i*M.size2() + j] = (char)((int)((1-m)*128));
				}
				else
					pixels[i*M.size2() + j] = 255;
			}
		}
	}
};

#endif