//
//  main.cpp
//  UnitTests
//
//  Created by xiss burg on 8/6/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "gtest/gtest.h"
#include "btVector3n.h"
#include "btSparseMatrix.h"


#define VN_SIZE 2


class btMatrixIndexTest : public ::testing::Test
{
    
};


class btVector3nTest : public ::testing::Test
{
protected:
    btVector3nTest() : v1(VN_SIZE), v2(VN_SIZE), v3(VN_SIZE) {}
    
    virtual void SetUp()
    {
        v1[0] = btVector3(1, 2, 3); v1[1] = btVector3(4, 5, 6);
        v2[0] = btVector3(4, 5, 6); v2[1] = btVector3(1, 2, 3);
    }
    
    btVector3n v1, v2, v3;
};


class btSparseMatrixTest : public ::testing::Test
{
protected:
    btSparseMatrixTest() :
        m(1,0,0,0,1,0,0,0,1),
        S(5, indices)
    {
        
    }
    
    virtual void SetUp()
    {
        std::set<btMatrixIndex>::const_iterator it = indices.begin();
        for (; it != indices.end(); ++it) {
            S(it->i, it->j) = m;
        }
    }
    
    
    btSparseMatrix S;
    const btMatrix3x3 m;
    static std::set<btMatrixIndex> indices;
    
    static std::set<btMatrixIndex> __initIndices()
    {
        btMatrixIndex mi[] = {
            {0, 0},
            {0, 2},
            {0, 3},
            {1, 0},
            {1, 1},
            {3, 2}
        };
        std::set<btMatrixIndex> indices(mi, mi + sizeof(mi)/sizeof(btMatrixIndex));
        return indices;
    }
};

std::set<btMatrixIndex> btSparseMatrixTest::indices = btSparseMatrixTest::__initIndices();


TEST_F(btMatrixIndexTest, SmallerOperator)
{
    btMatrixIndex mi0; 
    btMatrixIndex mi1;
    
    mi0 = (btMatrixIndex){2, 4};
    mi1 = (btMatrixIndex){5, 2};
    ASSERT_TRUE(mi0 < mi1);
    
    mi0 = (btMatrixIndex){3, 4};
    mi1 = (btMatrixIndex){3, 8};
    ASSERT_TRUE(mi0 < mi1);
    
    mi0 = (btMatrixIndex){3, 4};
    mi1 = (btMatrixIndex){8, 9};
    ASSERT_TRUE(mi0 < mi1);
    
    mi0 = (btMatrixIndex){4, 4};
    mi1 = (btMatrixIndex){5, 5};
    ASSERT_TRUE(mi0 < mi1);
    
    mi0 = (btMatrixIndex){2, 4};
    mi1 = (btMatrixIndex){2, 4};
    ASSERT_FALSE(mi0 < mi1);
}

TEST_F(btVector3nTest, Add)
{
    v3 = v1 + v2;
    
    ASSERT_EQ(v3[0], v3[1]);
}

TEST_F(btVector3nTest, Subtract)
{
    v3 = v1 - v2;
    
    //ASSERT_EQ(v3[0], v3[1]);
    
    btVector3n vZero(VN_SIZE, 0);
    btVector3n v4 = v3 - v3;
    
    ASSERT_EQ(v4, vZero);
}

TEST_F(btVector3nTest, ScalarMultiply)
{
    btScalar s = 2;
    btVector3n v4(v3 * s);
    
    ASSERT_EQ(v4[0], v3[0] * s);
}

TEST_F(btSparseMatrixTest, Constructor)
{
    std::set<btMatrixIndex>::iterator it = btSparseMatrixTest::indices.begin();
    for (int i=0; i<btSparseMatrixTest::indices.size(); ++i) {
        btMatrixIndex mi = *(it++);
        ASSERT_EQ(S(mi.i,mi.j), m);
    }
}

TEST_F(btSparseMatrixTest, MultiplyDiagonalRight)
{
    std::vector<btScalar> v;
    
    for (int i=0; i<S.size(); ++i) {
        v.push_back(i+2);
    }
    
    btSparseMatrix R(btSparseMatrix::multiplyDiagonalRight(S, v));
    std::set<btMatrixIndex>::iterator it = btSparseMatrixTest::indices.begin();
    
    for (int i=0; i<btSparseMatrixTest::indices.size(); ++i) {
        btMatrixIndex mi = *(it++);
        ASSERT_EQ(R(mi.i,mi.j), S(mi.i,mi.j)*(mi.i+2));
    }
}

TEST_F(btSparseMatrixTest, MultiplyDiagonalLeft)
{
    std::vector<btScalar> v;
    
    for (int i=0; i<S.size(); ++i) {
        v.push_back(i+2);
    }
    
    btSparseMatrix R = btSparseMatrix::multiplyDiagonalLeft(S, v);
    std::set<btMatrixIndex>::iterator it = btSparseMatrixTest::indices.begin();

    for (int i=0; i<btSparseMatrixTest::indices.size(); ++i) {
        btMatrixIndex mi = *(it++);
        ASSERT_EQ(R(mi.i,mi.j), S(mi.i,mi.j)*(mi.j+2));
    }
}

TEST_F(btSparseMatrixTest, MultiplyVector)
{
    btVector3n vn(S.size());
    
    for (int i=0; i<vn.size(); ++i) {
        vn[i].setValue(i+1, i+1, i+1);
    }
    
    btVector3n vr = S * vn;
    
    /*
    std::cout << S << std::endl;
    std::cout << vr << std::endl;
     */
    /*
    for (int i=0; i<btSparseMatrixTest::indices.size(); ++i) {
        btMatrixIndex mi = btSparseMatrixTest::indices[i];
        ASSERT_EQ(R(mi.i,mi.j), S(mi.i,mi.j)*(mi.j+2));
    }
     */
}

TEST_F(btSparseMatrixTest, MultiplyScalar)
{
    btScalar s = 2;
    
    btSparseMatrix Ss = S * s;
    
    for (int i=0; i<S.size(); ++i) {
        for (int j=0; j<S.size(); ++j) {
            ASSERT_EQ(Ss(i,j), S(i,j)*s);
        }
    }
    
    btSparseMatrix sS = s * S;
    
    for (int i=0; i<S.size(); ++i) {
        for (int j=0; j<S.size(); ++j) {
            ASSERT_EQ(sS(i,j), S(i,j)*s);
        }
    }
    
    //ASSERT_EQ(Ss, sS);
    /*
    std::cout << Ss << std::endl;
    std::cout << sS << std::endl;
     */
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

