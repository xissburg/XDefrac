//
//  btSparseMatrix.h
//  XDefrac
//
//  Created by xiss burg on 8/6/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _BT_SPARSE_MATRIX_H
#define _BT_SPARSE_MATRIX_H

#include "LinearMath/btMatrix3x3.h"
#include "btVector3n.h"
#include <set>
#include <ostream>


#define BT_ZERO_ROW -1


struct btMatrixIndex
{
    int i, j;
    
    bool operator < (const btMatrixIndex& mi) const
    {
        return (i == mi.i && j < mi.j) || (i < mi.i);
    }
};


/**
 * A sparse matrix of 3x3 blocks. Each element is an instance of btMatrix3x3.
 */
class btSparseMatrix
{
public:
    /**
     * Creates a new square sparse matrix with a fixed structure. The indices vector has the (i,j)
     * index of the entries that should be non-zero in the sparse matrix in row-major order.
     */
    btSparseMatrix(int size, const std::set<btMatrixIndex>& indices) :
        m_size(size),
        m_zero(0,0,0,0,0,0,0,0,0)
    {
        m_elements = new btMatrix3x3[indices.size()];
        m_columnIndices = new int[indices.size()];
        m_rowIndices = new int[m_size+1];
        
        m_rowIndices[0] = BT_ZERO_ROW;
        int ri = 0; //index for m_rowIndices
        int pri = -1; //previous matrix row index
        
        std::set<btMatrixIndex>::iterator it = indices.begin();
        
        for (int i=0; i<indices.size(); ++i) {
            m_elements[i].setValue(0,0,0,0,0,0,0,0,0);
            
            const btMatrixIndex& mi = *(it++);
            m_columnIndices[i] = mi.j;
            
            int d = mi.i - pri;
            
            if (d > 0) {
                if (d == 1) {
                    m_rowIndices[ri++] = i;
                }
                else { //d > 1
                    for (int j=0; j<d-1; ++j) {
                        m_rowIndices[ri++] = BT_ZERO_ROW;
                    }
                    m_rowIndices[ri++] = i;
                }
            }
            
            pri = mi.i;
        }
        
        while (ri < m_size) {
            m_rowIndices[ri++] = BT_ZERO_ROW;
        }
        
        m_rowIndices[m_size] = (int)indices.size();
    }
    
    btSparseMatrix(const btSparseMatrix& S) :
        m_size(S.m_size),
        m_zero(0,0,0,0,0,0,0,0,0)
    {
        int nonZeros = S.m_rowIndices[S.m_size];
        m_elements = new btMatrix3x3[nonZeros];
        m_columnIndices = new int[nonZeros];
        m_rowIndices = new int[m_size+1];
        
        for (int i=0; i<nonZeros; ++i) {
            m_elements[i] = S.m_elements[i];
            m_columnIndices[i] = S.m_columnIndices[i];
        }
        
        for (int i=0; i<m_size+1; ++i) {
            m_rowIndices[i] = S.m_rowIndices[i];
        }
    }
    
    ~btSparseMatrix()
    {
        delete[] m_elements;
        delete[] m_columnIndices;
        delete[] m_rowIndices;
    }
    
    /**
     * Returns the width = height in 3x3 block scale of this square sparse matrix. The actual
     * width = height is 3*size.
     */
    int size() const
    {
        return m_size;
    }
    
    btSparseMatrix& operator = (const btSparseMatrix& S)
    {
        for (int i=0; i<m_rowIndices[m_size]; ++i) {
            m_elements[i] = S.m_elements[i];
        }
        
        return *this;
    }
    
    /**
     * Returns a mutable reference to the 3x3 block at (i,j)
     */
    btMatrix3x3& operator () (int i, int j)
    {
        btMatrix3x3 *m = __get(i, j);
        
        if (m == NULL) {
            return zero();
        }
        
        return *m;
    }
    
    /**
     * Returns an immutable reference to the 3x3 block at (i,j)
     */
    const btMatrix3x3& operator () (int i, int j) const
    {
        btMatrix3x3 *m = __get(i, j);
        
        if (m == NULL) {
            return zero();
        }
        
        return *m;
    }
    
    /**
     * Multiplies this matrix by v as if v was the diagonal elements of a diagonal matrix. 
     * The multiplies are done block-wise: the 3x3 block at (i,j) is multiplied by v(i).
     * v.size() must be equal to this.size().
     */
    static btSparseMatrix multiplyDiagonalLeft(const btSparseMatrix& S, const std::vector<btScalar>& v) //v * this
    {
        btSparseMatrix ret(S);
        
        for (int i=0; i<S.size(); ++i) {
            if (S.m_rowIndices[i] == BT_ZERO_ROW) {
                continue;
            }
            
            int begin = S.m_rowIndices[i];
            int end = S.m_rowIndices[i+1];
            int ii = i+1;
            
            while (end == BT_ZERO_ROW) { //jump over empty(full zero) rows
                end = S.m_rowIndices[++ii];
            }
            
            for (int j=begin; j<end; ++j) {
                ret.m_elements[j] = S.m_elements[j] * v[i];
            }
        }
        
        return ret;
    }
    
    static btSparseMatrix multiplyDiagonalRight(const btSparseMatrix& S, const std::vector<btScalar>& v) //this * v
    {
        btSparseMatrix ret(S);
        
        for (int i=0; i<S.size(); ++i) {
            if (S.m_rowIndices[i] == BT_ZERO_ROW) {
                continue;
            }
            
            int begin = S.m_rowIndices[i];
            int end = S.m_rowIndices[i+1];
            int ii = i+1;
            
            while (end == BT_ZERO_ROW) { //jump over empty(full zero) rows
                end = S.m_rowIndices[++ii];
            }
            
            for (int j=begin; j<end; ++j) {
                int jj = S.m_columnIndices[j];
                ret.m_elements[j] = S.m_elements[j] * v[jj];
            }
        }
        
        return ret;
    }
    
    /**
     * Adds s to the diagonal of the matrix.
     */
    static btSparseMatrix addDiagonal(const btSparseMatrix& S, btScalar s)
    {
        btSparseMatrix ret(S);
        const btMatrix3x3 Is = btMatrix3x3::getIdentity() * s;
        
        for (int i=0; i<S.size(); ++i) {
            if (S.m_rowIndices[i] == BT_ZERO_ROW) {
                continue;
            }
            
            int begin = S.m_rowIndices[i];
            int end = S.m_rowIndices[i+1];
            int ii = i+1;
            
            while (end == BT_ZERO_ROW) { //jump over empty(full zero) rows
                end = S.m_rowIndices[++ii];
            }
            
            for (int j=begin; j<end; ++j) {
                if (S.m_columnIndices[j] > i) {
                    break;
                }
                else if (S.m_columnIndices[j] == i) {
                    ret.m_elements[j] += Is;
                }
            }
        }
        
        return ret;
    }
    
    btSparseMatrix& setZero()
    {
        int nonZeros = m_rowIndices[m_size];
        for (int i=0; i<nonZeros; ++i) {
            m_elements[i] = zero();
        }
        return *this;
    }
    
    friend btVector3n operator * (const btSparseMatrix& S, const btVector3n& v);
    friend btSparseMatrix operator * (const btSparseMatrix& S, btScalar s);
    friend std::ostream& operator << (std::ostream& out, const btSparseMatrix& S);
    
private:
    btMatrix3x3 *m_elements;
    int m_size; //Number of entries in m_elements.
    int *m_columnIndices; //Array containing the column index for each btMatrix3x3 in m_elements.
    int *m_rowIndices; //Array containing the index of the first element of each row in m_elements. The last is the number of elements, for 
    btMatrix3x3 m_zero; //Zero 3x3 matrix. Never access it directly, always use zero().
    
    btMatrix3x3& zero() 
    {
        m_zero.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
        return m_zero;
    }
    
    const btMatrix3x3& zero() const
    {
        return m_zero;
    }
    
    /**
     * Internal utility method to allow code reuse in both the const and non-const versions
     * of the operator (). It returns NULL if this matrix doesn't contain an element at i,j.
     */
    btMatrix3x3 *__get(int i, int j) const 
    {
        if (m_rowIndices[i] == BT_ZERO_ROW) {
            return NULL;
        }
        
        int begin = m_rowIndices[i];
        int end = m_rowIndices[i+1];
        int ii = i+1;
        
        while (end == BT_ZERO_ROW) { //jump over empty(full zero) rows
            end = m_rowIndices[++ii];
        }
        
        for (int jj=begin; jj<end; ++jj) {
            if (m_columnIndices[jj] == j) {
                return &m_elements[jj];
            }
        }
        
        return NULL;
    }
};


/**
 * Performs a matrix-vector multiply. v.size() must be equal to this.size().
 */
inline btVector3n operator * (const btSparseMatrix& S, const btVector3n& v)
{
    btVector3n ret(v.size(), 0);
    
    for (int i=0; i<S.size(); ++i) {
        if (S.m_rowIndices[i] == BT_ZERO_ROW) {
            continue;
        }
        
        int begin = S.m_rowIndices[i];
        int end = S.m_rowIndices[i+1];
        int ii = i+1;
        
        while (end == BT_ZERO_ROW) { //jump over empty(full zero) rows
            end = S.m_rowIndices[++ii];
        }
        
        for (int j=begin; j<end; ++j) {
            int jj = S.m_columnIndices[j];
            ret[i] += S.m_elements[j] * v[jj];
        }
    }
    
    return ret;
}

/**
 * Multiplies all 3x3 blocks by s.
 */
inline btSparseMatrix operator * (const btSparseMatrix& S, btScalar s)
{
    int nonZeros = S.m_rowIndices[S.m_size];
    btSparseMatrix ret(S);
    
    for (int i=0; i<nonZeros; ++i) {
        ret.m_elements[i] = S.m_elements[i] * s;
    }
    
    return ret;
}

inline btSparseMatrix operator * (btScalar s, const btSparseMatrix& S)
{
    return S * s;
}

inline std::ostream& operator << (std::ostream& out, const btSparseMatrix& S)
{
    for (int i=0; i<S.size(); ++i) {
        std::vector<btMatrix3x3> row;
        
        for (int j=0; j<S.size(); ++j) {
            row.push_back(S(i,j));
        }
        
        for (int ii=0; ii<3; ++ii) {
            for (int jj=0; jj<row.size(); ++jj) {
                btMatrix3x3& m = row[jj];
                out << "[" << m[ii][0] << " " << m[ii][1] << " " << m[ii][2] << "] ";
            }
            out << std::endl;
        }
        
        out << std::endl;
    }
    
    return out;
}
                   
#endif
