//
//  btVectorN.h
//  XDefrac
//
//  Created by xiss burg on 8/6/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _BT_VECTOR3N_H
#define _BT_VECTOR3N_H

#include "LinearMath/btVector3.h"
#include <ostream>
#include <vector>


/**
 * A vector where each element is a btVector3. It has n elements, hence its
 * actual size is 3*n.
 */
class btVector3n
{
public:
    /**
     * Constructs a btVector3n containing size btVector3's in it, hence its actual size is
     * 3*size.
     */
    btVector3n(int size) : m_size(size) {
        m_vectors = new btVector3[m_size];
    }
    
    btVector3n(const btVector3n& v) : m_size(v.size()) {
        m_vectors = new btVector3[m_size];
        for (int i=0; i<m_size; ++i) {
            m_vectors[i] = v.m_vectors[i];
        }
    }
    
    /**
     * Constructs a btVector3n containing size btVector3's in it, and initializes all these
     * to (s,s,s).
     */
    btVector3n(int size, btScalar s) : m_size(size) {
        m_vectors = new btVector3[m_size];
        for (int i=0; i<m_size; ++i) {
            m_vectors[i] = btVector3(s, s, s);
        }
    }
    
    ~btVector3n() {
        delete[] m_vectors;
    }
    
    /*
     * Returns the size of this vector in btVector3 scale. Hence, the actual size of this
     * vector is 3*size.
     */
    int size() const {
        return m_size;
    }
    
    btVector3n& operator = (const btVector3n& v)
    {
        if (m_size != v.m_size) {
            m_size = v.m_size;
            delete[] m_vectors;
            m_vectors = new btVector3[m_size];
        }
        
        for (int i=0; i<m_size; ++i) {
            m_vectors[i] = v.m_vectors[i];
        }
        return *this;
    }
    
    /**
     * Returns a mutable reference to the btVector3 at index i.
     */
    btVector3& operator () (int i) {
        return get(i);
    }
    
    btVector3& operator [] (int i) {
        return get(i);
    }
    
    btVector3& get(int i) {
        return m_vectors[i];
    }
    
    /**
     * Returns an imutable reference to the btVector3 at index i.
     */
    const btVector3& operator () (int i) const {
        return get(i);
    }
    
    const btVector3& operator [] (int i) const {
        return get(i);
    }
    
    const btVector3& get(int i) const {
        return m_vectors[i];
    }
    
    btVector3n& operator *= (btScalar s) {
        for (int i=0; i<m_size; ++i) {
            m_vectors[i] *= s;
        }
        return *this;
    }
    
    btVector3n& operator += (const btVector3n& vn) {
        for (int i=0; i<m_size; ++i) {
            m_vectors[i] += vn[i];
        }
        return *this;
    }
    
    btVector3n& operator -= (const btVector3n& vn) {
        for (int i=0; i<m_size; ++i) {
            m_vectors[i] -= vn[i];
        }
        return *this;
    }

    /**
     * Returns the dot product bewteen this and v.
     */
    static btScalar dot(const btVector3n& v1, const btVector3n& v2) {
        btScalar d = 0;
        for (int i=0; i<v1.m_size; ++i) {
            d += btDot(v1[i], v2[i]);
        }
        return d;
    }
    
    btScalar dot(const btVector3n& v) const {
        return btVector3n::dot(*this, v);
    }
    
private:
    btVector3 *m_vectors;
    int m_size;
};


/**
 * Multiplies each btVector3 of vn by s.
 */
inline btVector3n operator * (const btVector3n& vn, btScalar s)
{
    btVector3n ret(vn.size());
    for (int i=0; i<vn.size(); ++i) {
        ret[i] = vn[i] * s;
    }
    return ret;
}

inline btVector3n operator * (btScalar s, const btVector3n& vn)
{
    return vn * s;
}

/**
 * Multiplies the i-th btVector3 of v by sv[i] btScalar.
 */
inline btVector3n operator * (const btVector3n& vn, const std::vector<btScalar>& sv)
{
    assert(sv.size() == vn.size());
    btVector3n ret(vn.size());
    for (int i=0; i<vn.size(); ++i) {
        ret[i] = vn[i] * sv[i];
    }
    return ret;
}

inline btVector3n operator * (const std::vector<btScalar>& sv, const btVector3n& vn)
{
    return vn * sv;
}

/**
 * Adds v1 to v2.
 */
inline btVector3n operator + (const btVector3n& vn1, const btVector3n& vn2)
{
    btVector3n ret(vn1.size());
    for (int i=0; i<vn1.size(); ++i) {
        ret[i] = vn1[i] + vn2[i];
    }
    return ret;
}

/**
 * Subtracts v2 from v1 (v1 - v2).
 */
inline btVector3n operator - (const btVector3n& vn1, const btVector3n& vn2)
{
    btVector3n ret(vn1.size());
    for (int i=0; i<vn1.size(); ++i) {
        ret[i] = vn1[i] - vn2[i];
    }
    return ret;
}

/**
 * Negates v1.
 */
inline btVector3n operator - (const btVector3n& vn)
{
    btVector3n ret(vn.size());
    for (int i=0; i<vn.size(); ++i) {
        ret[i] = -vn[i];
    }
    return ret;
}

/**
 * Comparison.
 */
inline bool operator == (const btVector3n& vn1, const btVector3n& vn2)
{
    for (int i=0; i<vn1.size(); ++i) {
        if (!(vn1[i] == vn2[i])) {
            return false;
        }
    }
    return true;
}

inline bool operator != (const btVector3n& vn1, const btVector3n& vn2)
{
    return !(vn1 == vn2);
}

inline std::ostream& operator << (std::ostream& out, const btVector3n& vn)
{
    out << "[";
    
    for (int i=0; i<vn.size(); ++i) {
        const btVector3& v = vn[i];
        out << "[" << v.x() << " " << v.y() << " " << v.z() << "]";
    }
    
    out << "]" << std::endl;
    
    return out;
}


#endif
