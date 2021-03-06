/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/

/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#pragma OPENCL EXTENSION cl_amd_printf : enable

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
#define b2cl_maxManifoldPoints  2

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
#define b2cl_maxPolygonVertices 8

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
//#define b2cl_velocityThreshold        0.01f
#define b2cl_velocityThreshold      1.0f

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
#define b2cl_aabbMultiplier     2.0f

#define b2_maxFloat FLT_MAX
#define b2_epsilon FLT_EPSILON
#define b2_pi 3.14159265359f
#define b2_maxTranslation 2.0f
#define b2_maxRotation 0.5f*b2_pi
#define b2_baumgarte 0.2f
#define b2_linearSlop 0.005f
#define b2_angularSlop          (2.0f / 180.0f * b2_pi)
#define b2_polygonRadius        (2.0f * b2_linearSlop)
#define b2_maxLinearCorrection 0.2f
#define b2_maxAngularCorrection     (8.0f / 180.0f * b2_pi)

#ifndef MAXFLOAT
#define MAXFLOAT      3.402823466e+38F
#endif


/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
typedef struct b2clContactFeature
{
    uchar indexA;       ///< Feature index on shapeA
    uchar indexB;       ///< Feature index on shapeB
    uchar typeA;        ///< The feature type on shapeA
    uchar typeB;        ///< The feature type on shapeB
} b2clContactFeature;

/// Contact ids to facilitate warm starting.
typedef union b2clContactID
{
    b2clContactFeature cf;
    uint key;                   ///< Used to quickly compare contact ids.
} b2clContactID;

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
typedef struct b2clManifoldPoint
{
    float2 localPoint;      ///< usage depends on manifold type
    float normalImpulse;    ///< the non-penetration impulse
    float tangentImpulse;   ///< the friction impulse
    b2clContactID id;           ///< uniquely identifies a contact point between two shapes
    float dummy; // for alignment
} b2clManifoldPoint;

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
typedef struct b2clManifold
{
    float2 localNormal;                             ///< not use for Type::e_points
    float2 localPoint;                              ///< usage depends on manifold type
    b2clManifoldPoint points[b2cl_maxManifoldPoints];   ///< the points of contact
    int type;
    int pointCount;                             ///< the number of manifold points
    //float2 test;
} b2clManifold;

typedef struct b2clFilter
{
    unsigned short categoryBits;
    unsigned short maskBits;
    short groupIndex;
    short dummy;
} b2clFilter;

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
//////////////
/// We reuse b2clPolygonShape for b2clCircleShape
/// We may have to change this later
typedef struct b2clPolygonShape
{
    float2 m_centroid;
    float2 m_vertices[b2cl_maxPolygonVertices];
    float2 m_normals[b2cl_maxPolygonVertices];
    int m_type;
    float m_radius;
    int m_vertexCount;
    int m_bIsSensor;
    b2clFilter m_filter;
} b2clPolygonShape;

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
typedef struct b2clTransform
{
    float2 p;
    float2 q;
} b2clTransform;

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
typedef struct b2clSweep
{
    float2 localCenter; ///< local center of mass position
    float2 c0, c;       ///< center world positions
    float a0, a;        ///< world angles

    /// Fraction of the current time step in the range [0,1]
    /// c0 and a0 are the positions at alpha0.
    float alpha0;
    float dummy;
}  b2clSweep;

/// Used for computing contact manifolds.
typedef struct b2clClipVertex
{
    float2 v;
    b2clContactID id;
} b2clClipVertex;

typedef struct clb2Velocity
{
    float vx;
    float vy;
    float w;
}clb2Velocity;

typedef struct clb2Position
{
    float cx;
    float cy;
    float a;
}clb2Position;

typedef struct clb2Contact
{
    int color;
    int indexA;
    int indexB;
    float friction;
    float2 normal;
    float invMassA;
    float invIA;
    float invMassB;
    float invIB;
}clb2Contact;

typedef struct clb2Impulse
{
    float normalImpulse1;
    float tangentImpulse1;
    
    float normalImpulse2;
    float tangentImpulse2;
}clb2Impulse;

typedef struct clb2Points
{
    float2 rA1;
    float2 rB1;
    float normalMass1;
    float tangentMass1;
    
    float2 rA2;
    float2 rB2;
    float normalMass2;
    float tangentMass2;
    
    float velocityBias1;
    float velocityBias2;
}clb2Points;

typedef struct b2clFixtureStatic
{
    float m_friction;
    float m_restitution;
    int m_last_uid;
    int dummy; 
} b2clFixtureStatic;

#define MAX_CONNECTED_BODY_INDICES 8
typedef struct b2clBodyStatic
{
    float2 m_localCenter;   ///< local center of mass position
    float /*m_mass,*/ m_invMass;
    // Rotational inertia about the center of mass.
    float /*m_I,*/ m_invI;
    float m_linearDamping;
    float m_angularDamping;
    float m_gravityScale;
    float m_type;
    int m_connectedBodyIndices[MAX_CONNECTED_BODY_INDICES];
    int m_bIsBullet;
    int dummy;
} b2clBodyStatic;

typedef struct b2clBodyDynamic
{
    b2clSweep m_sweep;      // the swept motion for CCD

    float2 m_linearVelocity;
    float2 m_force;
    float m_angularVelocity;
    float m_torque;
    int m_last_uid ;
    int dummy ; 
} b2clBodyDynamic;

// Extract useful information of manifolds for position constraint solver
typedef struct clb2Manifold
{
    float2 localNormal;
    float2 localPoint;
    float2 localPoints1;
    float2 localPoints2;
    int pointCount;
    int type;
    float radiusA;
    float radiusB;
    float2 localCenterA;
    float2 localCenterB;
}clb2Manifold;

typedef struct{
    int type;
    int pointCount;
    float localPointX;
    float localPointY;
    float localNormalX;
    float localNormalY; 
    float point0X;
    float point0Y;
    float point1X;
    float point1Y; 
}clb2SDManifold;

typedef struct{
    int fixtureAIndex;
    int fixtureBIndex;
    int bodyAIndex ; 
    int bodyBIndex ; 
}clb2SDContact;

typedef struct{
    int bodyIndex ;
    float posX;
    float posY ; 
    float posAngle;
    float xfX;
    float xfY;
    float xfS;
    float xfC;
    float alpha;
    float velocityX;
    float velocityY;
    float velocityAngular; 
}clb2SDBody;

typedef struct clb2PositionSolverManifold
{
    float2 normal;
    float2 point;
    float separation;
}clb2PositionSolverManifold;

typedef struct clb2Rotation
{
    float s;
    float c;
}clb2Rotation;

typedef struct clb2Transform
{
    clb2Rotation rotation;
    float2 translation;
}clb2Transform;

typedef struct b2clMat22
{
    float ex[2];
    float ey[2];
}b2clMat22;

typedef struct b2clMat33
{
    float ex[3];
    float ey[3];
    float ez[3];
}b2clMat33;

enum b2clLimitState
{
    e_inactiveLimit,
    e_atLowerLimit,
    e_atUpperLimit,
    e_equalLimits
};

enum b2JointType
{
    e_unknownJoint,
    e_revoluteJoint,
    e_prismaticJoint,
    e_distanceJoint,
    e_pulleyJoint,
    e_mouseJoint,
    e_gearJoint,
    e_wheelJoint,
    e_weldJoint,
    e_frictionJoint,
    e_ropeJoint
};

typedef struct b2clDistanceJointDatastruct
{
    float frequencyHz;
    float dampingRatio;
    float bias;

    float localAnchorA[2];
    float localAnchorB[2];
    float gamma;
    float nlength;

    float u[2];
    float rA[2];
    float rB[2];
    float localCenterA[2];
    float localCenterB[2];
    float invMassA;
    float invMassB;
    float invIA;
    float invIB;
    float mass;
} b2clDistanceJointData;

typedef struct b2clRevoluteJointData
{
    float localAnchorA[2];
    float localAnchorB[2];

    int enableMotor;
    float maxMotorTorque;
    float motorSpeed;

    int enableLimit;
    float referenceAngle;
    float lowerAngle;
    float upperAngle;

    // Solver temp
    float rA[2];
    float rB[2];
    float localCenterA[2];
    float localCenterB[2];
    float invMassA;
    float invMassB;
    float invIA;
    float invIB;
    b2clMat33 mass;         // effective mass for point-to-point constraint.
    float motorMass;    // effective mass for motor/limit angular constraint.
    int limitState;
} b2clRevoluteJointData;

typedef struct b2clPrismaticJointData
{
    // Solver shared
    float localAnchorA[2];
    float localAnchorB[2];
    float localXAxisA[2];
    float localYAxisA[2];
    float referenceAngle;
    float lowerTranslation;
    float upperTranslation;
    float maxMotorForce;
    float motorSpeed;
    int enableLimit;
    int enableMotor;
    int limitState;

    // Solver temp
    float localCenterA[2];
    float localCenterB[2];
    float invMassA;
    float invMassB;
    float invIA;
    float invIB;
    float axis[2], perp[2];
    float s1, s2;
    float a1, a2;
    b2clMat33 K;
    float motorMass;
} b2clPrismaticJointData;

typedef struct b2clGearJointData
{
    int joint1;
    int joint2;
    int typeA;
    int typeB;

    float localAnchorA[2];
    float localAnchorB[2];
    float localAnchorC[2];
    float localAnchorD[2];

    float localAxisC[2];
    float localAxisD[2];

    float referenceAngleA;
    float referenceAngleB;

    float gearConstant;
    float ratio;

    // Solver temp
    float lcA[2], lcB[2], lcC[2], lcD[2];
    float mA, mB, mC, mD;
    float iA, iB, iC, iD;
    float JvAC[2], JvBD[2];
    float JwA, JwB, JwC, JwD;
    float mass;
} b2clGearJointData;

typedef struct b2clPulleyJointData
{
    float groundAnchorA[2];
    float groundAnchorB[2];
    float lengthA;
    float lengthB;
    float localAnchorA[2];
    float localAnchorB[2]; 
    float pulleyConstant; 
    float ratio; 
    float uA[2]; 
    float uB[2]; 
    float rA[2]; 
    float rB[2]; 
    float localCenterA[2];
    float localCenterB[2]; 
    float invMassA; 
    float invMassB; 
    float invIA; 
    float invIB;
    float mass; 
}b2clPulleyJointData; 

typedef struct b2clRopeJointData
{   
    float localAnchorA[2];
    float localAnchorB[2]; 
    float maxLength; 
    float nlength;
    float u[2]; 
    float rA[2]; 
    float rB[2]; 
    float localCenterA[2];
    float localCenterB[2]; 
    float invMassA; 
    float invMassB; 
    float invIA; 
    float invIB;
    float mass; 
    int limitState; 
} b2clRopeJointData; 

typedef struct b2clWheelJointData
{
    float frequencyHz;
    float dampingRatio;

    // Solver shared
    float localAnchorA[2];
    float localAnchorB[2];
    float localXAxisA[2];
    float localYAxisA[2];

    float maxMotorTorque;
    float motorSpeed;
    int enableMotor;

    // Solver temp
    float localCenterA[2];
    float localCenterB[2];
    float invMassA;
    float invMassB;
    float invIA;
    float invIB;

    float ax[2], ay[2];
    float sAx, sBx;
    float sAy, sBy;

    float mass;
    float motorMass;
    float springMass;

    float bias;
    float gamma;
} b2clWheelJointData;

typedef struct b2clWeldJointData
{
    float frequencyHz;
    float dampingRatio;
    float bias;

    // Solver shared
    float localAnchorA[2];
    float localAnchorB[2];
    float referenceAngle;
    float gamma;

    // Solver temp
    float rA[2];
    float rB[2];
    float localCenterA[2];
    float localCenterB[2];
    float invMassA;
    float invMassB;
    float invIA;
    float invIB;
    b2clMat33 mass;
} b2clWeldJointData;

typedef struct b2clMouseJointData
{
    float localAnchorB[2];
    float targetA[2];
    float frequencyHz;
    float dampingRatio;
    float beta;
    
    // Solver shared
    float maxForce;
    float gamma;

    // Solver temp
    float rB[2];
    float localCenterB[2];
    float invMassB;
    float invIB;
    b2clMat22 mass;
    float C[2];
} b2clMouseJointData;

typedef struct b2clFrictionJointData
{
    float localAnchorA[2];
    float localAnchorB[2];

    float maxForce; 
    float maxTorque; 

    // Solver temp
    float rA[2];
    float rB[2];
    float localCenterA[2];
    float localCenterB[2];
    float invMassA;
    float invMassB;
    float invIA;
    float invIB;
    b2clMat22 linearMass;
    float angularMass; 
} b2clFrictionJointData;

typedef struct clJoint
{
    int index;

    // impulse : 4 floats
    union a1{
        struct x1{
            float impulse[3];
        }x;
        struct y1{
            float scalarImpulse;
            float springImpulse;
        }y;
        struct z1{
            float linearImpulse[2];
            float angularImpulse;
        }z;
    }a;
    float motorImpulse;

    int color;
    int type;
    int collideConnected;

    int indexA, indexB, indexC, indexD;

    union b1{
        b2clDistanceJointData distanceJointData;
        b2clRevoluteJointData revoluteJointData;
        b2clPrismaticJointData prismaticJointData;
        b2clGearJointData gearJointData;
        b2clPulleyJointData pulleyJointData;
        b2clRopeJointData ropeJointData;
        b2clWheelJointData wheelJointData;
        b2clWeldJointData weldJointData;
        b2clMouseJointData mouseJointData;
        b2clFrictionJointData frictionJointData;
    }b;
} b2clJoint;

typedef struct
{
    int index;
    float nimpulse[4];
} b2clJointImpulseNode;

typedef struct
{
    int isCollide;
    float normal[2];
    float fraction;
    unsigned int shapeIndex;
} b2clRayCastOutput;

typedef struct
{
    float2 m_buffer[2];
    float2 m_vertices[b2cl_maxPolygonVertices];
    int m_count;
    float m_radius;
} b2clDistanceProxy;

typedef struct 
{
    float metric;       ///< length or area
    short count;
    unsigned char indexA[3];    ///< vertices on shape A
    unsigned char indexB[3];    ///< vertices on shape B
} b2clSimplexCache;

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
typedef struct 
{
    b2clDistanceProxy proxyA;
    b2clDistanceProxy proxyB;
    b2clTransform transformA;
    b2clTransform transformB;
    bool useRadii;
} b2clDistanceInput;

/// Output for b2Distance.
typedef struct
{
    float2 pointA;      ///< closest point on shapeA
    float2 pointB;      ///< closest point on shapeB
    float ndistance;
    int iterations; ///< number of GJK iterations used
} b2clDistanceOutput;

typedef struct
{
    b2clDistanceProxy proxyA;
    b2clDistanceProxy proxyB;
    b2clSweep sweepA;
    b2clSweep sweepB;
    float tMax;     // defines sweep interval [0, tMax]
    float dummy;
} b2clTOIInput;

enum b2clTOIOutputState
{
    e_unknown,
    e_failed,
    e_overlapped,
    e_touching,
    e_separated
};

typedef struct
{
    int state;
    float t;
} b2clTOIOutput;

typedef struct
{
    float2 wA;      // support point in proxyA
    float2 wB;      // support point in proxyB
    float2 w;       // wB - wA
    float a;        // barycentric coordinate for closest point
    int indexA; // wA index
    int indexB; // wB index
} b2clSimplexVertex;

typedef struct 
{
    b2clSimplexVertex m_v1, m_v2, m_v3;
    int m_count;
} b2clSimplex;

//typedef struct{
//float v[30];
//} testData; 

//==============================================================================================
// Math functions
//==============================================================================================

/// Multiply two rotations: q * r
inline float2 b2clMul_TwoRotation(const float2 q, const float2 r)
{
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
    // s = qs * rc + qc * rs
    // c = qc * rc - qs * rs
    float2 qr;
    qr.x = q.x * r.y + q.y * r.x;
    qr.y = q.y * r.y - q.x * r.x;
    return qr;
}

/// Transpose multiply two rotations: qT * r
inline float2 b2clMulT_TwoRotation(const float2 q, const float2 r)
{
    // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
    // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
    // s = qc * rs - qs * rc
    // c = qc * rc + qs * rs
    float2 qr;
    qr.x = q.y * r.x - q.x * r.y;
    qr.y = q.y * r.y + q.x * r.x;
    return qr;
}

/// Rotate a vector
inline float2 b2clMul_Rotate(const float2 q, const float2 v)
{
    float2 result;
    result.x = q.y * v.x - q.x * v.y;
    result.y = q.x * v.x + q.y * v.y;

    return result;
}

/// Inverse rotate a vector
inline float2 b2clMulT_Rotate(const float2 q, const float2 v)
{
    float2 result;
    result.x = q.y * v.x + q.x * v.y;
    result.y = -q.x * v.x + q.y * v.y;

    return result;
}

/// Transform a vector
inline float2 b2clMul_Transform(const b2clTransform *T, const float2 v)
{
    float2 result;
    result.x = (T->q.y * v.x - T->q.x * v.y) + T->p.x;
    result.y = (T->q.x * v.x + T->q.y * v.y) + T->p.y;

    return result;
}

/// Invert transform a vector
inline float2 b2clMulT_Transform(const b2clTransform *T, const float2 v)
{
    float px = v.x - T->p.x;
    float py = v.y - T->p.y;
    float2 result;
    result.x = (T->q.y * px + T->q.x * py);
    result.y = (-T->q.x * px + T->q.y * py);

    return result;
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b2clTransform b2clMul_TwoTransform(const b2clTransform *A, const b2clTransform *B)
{
    b2clTransform C;
    C.q = b2clMul_TwoRotation(A->q, B->q);
    C.p = b2clMul_TwoRotation(A->q, B->p) + A->p;
    return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b2clTransform b2clMulT_TwoTransform(const b2clTransform *A, const b2clTransform *B)
{
    b2clTransform C;
    C.q = b2clMulT_TwoRotation(A->q, B->q);
    C.p = b2clMulT_TwoRotation(A->q, B->p - A->p);
    return C;
}

/// Perform the dot product on two vectors.
inline float b2clDot(const float2 a, const float2 b)
{
    return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float b2clCross_VV(const float2 a, const float2 b)
{
    return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline float2 b2clCross_VS(const float2 a, float s)
{
    return (float2)(s * a.y, -s *a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline float2 b2clCross_SV(float s, const float2 a)
{
    return (float2)(-s * a.y, s * a.x);
}

inline float b2clClamp(float a, float low, float high)
{
    return max(low, min(a, high));
}

inline float2 b2clMin(const float2 a, const float2 b)
{
    return (float2)(min(a.x, b.x), min(a.y, b.y));
}

inline float2 b2clMax(const float2 a, const float2 b)
{
    return (float2)(max(a.x, b.x), max(a.y, b.y));
}

inline float b2clDistanceSquared(const float2 a, const float2 b)
{
    float2 c = a - b;
    return b2clDot(c, c);
}

inline float b2clDistance(const float2 a, const float2 b)
{
    float2 c = a - b;
    return sqrt(b2clDot(c, c));
}

inline float b2clLengthSquared(const float2 v)
{
    return b2clDot(v, v);
}

inline float b2clAbs(float a)
{
    return a > 0.0f ? a : -a;
}

/// Perform the dot product on two vectors.
inline float b2clDot3(const float3 a, const float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float3 b2clCross3_VV(const float3 a, const float3 b)
{
    return (float3)(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
inline float3 b2clMat33Solve(const b2clMat33 mat, const float3 b)
{
    float3 ex = (float3)(mat.ex[0], mat.ex[1], mat.ex[2]);
    float3 ey = (float3)(mat.ey[0], mat.ey[1], mat.ey[2]);
    float3 ez = (float3)(mat.ez[0], mat.ez[1], mat.ez[2]);

    float det = b2clDot3(ex, b2clCross3_VV(ey, ez));

    if (det != 0.0f)
    {
        det = 1.0f / det;
    }
    float3 x;
    x.x = det * b2clDot3(b, b2clCross3_VV(ey, ez));
    x.y = det * b2clDot3(ex, b2clCross3_VV(b, ez));
    x.z = det * b2clDot3(ex, b2clCross3_VV(ey, b));
    return x;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
inline float2 b2clMat33Solve22(const b2clMat33 mat, const float2 b)
{
    float2 ex = (float2)(mat.ex[0], mat.ex[1]);
    float2 ey = (float2)(mat.ey[0], mat.ey[1]);

    float det = ex.x * ey.y - ey.x * ex.y;
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }
    float2 x;
    x.x = det * (ey.y * b.x - ey.x * b.y);
    x.y = det * (ex.x * b.y - ex.y * b.x);
    return x;
}

inline float2 b2clMat22Solve(const b2clMat22 mat, const float2 b)
{
    float2 ex = (float2)(mat.ex[0], mat.ex[1]);
    float2 ey = (float2)(mat.ey[0], mat.ey[1]);

    float det = ex.x * ey.y - ey.x * ex.y;
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }
    float2 x;
    x.x = det * (ey.y * b.x - ey.x * b.y);
    x.y = det * (ex.x * b.y - ex.y * b.x);
    return x;
}

inline float2 b2clMat33Mul22(const b2clMat33 A, const float2 v)
{
    return (float2)(A.ex[0] * v.x + A.ey[0] * v.y, A.ex[1] * v.x + A.ey[1] * v.y);
}

inline float2 b2clMat22Mul(const b2clMat22 A, const float2 v)
{
    return (float2)(A.ex[0] * v.x + A.ey[0] * v.y, A.ex[1] * v.x + A.ey[1] * v.y);
}

inline void b2clGetInverse22(const b2clMat33 in, b2clMat33* out)
{
    float a = in.ex[0], b = in.ey[0], c = in.ex[1], d = in.ey[1];
    float det = a * d - b * c;
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    out->ex[0] =  det * d;  out->ey[0] = -det * b; out->ex[2] = 0.0f;
    out->ex[1] = -det * c;  out->ey[1] =  det * a; out->ey[2] = 0.0f;
    out->ez[0] = 0.0f; out->ez[1] = 0.0f; out->ez[2] = 0.0f;
}

inline void b2clGetSymInverse33(const b2clMat33 in, b2clMat33* out)
{
    //float det = b2clDot(ex, b2clCross(ey, ez));
    float det = in.ex[0] * (in.ey[1] * in.ez[2] - in.ey[2] * in.ez[1]) 
        + in.ex[1] * (in.ey[2] * in.ez[0] - in.ey[0] * in.ez[2]) 
        + in.ex[2] * (in.ey[0] * in.ez[1] - in.ey[1] * in.ez[0]);

    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    float a11 = in.ex[0], a12 = in.ey[0], a13 = in.ez[0];
    float a22 = in.ey[1], a23 = in.ez[1];
    float a33 = in.ez[2];

    out->ex[0] = det * (a22 * a33 - a23 * a23);
    out->ex[1] = det * (a13 * a23 - a12 * a33);
    out->ex[2] = det * (a12 * a23 - a13 * a22);

    out->ey[0] = out->ex[1];
    out->ey[1] = det * (a11 * a33 - a13 * a13);
    out->ey[2] = det * (a13 * a12 - a11 * a23);

    out->ez[0] = out->ex[2];
    out->ez[1] = out->ey[2];
    out->ez[2] = det * (a11 * a22 - a12 * a12);
}

inline float2 b2clMat33MulV2(const b2clMat33 A, const float2 v)
{
    return (float2)(A.ex[0] * v.x + A.ey[0] * v.y, A.ex[1] * v.x + A.ey[1] * v.y);
}

inline float3 b2clMat33MulV3(const b2clMat33 A, const float3 v)
{
    //return v.x * A.ex + v.y * A.ey + v.z * A.ez;
    return (float3)(A.ex[0] * v.x + A.ey[0] * v.y + A.ez[0] * v.z,
        A.ex[1] * v.x + A.ey[1] * v.y + A.ez[1] * v.z,
        A.ex[2] * v.x + A.ey[2] * v.y + A.ez[2] * v.z);
}

inline void b2clMat22GetInverse(const b2clMat22 in, b2clMat22* out)
{
    float a = in.ex[0], b = in.ey[0], c = in.ex[1], d = in.ey[1];
    float det = a * d - b * c;
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }
    out->ex[0] =  det * d;  out->ey[0] = -det * b;
    out->ex[1] = -det * c;  out->ey[1] =  det * a;
}

inline void b2clSweepNormalize(b2clSweep* sweep)
{
    float twoPi = 2.0f * b2_pi;
    float d =  twoPi * floor(sweep->a0 / twoPi);
    sweep->a0 -= d;
    sweep->a -= d;
}

inline void b2clSweepGetTransform(b2clSweep* sweep, b2clTransform* xf, float beta)
{
    xf->p = (1.0f - beta) * sweep->c0 + beta * sweep->c;
    float angle = (1.0f - beta) * sweep->a0 + beta * sweep->a;
    xf->q.x = sin(angle);
    xf->q.y = cos(angle);

    // Shift to origin
    xf->p -= b2clMul_Rotate(xf->q, sweep->localCenter);
}

inline void b2clSweepAdvance(b2clSweep* sweep, float alpha)
{
    float beta = (alpha - sweep->alpha0) / (1.0f - sweep->alpha0);
    sweep->c0 = (1.0f - beta) * sweep->c0 + beta * sweep->c;
    sweep->a0 = (1.0f - beta) * sweep->a0 + beta * sweep->a;
    sweep->alpha0 = alpha;
}
inline void b2clBodySweepAdvance (b2clBodyDynamic* body, b2clTransform* xf, float alpha) {
    b2clSweepAdvance(&(body->m_sweep), alpha); 
    body->m_sweep.c = body->m_sweep.c0;
    body->m_sweep.a = body->m_sweep.a0;
    xf->q.x = sin (body->m_sweep.a);
    xf->q.y = cos (body->m_sweep.a); 
    xf->p = body->m_sweep.c - b2clMul_Rotate(xf->q, body->m_sweep.localCenter);
}

inline void b2clDistanceProxySet(b2clDistanceProxy* proxy, const b2clPolygonShape* shape)
{
    proxy->m_count = shape->m_vertexCount;
    proxy->m_radius = shape->m_radius;
    for (int i = 0; i < shape->m_vertexCount; ++i)
    {
        proxy->m_vertices[i] = shape->m_vertices[i];
    }
}

inline int b2clDistanceProxyGetSupport(const b2clDistanceProxy* proxy, float2 d)
{
    int bestIndex = 0;
    float bestValue = b2clDot(proxy->m_vertices[0], d);
    for (int i = 1; i < proxy->m_count; ++i)
    {
        float value = b2clDot(proxy->m_vertices[i], d);
        if (value > bestValue)
        {
            bestIndex = i;
            bestValue = value;
        }
    }
    return bestIndex;
}

inline float2 b2clDistanceProxyGetVertex(const b2clDistanceProxy* proxy, int index)
{
    return proxy->m_vertices[index];
}

// Convert a vector into a unit vector. Returns the length.
inline float b2clNormalize(float2* v)
{
    float length = sqrt(b2clDot(*v, *v));
    if (length < b2_epsilon)
    {
        return 0.0f;
    }
    float invLength = 1.0f / length;
    (*v).x *= invLength;
    (*v).y *= invLength;

    return length;
}

/*
inline float cos_wrapper(float a)
{
    //float sina = sin(a);
    //return sqrt(1-sina*sina);

    //return sin(1.5707963268-a);

    return cos(a);
}
*/

//#include <Box2D/Common/OpenCL/b2CLTypeDefOCL.h>

__kernel void IntegratePosition(
                                __global clb2Velocity* velocities,
                                __global clb2Position* positions,
                                const float dt,
                                const int bodyCount
                                )
{
    int bodyIndex=get_global_id(0);
    
    if(bodyIndex>=bodyCount) return;
    
    clb2Position thisPosition=positions[bodyIndex];
    clb2Velocity thisVelocity=velocities[bodyIndex];
    
    float2 translation = (float2)(dt*thisVelocity.vx, dt*thisVelocity.vy);
    float translationLength=sqrt(dot(translation,translation));
    if(translationLength>b2_maxTranslation){
        thisVelocity.vx *= (b2_maxTranslation/translationLength);
		thisVelocity.vy *= (b2_maxTranslation/translationLength);
    }
    float rotation = dt*thisVelocity.w;
    float rotationMagnitude = fabs(rotation);
    if(rotationMagnitude>b2_maxRotation){
        thisVelocity.w *= (b2_maxRotation/rotationMagnitude);
    }
    
    thisPosition.cx += dt*thisVelocity.vx;
    thisPosition.cy += dt*thisVelocity.vy;
    thisPosition.a += dt*thisVelocity.w;
    
    positions[bodyIndex] = thisPosition;
    velocities[bodyIndex] = thisVelocity;
}

void computePositionSolverManifoldCircles(clb2Manifold* manifold,clb2PositionSolverManifold* positionSolverManifold,clb2Transform* xfA,clb2Transform* xfB,int index)
{
    float2 pointA=(float2)(manifold->localPoint.x*xfA->rotation.c - manifold->localPoint.y*xfA->rotation.s + xfA->translation.x, manifold->localPoint.x*xfA->rotation.s + manifold->localPoint.y*xfA->rotation.c + xfA->translation.y);
    float2 pointB=(float2)(manifold->localPoints1.x*xfB->rotation.c - manifold->localPoints1.y*xfB->rotation.s + xfB->translation.x, manifold->localPoints1.x*xfB->rotation.s + manifold->localPoints1.y*xfB->rotation.c + xfB->translation.y);
    
    float2 normal=pointB-pointA;
    positionSolverManifold->normal=normalize(normal);
    positionSolverManifold->point=0.5f*(pointA+pointB);
    positionSolverManifold->separation=length(normal) - manifold->radiusA - manifold->radiusB;
}

void computePositionSolverManifoldFaceA(clb2Manifold* manifold,clb2PositionSolverManifold* positionSolverManifold,clb2Transform* xfA,clb2Transform* xfB,int index)
{
    float2 planePoint=(float2)(manifold->localPoint.x*xfA->rotation.c - manifold->localPoint.y*xfA->rotation.s + xfA->translation.x, manifold->localPoint.x*xfA->rotation.s + manifold->localPoint.y*xfA->rotation.c + xfA->translation.y);
    float2 normal=(float2)(xfA->rotation.c*manifold->localNormal.x-xfA->rotation.s*manifold->localNormal.y,xfA->rotation.s*manifold->localNormal.x+xfA->rotation.c*manifold->localNormal.y);
    
    if(index==1){
        float2 clipPoint=(float2)(manifold->localPoints1.x*xfB->rotation.c - manifold->localPoints1.y*xfB->rotation.s + xfB->translation.x, manifold->localPoints1.x*xfB->rotation.s + manifold->localPoints1.y*xfB->rotation.c + xfB->translation.y);
        
        positionSolverManifold->normal=normal;
        positionSolverManifold->separation=dot(clipPoint-planePoint,normal)-manifold->radiusA-manifold->radiusB;
        positionSolverManifold->point=clipPoint;
    }else{
        float2 clipPoint=(float2)(manifold->localPoints2.x*xfB->rotation.c - manifold->localPoints2.y*xfB->rotation.s + xfB->translation.x, manifold->localPoints2.x*xfB->rotation.s + manifold->localPoints2.y*xfB->rotation.c + xfB->translation.y);
        
        positionSolverManifold->normal=normal;
        positionSolverManifold->separation=dot(clipPoint-planePoint,normal)-manifold->radiusA-manifold->radiusB;
        positionSolverManifold->point=clipPoint;
    }
}

void computePositionSolverManifoldFaceB(clb2Manifold* manifold,clb2PositionSolverManifold* positionSolverManifold,clb2Transform* xfA,clb2Transform* xfB,int index)
{
    float2 planePoint=(float2)(manifold->localPoint.x*xfB->rotation.c - manifold->localPoint.y*xfB->rotation.s + xfB->translation.x, manifold->localPoint.x*xfB->rotation.s + manifold->localPoint.y*xfB->rotation.c + xfB->translation.y);
    float2 normal=(float2)(xfB->rotation.c*manifold->localNormal.x-xfB->rotation.s*manifold->localNormal.y,xfB->rotation.s*manifold->localNormal.x+xfB->rotation.c*manifold->localNormal.y);
    
    if(index==1){
        float2 clipPoint=(float2)(manifold->localPoints1.x*xfA->rotation.c - manifold->localPoints1.y*xfA->rotation.s + xfA->translation.x, manifold->localPoints1.x*xfA->rotation.s + manifold->localPoints1.y*xfA->rotation.c + xfA->translation.y);
        
        positionSolverManifold->normal=-normal;
        positionSolverManifold->separation=dot(clipPoint-planePoint,normal)-manifold->radiusA-manifold->radiusB;
        positionSolverManifold->point=clipPoint;
    }else{
        float2 clipPoint=(float2)(manifold->localPoints2.x*xfA->rotation.c - manifold->localPoints2.y*xfA->rotation.s + xfA->translation.x, manifold->localPoints2.x*xfA->rotation.s + manifold->localPoints2.y*xfA->rotation.c + xfA->translation.y);
        
        positionSolverManifold->normal=-normal;
        positionSolverManifold->separation=dot(clipPoint-planePoint,normal)-manifold->radiusA-manifold->radiusB;
        positionSolverManifold->point=clipPoint;
    }
}

__kernel void SolvePositionConstraint(
                                      __global clb2Position* positions,
                                      __global clb2Manifold* manifolds,
                                      __global clb2Contact* contacts,
                                      const unsigned int offset,
                                      const unsigned int contactCount
                                      )
{
    unsigned int contactIndex=get_global_id(0)+offset;
    if(contactIndex>=offset+contactCount) return;
    clb2Contact thisContact=contacts[contactIndex];
    clb2Manifold thisManifold=manifolds[contactIndex];
    int indexA=thisContact.indexA;
    int indexB=thisContact.indexB;
    clb2Position thisPosA=positions[indexA];
    clb2Position thisPosB=positions[indexB];
    
    clb2PositionSolverManifold thisPositionManifold;
    
    clb2Transform xfA,xfB;
    
    float sina, cosa;
    sina = sincos(thisPosA.a, &cosa);
    xfA.rotation.s = sina;
    xfA.rotation.c = cosa;
    //xfA.rotation.s = native_sin(thisPosA.a);
    //xfA.rotation.c = native_cos(thisPosA.a);
    //xfA.rotation.c = cos_wrapper(thisPosA.a);
    
    sina = sincos(thisPosB.a, &cosa);
    xfB.rotation.s = sina;
    xfB.rotation.c = cosa;
    //xfB.rotation.s = native_sin(thisPosB.a);
    //xfB.rotation.c = native_cos(thisPosB.a);
    //xfB.rotation.c = cos_wrapper(thisPosB.a);
    
    xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
    xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y);  
    
    if(thisManifold.type==0)
        computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==1)
        computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==2)
        computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    
    float2 dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
    float2 dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
    
    float C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
    
    float rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
    float rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
    
    float K=thisContact.invMassA + thisContact.invMassB + thisContact.invIA*rnA*rnA + thisContact.invIB*rnB*rnB;
    
    float2 P=max(-C/K,0.0f)*thisPositionManifold.normal;
 
    
    thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
    thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
    thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 
    
    thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
    thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
    thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);
    
    if(thisManifold.pointCount==2){
        
        sina = sincos(thisPosA.a, &cosa);
        xfA.rotation.s = sina;
        xfA.rotation.c = cosa;
    	// xfA.rotation.s = native_sin(thisPosA.a);
    	// xfA.rotation.c = native_cos(thisPosA.a);
    	//xfA.rotation.c = cos_wrapper(thisPosA.a);
    	 
        sina = sincos(thisPosB.a, &cosa);
        xfB.rotation.s = sina;
        xfB.rotation.c = cosa;
    	// xfB.rotation.s = native_sin(thisPosB.a);
    	// xfB.rotation.c = native_cos(thisPosB.a);
		//xfB.rotation.c = cos_wrapper(thisPosB.a);
		 
        xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
        xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y); 
        
        if(thisManifold.type==0)
            computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==1)
            computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==2)
            computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        
        dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
        dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
        
        C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
        
        rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
        rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
        
        K=thisContact.invMassA + thisContact.invMassB + thisContact.invIA*rnA*rnA + thisContact.invIB*rnB*rnB;
        
        P=max(-C/K,0.0f)*thisPositionManifold.normal;
        
		// float absy = P.y ; if (absy < 0) absy = 0 - absy ; 
		//if (absy > 0.1) P.x *= lv ; P.y *= lv ; 

        thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
        thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
        thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 
        
        thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
        thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
        thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);
    }
    
    positions[indexA]=thisPosA;
    positions[indexB]=thisPosB;
}


__kernel void SolveSplitPositionConstraint(
                                      __global clb2Position* positions,
                                      __global clb2Manifold* manifolds,
                                      __global clb2Contact* contacts,
									  __global clb2Velocity* splitVelocities,
									  __global unsigned int* numContacts4EachBody, 
									  __global unsigned int* indexContact2BodySplitVelocity, 
                                      const unsigned int numContacts
                          
                                      )
{
    unsigned int contactIndex=get_global_id(0); 
    if(contactIndex>= numContacts) return;

	unsigned int indexContact2BodyA = indexContact2BodySplitVelocity[contactIndex*2+0];
	unsigned int indexContact2BodyB = indexContact2BodySplitVelocity[contactIndex*2+1]; 
	int maxContactNumPerBody = 20 ; 
    clb2Contact thisContact=contacts[contactIndex];
    clb2Manifold thisManifold=manifolds[contactIndex];
    int indexA=thisContact.indexA;
    int indexB=thisContact.indexB;
    clb2Position thisPosA=positions[indexA];
    clb2Position thisPosB=positions[indexB];
	int numBodyA = numContacts4EachBody [thisContact.indexA];
	int numBodyB = numContacts4EachBody [thisContact.indexB];

	clb2Velocity deltaPosA ;deltaPosA.vx = deltaPosA.vy = deltaPosA.w = 0 ;  
	clb2Velocity deltaPosB ; deltaPosB.vx = deltaPosB.vy = deltaPosB.w = 0 ; 
    
    clb2PositionSolverManifold thisPositionManifold;
    
    clb2Transform xfA,xfB;
    float sina, cosa;
    
    sina = sincos(thisPosA.a, &cosa);
    xfA.rotation.s = sina;
    xfA.rotation.c = cosa;
    //xfA.rotation.s = native_sin(thisPosA.a);
    //xfA.rotation.c = native_cos(thisPosA.a);
    //xfA.rotation.c = cos_wrapper(thisPosA.a);
    
    sina = sincos(thisPosB.a, &cosa);
    xfB.rotation.s = sina;
    xfB.rotation.c = cosa;
    //xfB.rotation.s = native_sin(thisPosB.a);
    //xfB.rotation.c = native_cos(thisPosB.a);
    //xfB.rotation.c = cos_wrapper(thisPosB.a);
    
    xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
    xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y);  
    
    if(thisManifold.type==0)
        computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==1)
        computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==2)
        computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    
    float2 dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
    float2 dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
    
    float C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
    
    float rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
    float rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
    
    float K=thisContact.invMassA*numBodyA + thisContact.invMassB *numBodyB + thisContact.invIA*rnA*rnA*numBodyA + thisContact.invIB*rnB*rnB *numBodyB;
    
    float2 P=max(-C/K,0.0f)*thisPositionManifold.normal;

	//float lv = 1.0 ;
    //float absy = P.y ; if (absy < 0) absy = 0 - absy ; 
	//if (P.y > 0 && absy > 0.1) P.x *= lv ; P.y *= lv ;  
    
    thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
    thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
    thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 

	deltaPosA.vx -= thisContact.invMassA*P.x;
	deltaPosA.vy -= thisContact.invMassA*P.y;
	deltaPosA.w  -= thisContact.invIA * (dA.x * P.y - dA.y * P.x);

    
    thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
    thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
    thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);

	deltaPosB.vx += thisContact.invMassB * P.x; 
	deltaPosB.vy += thisContact.invMassB * P.y;
	deltaPosB.w += thisContact.invIB * (dB.x * P.y - dB.y * P.x);
    
    if(thisManifold.pointCount==2){
        
		sina = sincos(thisPosA.a, &cosa);
    	xfA.rotation.s = sina;
    	xfA.rotation.c = cosa;
        //xfA.rotation.s = native_sin(thisPosA.a);
        //xfA.rotation.c = native_cos(thisPosA.a);
        //xfA.rotation.c = cos_wrapper(thisPosA.a);
        
		sina = sincos(thisPosB.a, &cosa);
    	xfB.rotation.s = sina;
    	xfB.rotation.c = cosa;
        //xfB.rotation.s = native_sin(thisPosB.a);
        //xfB.rotation.c = native_cos(thisPosB.a);
        //xfB.rotation.c = cos_wrapper(thisPosB.a);
        
        xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
        xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y); 
        
        if(thisManifold.type==0)
            computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==1)
            computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==2)
            computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        
        dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
        dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
        
        C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
        
        rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
        rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
        
        K=thisContact.invMassA*numBodyA + thisContact.invMassB*numBodyB + thisContact.invIA*rnA*rnA*numBodyA + thisContact.invIB*rnB*rnB*numBodyB;
        
        P=max(-C/K,0.0f)*thisPositionManifold.normal;
        
		// float absy = P.y ; if (absy < 0) absy = 0 - absy ; 
		//if (absy > 0.1) P.x *= lv ; P.y *= lv ; 

        thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
        thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
        thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 


		deltaPosA.vx -= thisContact.invMassA*P.x;
		deltaPosA.vy -= thisContact.invMassA*P.y;
		deltaPosA.w  -= thisContact.invIA * (dA.x * P.y - dA.y * P.x);

        
        thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
        thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
        thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);

		deltaPosB.vx += thisContact.invMassB * P.x; 
		deltaPosB.vy += thisContact.invMassB * P.y;
		deltaPosB.w += thisContact.invIB * (dB.x * P.y - dB.y * P.x);
    }
    
	if (numBodyA != 0) splitVelocities[thisContact.indexA * maxContactNumPerBody + indexContact2BodyA] = deltaPosA; 
	if (numBodyB != 0) splitVelocities [thisContact.indexB * maxContactNumPerBody + indexContact2BodyB] = deltaPosB ; 

    //positions[indexA]=thisPosA; 
    //positions[indexB]=thisPosB;
}



__kernel void SolvePositionConstraint_MergeSplittedMass(
									  __global clb2Position* positions, 
                                      __global clb2Velocity* splitVelocities,
                                      __global clb2Contact* contacts,
								 __global unsigned int* numContacts4EachBody,
								 __global unsigned int* indexContact2BodySplitVelocity,
								 const uint bodyCount
                                      )
{
    unsigned int bodyIndex = get_global_id(0);
	if (bodyIndex >= bodyCount) return ; 
	clb2Position thisBodyPosition = positions[bodyIndex];
	unsigned int numContacts4Body = numContacts4EachBody[bodyIndex]; 
	clb2Velocity  deltaMergedVelocity; deltaMergedVelocity.vx = deltaMergedVelocity.vy = deltaMergedVelocity.w = 0 ; 
	//__global clb2Velocity* pDeltaSplitVelocity  ; 
	clb2Velocity  thisSplitVelocity ; 

	int maxContactNumPerBody = 20 ; 
	for ( int i = 0 ; i < numContacts4Body; i ++ ) {
		thisSplitVelocity = splitVelocities[bodyIndex*maxContactNumPerBody+i] ; 
		deltaMergedVelocity.vx += thisSplitVelocity.vx ; deltaMergedVelocity.vy += thisSplitVelocity.vy ; deltaMergedVelocity.w += thisSplitVelocity.w ;
	} 

	thisBodyPosition.cx += deltaMergedVelocity.vx ; 
	thisBodyPosition.cy += deltaMergedVelocity.vy ;
	thisBodyPosition.a += deltaMergedVelocity.w ;
	positions[bodyIndex] = thisBodyPosition; 
}

__kernel void SolveDistanceJointPositionConstraint(
									__global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveRevoluteJointPositionConstraint
//
// Compute positions of bodies which are constrained by a Revolute joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveRevoluteJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;

    if(jointIndex >= offset + colorLength) 
		return;

	b2clJoint thisJoint = jointList[jointIndex]; 

	clb2Position positionA = positions[thisJoint.indexA];
	clb2Position positionB = positions[thisJoint.indexB];

	float2 cA = (float2)(positionA.cx, positionA.cy);
	float aA = positionA.a;
	float2 cB = (float2)(positionB.cx, positionB.cy);
	float aB = positionB.a;

	float sina, cosa;
	
	sina = sincos(aA, &cosa);
	float2 qA = (float2)(sina,  cosa);
	sina = sincos(aB, &cosa);
	float2 qB = (float2)(sina,  cosa);
	//float2 qA = (float2)(sin(aA),  cos(aA));
	//float2 qB = (float2)(sin(aB), cos(aB));
	//float2 qA = (float2)(sin(aA),  cos_wrapper(aA));
	//float2 qB = (float2)(sin(aB), cos_wrapper(aB));

	float angularError = 0.0f;
	float positionError = 0.0f;

	bool fixedRotation = (thisJoint.b.revoluteJointData.invIA + thisJoint.b.revoluteJointData.invIB == 0.0f);

	// Solve angular limit constraint.
	if (thisJoint.b.revoluteJointData.enableLimit && thisJoint.b.revoluteJointData.limitState != e_inactiveLimit && fixedRotation == false)
	{
		float angle = aB - aA - thisJoint.b.revoluteJointData.referenceAngle;
		float limitImpulse = 0.0f;

		if (thisJoint.b.revoluteJointData.limitState == e_equalLimits)
		{
			// Prevent large angular corrections
			float C = b2clClamp(angle - thisJoint.b.revoluteJointData.lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -thisJoint.b.revoluteJointData.motorMass * C;
			angularError = b2clAbs(C);
		}
		else if (thisJoint.b.revoluteJointData.limitState == e_atLowerLimit)
		{
			float C = angle - thisJoint.b.revoluteJointData.lowerAngle;
			angularError = -C;

			// Prevent large angular corrections and allow some slop.
			C = b2clClamp(C + b2_angularSlop, -b2_maxAngularCorrection, 0.0f);
			limitImpulse = -thisJoint.b.revoluteJointData.motorMass * C;
		}
		else if (thisJoint.b.revoluteJointData.limitState == e_atUpperLimit)
		{
			float C = angle - thisJoint.b.revoluteJointData.upperAngle;
			angularError = C;

			// Prevent large angular corrections and allow some slop.
			C = b2clClamp(C - b2_angularSlop, 0.0f, b2_maxAngularCorrection);
			limitImpulse = -thisJoint.b.revoluteJointData.motorMass * C;
		}

		aA -= thisJoint.b.revoluteJointData.invIA * limitImpulse;
		aB += thisJoint.b.revoluteJointData.invIB * limitImpulse;
	}

	// Solve point-to-point constraint.
	{
		sina = sincos(aA, &cosa);
		qA = (float2)(sina,  cosa);
		sina = sincos(aB, &cosa);
		qB = (float2)(sina,  cosa);
		//qA = (float2)(sin(aA), cos(aA));
		//qB = (float2)(sin(aB), cos(aB));
		//qA = (float2)(sin(aA), cos_wrapper(aA));
		//qB = (float2)(sin(aB), cos_wrapper(aB));
		float2 rA = b2clMul_Rotate(qA, (float2)(thisJoint.b.revoluteJointData.localAnchorA[0] - thisJoint.b.revoluteJointData.localCenterA[0], thisJoint.b.revoluteJointData.localAnchorA[1] - thisJoint.b.revoluteJointData.localCenterA[1]));
		float2 rB = b2clMul_Rotate(qB, (float2)(thisJoint.b.revoluteJointData.localAnchorB[0] - thisJoint.b.revoluteJointData.localCenterB[0], thisJoint.b.revoluteJointData.localAnchorB[1] - thisJoint.b.revoluteJointData.localCenterB[1]));

		float2 C = cB + rB - cA - rA;
		float length = sqrt(b2clDot(C, C)); 
		positionError = length;

		float mA = thisJoint.b.revoluteJointData.invMassA, mB = thisJoint.b.revoluteJointData.invMassB;
		float iA = thisJoint.b.revoluteJointData.invIA, iB = thisJoint.b.revoluteJointData.invIB;

		b2clMat22 K;
		K.ex[0] = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
		K.ex[1] = -iA * rA.x * rA.y - iB * rB.x * rB.y;
		K.ey[0] = K.ex[1];
		K.ey[1] = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

		float2 impulse = -b2clMat22Solve(K, C);

		cA -= mA * impulse;
		aA -= iA * b2clCross_VV(rA, impulse);

		cB += mB * impulse;
		aB += iB * b2clCross_VV(rB, impulse);
	}

	positionA.cx = cA.x;
	positionA.cy = cA.y;
	positionA.a = aA;
	positionB.cx = cB.x;
	positionB.cy = cB.y;
	positionB.a = aB;
	positions[thisJoint.indexA] = positionA;
	positions[thisJoint.indexB] = positionB;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolvePrismaticJointPositionConstraint
//
// Compute positions of bodies which are constrained by a Prismatic joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolvePrismaticJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;

    if(jointIndex >= offset + colorLength) 
		return;

	b2clJoint thisJoint = jointList[jointIndex]; 

	clb2Position positionA = positions[thisJoint.indexA];
	clb2Position positionB = positions[thisJoint.indexB];

	float2 m_localAnchorA = (float2) (thisJoint.b.prismaticJointData.localAnchorA[0] , thisJoint.b.prismaticJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.prismaticJointData.localAnchorB[0] , thisJoint.b.prismaticJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (thisJoint.b.prismaticJointData.localCenterA[0] , thisJoint.b.prismaticJointData.localCenterA[1]);
	float2 m_localCenterB = (float2) (thisJoint.b.prismaticJointData.localCenterB[0] , thisJoint.b.prismaticJointData.localCenterB[1]);
	float2 m_localXAxisA = (float2) (thisJoint.b.prismaticJointData.localXAxisA[0] , thisJoint.b.prismaticJointData.localXAxisA[1]);
	float2 m_localYAxisA = (float2) (thisJoint.b.prismaticJointData.localYAxisA[0] , thisJoint.b.prismaticJointData.localYAxisA[1]);

	float2 cA = (float2)(positionA.cx, positionA.cy);
	float aA = positionA.a;
	float2 cB = (float2)(positionB.cx, positionB.cy);
	float aB = positionB.a;

	float sina, cosa;
	sina = sincos(aA, &cosa);
	float2 qA = (float2)(sina,  cosa);
	sina = sincos(aB, &cosa);
	float2 qB = (float2)(sina,  cosa);
	//float2 qA = (float2)(sin(aA), cos(aA));
	//float2 qB = (float2)(sin(aB), cos(aB));
	//float2 qA = (float2)(sin(aA), cos_wrapper(aA));
	//float2 qB = (float2)(sin(aB), cos_wrapper(aB));

	float mA = thisJoint.b.prismaticJointData.invMassA, mB = thisJoint.b.prismaticJointData.invMassB;
	float iA = thisJoint.b.prismaticJointData.invIA, iB = thisJoint.b.prismaticJointData.invIB;

	// Compute fresh Jacobians
	float2 rA = b2clMul_Rotate(qA, m_localAnchorA - m_localCenterA);
	float2 rB = b2clMul_Rotate(qB, m_localAnchorB - m_localCenterB);
	float2 d = cB + rB - cA - rA;

	float2 axis = b2clMul_Rotate(qA, m_localXAxisA);
	float a1 = b2clCross_VV(d + rA, axis);
	float a2 = b2clCross_VV(rB, axis);
	float2 perp = b2clMul_Rotate(qA, m_localYAxisA);

	float s1 = b2clCross_VV(d + rA, perp);
	float s2 = b2clCross_VV(rB, perp);

	float3 impulse;
	float2 C1;
	C1.x = b2clDot(perp, d);
	C1.y = aB - aA - thisJoint.b.prismaticJointData.referenceAngle;

	float linearError = b2clAbs(C1.x);
	float angularError = b2clAbs(C1.y);

	bool active = false;
	float C2 = 0.0f;
	if (thisJoint.b.prismaticJointData.enableLimit)
	{
		float translation = b2clDot(axis, d);
		if (b2clAbs(thisJoint.b.prismaticJointData.upperTranslation - thisJoint.b.prismaticJointData.lowerTranslation) < 2.0f * b2_linearSlop)
		{
			// Prevent large angular corrections
			C2 = b2clClamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
			linearError = max(linearError, b2clAbs(translation));
			active = true;
		}
		else if (translation <= thisJoint.b.prismaticJointData.lowerTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C2 = b2clClamp(translation - thisJoint.b.prismaticJointData.lowerTranslation + b2_linearSlop, -b2_maxLinearCorrection, 0.0f);
			linearError = max(linearError, thisJoint.b.prismaticJointData.lowerTranslation - translation);
			active = true;
		}
		else if (translation >= thisJoint.b.prismaticJointData.upperTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C2 = b2clClamp(translation - thisJoint.b.prismaticJointData.upperTranslation - b2_linearSlop, 0.0f, b2_maxLinearCorrection);
			linearError = max(linearError, translation - thisJoint.b.prismaticJointData.upperTranslation);
			active = true;
		}
	}

	if (active)
	{
		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k13 = iA * s1 * a1 + iB * s2 * a2;
		float k22 = iA + iB;
		if (k22 == 0.0f)
		{
			// For fixed rotation
			k22 = 1.0f;
		}
		float k23 = iA * a1 + iB * a2;
		float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

		b2clMat33 K;
		K.ex[0] = k11;
		K.ex[1] = k12;
		K.ex[2] = k13;
		K.ey[0] = k12;
		K.ey[1] = k22;
		K.ey[2] = k23;
		K.ez[0] = k13;
		K.ez[1] = k23;
		K.ez[2] = k33;

		float3 C = (float3)(C1.x, C1.y, C2);

		impulse = b2clMat33Solve(K, -C);
	}
	else
	{
		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k22 = iA + iB;
		if (k22 == 0.0f)
		{
			k22 = 1.0f;
		}

		b2clMat22 K;
		K.ex[0] = k11;
		K.ex[1] = k12;
		K.ey[0] = k12;
		K.ey[1] = k22;

		float2 impulse1 = b2clMat22Solve(K, -C1);
		impulse.x = impulse1.x;
		impulse.y = impulse1.y;
		impulse.z = 0.0f;
	}

	float2 P = impulse.x * perp + impulse.z * axis;
	float LA = impulse.x * s1 + impulse.y + impulse.z * a1;
	float LB = impulse.x * s2 + impulse.y + impulse.z * a2;

	cA -= mA * P;
	aA -= iA * LA;
	cB += mB * P;
	aB += iB * LB;

	positionA.cx = cA.x;
	positionA.cy = cA.y;
	positionA.a = aA;
	positionB.cx = cB.x;
	positionB.cy = cB.y;
	positionB.a = aB;
	positions[thisJoint.indexA] = positionA;
	positions[thisJoint.indexB] = positionB;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveGearJointPositionConstraint
//
// Compute positions of bodies which are constrained by a Gear joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveGearJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;

    if(jointIndex >= offset + colorLength) 
		return;

	b2clJoint thisJoint = jointList[jointIndex]; 

	clb2Position positionA = positions[thisJoint.indexA];
	clb2Position positionB = positions[thisJoint.indexB];
	clb2Position positionC = positions[thisJoint.indexC];
	clb2Position positionD = positions[thisJoint.indexD];

	float2 cA = (float2)(positionA.cx, positionA.cy);
	float aA = positionA.a;
	float2 cB = (float2)(positionB.cx, positionB.cy);
	float aB = positionB.a;
	float2 cC = (float2)(positionC.cx, positionC.cy);
	float aC = positionC.a;
	float2 cD = (float2)(positionD.cx, positionD.cy);
	float aD = positionD.a;

    float sina, cosa;
	sina = sincos(aA, &cosa);
	float2 qA = (float2)(sina,  cosa);
	sina = sincos(aB, &cosa);
	float2 qB = (float2)(sina,  cosa);
	sina = sincos(aC, &cosa);
	float2 qC = (float2)(sina,  cosa);
	sina = sincos(aD, &cosa);
	float2 qD = (float2)(sina,  cosa);

	//float2 qA = (float2)(sin(aA), cos(aA));
	//float2 qB = (float2)(sin(aB), cos(aB));
	//float2 qC = (float2)(sin(aC), cos(aC));
	//float2 qD = (float2)(sin(aD), cos(aD));

	//float2 qA = (float2)(sin(aA), cos_wrapper(aA));
	//float2 qB = (float2)(sin(aB), cos_wrapper(aB));
	//float2 qC = (float2)(sin(aC), cos_wrapper(aC));
	//float2 qD = (float2)(sin(aD), cos_wrapper(aD));

	float2 m_localAnchorA = (float2) (thisJoint.b.gearJointData.localAnchorA[0] , thisJoint.b.gearJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.gearJointData.localAnchorB[0] , thisJoint.b.gearJointData.localAnchorB[1]);
	float2 m_localAnchorC = (float2) (thisJoint.b.gearJointData.localAnchorC[0] , thisJoint.b.gearJointData.localAnchorC[1]);
	float2 m_localAnchorD = (float2) (thisJoint.b.gearJointData.localAnchorD[0] , thisJoint.b.gearJointData.localAnchorD[1]);
	float2 m_localAxisC = (float2) (thisJoint.b.gearJointData.localAxisC[0] , thisJoint.b.gearJointData.localAxisC[1]);
	float2 m_localAxisD = (float2) (thisJoint.b.gearJointData.localAxisD[0] , thisJoint.b.gearJointData.localAxisD[1]);
	float2 m_lcA = (float2) (thisJoint.b.gearJointData.lcA[0] , thisJoint.b.gearJointData.lcA[1]);
	float2 m_lcB = (float2) (thisJoint.b.gearJointData.lcB[0] , thisJoint.b.gearJointData.lcB[1]);
	float2 m_lcC = (float2) (thisJoint.b.gearJointData.lcC[0] , thisJoint.b.gearJointData.lcC[1]);
	float2 m_lcD = (float2) (thisJoint.b.gearJointData.lcD[0] , thisJoint.b.gearJointData.lcD[1]);

	float linearError = 0.0f;

	float coordinateA, coordinateB;

	float2 JvAC, JvBD;
	float JwA, JwB, JwC, JwD;
	float mass = 0.0f;
	
	if (thisJoint.b.gearJointData.typeA == e_revoluteJoint)
	{
		JvAC = (float2)(0.0f, 0.0f);
		JwA = 1.0f;
		JwC = 1.0f;
		mass += thisJoint.b.gearJointData.iA + thisJoint.b.gearJointData.iC;

		coordinateA = aA - aC - thisJoint.b.gearJointData.referenceAngleA;
	}
	else
	{
		float2 u = b2clMul_Rotate(qC, m_localAxisC);
		float2 rC = b2clMul_Rotate(qC, m_localAnchorC - m_lcC);
		float2 rA = b2clMul_Rotate(qA, m_localAnchorA - m_lcA);
		JvAC = u;
		JwC = b2clCross_VV(rC, u);
		JwA = b2clCross_VV(rA, u);
		mass += thisJoint.b.gearJointData.mC + thisJoint.b.gearJointData.mA + thisJoint.b.gearJointData.iC * JwC * JwC + thisJoint.b.gearJointData.iA * JwA * JwA;

		float2 pC = m_localAnchorC - m_lcC;
		float2 pA = b2clMulT_Rotate(qC, rA + (cA - cC));
		coordinateA = b2clDot(pA - pC, m_localAxisC);
	}

	if (thisJoint.b.gearJointData.typeB == e_revoluteJoint)
	{
		JvBD = (float2)(0.0f, 0.0f);
		JwB = thisJoint.b.gearJointData.ratio;
		JwD = thisJoint.b.gearJointData.ratio;
		mass += thisJoint.b.gearJointData.ratio * thisJoint.b.gearJointData.ratio * (thisJoint.b.gearJointData.iB + thisJoint.b.gearJointData.iD);

		coordinateB = aB - aD - thisJoint.b.gearJointData.referenceAngleB;
	}
	else
	{
		float2 u = b2clMul_Rotate(qD, m_localAxisD);
		float2 rD = b2clMul_Rotate(qD, m_localAnchorD - m_lcD);
		float2 rB = b2clMul_Rotate(qB, m_localAnchorB - m_lcB);
		JvBD = thisJoint.b.gearJointData.ratio * u;
		JwD = thisJoint.b.gearJointData.ratio * b2clCross_VV(rD, u);
		JwB = thisJoint.b.gearJointData.ratio * b2clCross_VV(rB, u);
		mass += thisJoint.b.gearJointData.ratio * thisJoint.b.gearJointData.ratio * (thisJoint.b.gearJointData.mD + thisJoint.b.gearJointData.mB) + thisJoint.b.gearJointData.iD * JwD * JwD + thisJoint.b.gearJointData.iB * JwB * JwB;

		float2 pD = m_localAnchorD - m_lcD;
		float2 pB = b2clMulT_Rotate(qD, rB + (cB - cD));
		coordinateB = b2clDot(pB - pD, m_localAxisD);
	}
	
	float C = (coordinateA + thisJoint.b.gearJointData.ratio * coordinateB) - thisJoint.b.gearJointData.gearConstant;

	float impulse = 0.0f;
	if (mass > 0.0f)
	{
		impulse = -C / mass;
	}

	cA += thisJoint.b.gearJointData.mA * impulse * JvAC;
	aA += thisJoint.b.gearJointData.iA * impulse * JwA;
	cB += thisJoint.b.gearJointData.mB * impulse * JvBD;
	aB += thisJoint.b.gearJointData.iB * impulse * JwB;
	cC -= thisJoint.b.gearJointData.mC * impulse * JvAC;
	aC -= thisJoint.b.gearJointData.iC * impulse * JwC;
	cD -= thisJoint.b.gearJointData.mD * impulse * JvBD;
	aD -= thisJoint.b.gearJointData.iD * impulse * JwD;

	positionA.cx = cA.x;
	positionA.cy = cA.y;
	positionA.a = aA;
	positionB.cx = cB.x;
	positionB.cy = cB.y;
	positionB.a = aB;
	positionC.cx = cC.x;
	positionC.cy = cC.y;
	positionC.a = aC;
	positionD.cx = cD.x;
	positionD.cy = cD.y;
	positionD.a = aD;

	positions[thisJoint.indexA] = positionA;
	positions[thisJoint.indexB] = positionB;
	positions[thisJoint.indexC] = positionC;
	positions[thisJoint.indexD] = positionD;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolvePulleyJointPositionConstraint
//
// Compute positions of bodies which are constrained by a Pulley joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolvePulleyJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;
    if(jointIndex >= offset + colorLength) 
		return;
	b2clJoint thisJoint = jointList[jointIndex]; 
	clb2Position positionA = positions[thisJoint.indexA];
	clb2Position positionB = positions[thisJoint.indexB];

	
	float2 cA = (float2)(positionA.cx, positionA.cy);	float aA = positionA.a;
	float2 cB = (float2)(positionB.cx, positionB.cy);	float aB = positionB.a;

	float sina, cosa;
	sina = sincos(aA, &cosa);
	float2 qA = (float2)(sina,  cosa);
	sina = sincos(aB, &cosa);
	float2 qB = (float2)(sina,  cosa);
	//float2 qA = (float2)(sin(aA), cos(aA));
	//float2 qB = (float2)(sin(aB), cos(aB));
	//float2 qA = (float2)(sin(aA), cos_wrapper(aA));
	//float2 qB = (float2)(sin(aB), cos_wrapper(aB));

	float2 m_localAnchorA = (float2) (thisJoint.b.pulleyJointData.localAnchorA[0] , thisJoint.b.pulleyJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.pulleyJointData.localAnchorB[0] , thisJoint.b.pulleyJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (thisJoint.b.pulleyJointData.localCenterA[0] , thisJoint.b.pulleyJointData.localCenterA[1]);
	float2 m_localCenterB = (float2) (thisJoint.b.pulleyJointData.localCenterB[0] , thisJoint.b.pulleyJointData.localCenterB[1]);
	float2 m_groundAnchorA = (float2) (thisJoint.b.pulleyJointData.groundAnchorA[0], thisJoint.b.pulleyJointData.groundAnchorA[1]);
	float2 m_groundAnchorB = (float2) (thisJoint.b.pulleyJointData.groundAnchorB[0], thisJoint.b.pulleyJointData.groundAnchorB[1]);
	
	float2 rA = b2clMul_Rotate (qA, m_localAnchorA - m_localCenterA); 
	float2 rB = b2clMul_Rotate (qB, m_localAnchorB - m_localCenterB);
	
	float2 uA = cA + rA - m_groundAnchorA ; 
	float2 uB = cB + rB - m_groundAnchorB ; 
		 
	
	float lengthA = uA.x * uA.x + uA.y * uA.y ; lengthA = sqrt (lengthA);
	float lengthB = uB.x * uB.x + uB.y * uB.y ; lengthB = sqrt (lengthB);

	
	if (lengthA > 10.0f * b2_linearSlop){
		uA *= 1.0f /lengthA ; 
	} 
	else {
		uA.x = uA.y = 0 ; 
	}
	if (lengthB > 10.0f * b2_linearSlop){
		uB *= 1.0f / lengthB ; 
	}
	else {
		uB.x = uB.y = 0 ; 
	}
	float ruA = b2clCross_VV (rA, uA);
	float ruB = b2clCross_VV (rB, uB);
	float mA = thisJoint.b.pulleyJointData.invMassA + thisJoint.b.pulleyJointData.invIA * ruA * ruA ; 
	float mB = thisJoint.b.pulleyJointData.invMassB + thisJoint.b.pulleyJointData.invIB * ruB * ruB ; 

	float mass = mA + thisJoint.b.pulleyJointData.ratio * thisJoint.b.pulleyJointData.ratio * mB ; 
	if (mass > 0.0f){
		mass = 1.0f /mass ; 
	}
	float C = thisJoint.b.pulleyJointData.pulleyConstant - lengthA - thisJoint.b.pulleyJointData.ratio * lengthB ; 
	float linearError = b2clAbs(C);
	float impulse = -mass * C ; 
		
	//float2 PA = (float2)(-impulse * uA.x , -impulse * uA.y ) ; 
	//float2 PB = (float2)(-thisJoint.b.pulleyJointData.ratio * impulse * uB.x , -thisJoint.b.pulleyJointData.ratio * impulse * uB.y) ;

	float2 PA =  uA * (0-impulse) ; 
	float2 PB =  uB * (0 - impulse* thisJoint.b.pulleyJointData.ratio); 

	cA +=  PA * thisJoint.b.pulleyJointData.invMassA ; 
	aA += thisJoint.b.pulleyJointData.invIA * b2clCross_VV (rA, PA);
	cB += thisJoint.b.pulleyJointData.invMassB * PB ; 
	aB += thisJoint.b.pulleyJointData.invIB * b2clCross_VV (rB, PB);
	
	positionA.cx = cA.x;	positionA.cy = cA.y; positionA.a = aA;
	positionB.cx = cB.x;	positionB.cy = cB.y; positionB.a = aB;
	positions[thisJoint.indexA] = positionA;	positions[thisJoint.indexB] = positionB;
	 
	 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveRopeJointPositionConstraint
//
// Compute positions of bodies which are constrained by a Rope joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveRopeJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;
    if(jointIndex >= offset + colorLength) 
		return;
	b2clJoint thisJoint = jointList[jointIndex]; 
	clb2Position positionA = positions[thisJoint.indexA];
	clb2Position positionB = positions[thisJoint.indexB];

	
	float2 cA = (float2)(positionA.cx, positionA.cy);	float aA = positionA.a;
	float2 cB = (float2)(positionB.cx, positionB.cy);	float aB = positionB.a;

	float sina, cosa;
	sina = sincos(aA, &cosa);
	float2 qA = (float2)(sina,  cosa);
	sina = sincos(aB, &cosa);
	float2 qB = (float2)(sina,  cosa);
	//float2 qA = (float2)(sin(aA), cos(aA));
	//float2 qB = (float2)(sin(aB), cos(aB));
	//float2 qA = (float2)(sin(aA), cos_wrapper(aA));
	//float2 qB = (float2)(sin(aB), cos_wrapper(aB));

	float2 m_localAnchorA = (float2) (thisJoint.b.ropeJointData.localAnchorA[0] , thisJoint.b.ropeJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.ropeJointData.localAnchorB[0] , thisJoint.b.ropeJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (thisJoint.b.ropeJointData.localCenterA[0] , thisJoint.b.ropeJointData.localCenterA[1]);
	float2 m_localCenterB = (float2) (thisJoint.b.ropeJointData.localCenterB[0] , thisJoint.b.ropeJointData.localCenterB[1]);

	
	float2 rA = b2clMul_Rotate (qA, m_localAnchorA - m_localCenterA); 
	float2 rB = b2clMul_Rotate (qB, m_localAnchorB - m_localCenterB);
    float2 u = cB +rB - cA - rA ; 
	float length = sqrt(b2clDot(u,u)); 
	float C = length - thisJoint.b.ropeJointData.maxLength ; 
	 C=clamp(C, 0.0f,  b2_maxLinearCorrection);
	float impulse = -thisJoint.b.ropeJointData.mass * C ; 
	float2 P = u * impulse ; 

	

	cA -=  P * thisJoint.b.ropeJointData.invMassA ; 
	aA -= thisJoint.b.ropeJointData.invIA * b2clCross_VV (rA, P);
	cB += thisJoint.b.ropeJointData.invMassB * P ; 
	aB += thisJoint.b.ropeJointData.invIB * b2clCross_VV (rB, P);
	
	positionA.cx = cA.x;	positionA.cy = cA.y; positionA.a = aA;
	positionB.cx = cB.x;	positionB.cy = cB.y; positionB.a = aB;
	positions[thisJoint.indexA] = positionA;	positions[thisJoint.indexB] = positionB;
	 
	 
}





__kernel void TestKernel(
                                      __global clb2Position* positions,
                                      __global clb2Manifold* manifolds,
                                      __global clb2Contact* contacts,
                                      const unsigned int offset,
                                      const unsigned int contactCount
                                      )
{
    unsigned int contactIndex=get_global_id(0)+offset;
    if(contactIndex>=offset+contactCount) return;
    clb2Contact thisContact=contacts[contactIndex];
    clb2Manifold thisManifold=manifolds[contactIndex];
    int indexA=thisContact.indexA;
    int indexB=thisContact.indexB;
    clb2Position thisPosA=positions[indexA];
    clb2Position thisPosB=positions[indexB];
    
    clb2PositionSolverManifold thisPositionManifold;
    
    clb2Transform xfA,xfB;

	float sina, cosa;
	
	sina = sincos(thisPosA.a, &cosa);
	xfA.rotation.s = sina;
	xfA.rotation.c = cosa;
    //xfA.rotation.s = native_sin(thisPosA.a);
    //xfA.rotation.c = native_cos(thisPosA.a);
    //xfA.rotation.c = cos_wrapper(thisPosA.a);
     
	sina = sincos(thisPosB.a, &cosa);
	xfB.rotation.s = sina;
	xfB.rotation.c = cosa;
    //xfB.rotation.s = native_sin(thisPosB.a);
    //xfB.rotation.c = native_cos(thisPosB.a);
    //xfB.rotation.c = cos_wrapper(thisPosB.a);


    xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
    xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y);  
    
    if(thisManifold.type==0)
        computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==1)
        computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==2)
        computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    
    float2 dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
    float2 dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
    
    float C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
    
    float rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
    float rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
    
    float K=thisContact.invMassA + thisContact.invMassB + thisContact.invIA*rnA*rnA + thisContact.invIB*rnB*rnB;
    
    float2 P=max(-C/K,0.0f)*thisPositionManifold.normal;
    
    thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
    thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
    thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 
    
    thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
    thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
    thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);
    
    if(thisManifold.pointCount==2){
        
		sina = sincos(thisPosA.a, &cosa);
		xfA.rotation.s = sina;
		xfA.rotation.c = cosa;
    	//xfA.rotation.s = native_sin(thisPosA.a);
    	//xfA.rotation.c = native_cos(thisPosA.a);
    	//xfA.rotation.c = cos_wrapper(thisPosA.a);
    	
		sina = sincos(thisPosB.a, &cosa);
		xfB.rotation.s = sina;
		xfB.rotation.c = cosa;
    	//xfB.rotation.s = native_sin(thisPosB.a);
    	//xfB.rotation.c = native_cos(thisPosB.a);
    	//xfB.rotation.c = cos_wrapper(thisPosB.a);
    	
        xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
        xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y); 
        
        if(thisManifold.type==0)
            computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==1)
            computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==2)
            computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        
        dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
        dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
        
        C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
        
        rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
        rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
        
        K=thisContact.invMassA + thisContact.invMassB + thisContact.invIA*rnA*rnA + thisContact.invIB*rnB*rnB;
        
        P=max(-C/K,0.0f)*thisPositionManifold.normal;
        
        thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
        thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
        thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 
        
        thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
        thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
        thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);
    }
    
    positions[indexA]=thisPosA;
    positions[indexB]=thisPosB;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveWheelJointPositionConstraint
//
// Compute positions of bodies which are constrained by a Wheel joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveWheelJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;

    if(jointIndex >= offset + colorLength) 
		return;

	b2clJoint thisJoint = jointList[jointIndex]; 

	clb2Position positionA = positions[thisJoint.indexA];
	clb2Position positionB = positions[thisJoint.indexB];

	float2 cA = (float2)(positionA.cx, positionA.cy);
	float aA = positionA.a;
	float2 cB = (float2)(positionB.cx, positionB.cy);
	float aB = positionB.a;

	float sina, cosa;

	sina = sincos(aA, &cosa);
	float2 qA = (float2)(sina, cosa);
	sina = sincos(aB, &cosa);
	float2 qB = (float2)(sina, cosa);
	//float2 qA = (float2)(sin(aA), cos(aA));
	//float2 qB = (float2)(sin(aB), cos(aB));
	//float2 qA = (float2)(sin(aA), cos_wrapper(aA));
	//float2 qB = (float2)(sin(aB), cos_wrapper(aB));

	float2 rA = b2clMul_Rotate(qA, (float2)(thisJoint.b.wheelJointData.localAnchorA[0] - thisJoint.b.wheelJointData.localCenterA[0], thisJoint.b.wheelJointData.localAnchorA[1] - thisJoint.b.wheelJointData.localCenterA[1]));
	float2 rB = b2clMul_Rotate(qB, (float2)(thisJoint.b.wheelJointData.localAnchorB[0] - thisJoint.b.wheelJointData.localCenterB[0], thisJoint.b.wheelJointData.localAnchorB[1] - thisJoint.b.wheelJointData.localCenterB[1]));
	float2 d = (cB - cA) + rB - rA;

	float2 m_localYAxisA = (float2)(thisJoint.b.wheelJointData.localYAxisA[0], thisJoint.b.wheelJointData.localYAxisA[1]);

	float2 ay = b2clMul_Rotate(qA, m_localYAxisA);

	float sAy = b2clCross_VV(d + rA, ay);
	float sBy = b2clCross_VV(rB, ay);

	float C = b2clDot(d, ay);

	float k = thisJoint.b.wheelJointData.invMassA + thisJoint.b.wheelJointData.invMassB + thisJoint.b.wheelJointData.invIA * thisJoint.b.wheelJointData.sAy * thisJoint.b.wheelJointData.sAy + thisJoint.b.wheelJointData.invIB * thisJoint.b.wheelJointData.sBy * thisJoint.b.wheelJointData.sBy;

	float impulse;
	if (k != 0.0f)
	{
		impulse = - C / k;
	}
	else
	{
		impulse = 0.0f;
	}

	float2 P = impulse * ay;
	float LA = impulse * sAy;
	float LB = impulse * sBy;

	cA -= thisJoint.b.wheelJointData.invMassA * P;
	aA -= thisJoint.b.wheelJointData.invIA * LA;
	cB += thisJoint.b.wheelJointData.invMassB * P;
	aB += thisJoint.b.wheelJointData.invIB * LB;

	positionA.cx = cA.x;
	positionA.cy = cA.y;
	positionA.a = aA;
	positionB.cx = cB.x;
	positionB.cy = cB.y;
	positionB.a = aB;
	positions[thisJoint.indexA] = positionA;
	positions[thisJoint.indexB] = positionB;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveWeldJointPositionConstraint
//
// Compute positions of bodies which are constrained by a Weld joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveWeldJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;

    if(jointIndex >= offset + colorLength) 
		return;

	b2clJoint thisJoint = jointList[jointIndex]; 
	
	clb2Position positionA = positions[thisJoint.indexA];
	clb2Position positionB = positions[thisJoint.indexB];

	float2 cA = (float2)(positionA.cx, positionA.cy);
	float aA = positionA.a;
	float2 cB = (float2)(positionB.cx, positionB.cy);
	float aB = positionB.a;

	float sina, cosa;

	sina = sincos(aA, &cosa);
	float2 qA = (float2)(sina, cosa);
	sina = sincos(aB, &cosa);
	float2 qB = (float2)(sina, cosa);

	//float2 qA = (float2)(sin(aA), cos(aA));
	//float2 qB = (float2)(sin(aB), cos(aB));
	//float2 qA = (float2)(sin(aA), cos_wrapper(aA));
	//float2 qB = (float2)(sin(aB), cos_wrapper(aB));

	float2 rA = b2clMul_Rotate(qA, (float2)(thisJoint.b.weldJointData.localAnchorA[0] - thisJoint.b.weldJointData.localCenterA[0], thisJoint.b.weldJointData.localAnchorA[1] - thisJoint.b.weldJointData.localCenterA[1]));
	float2 rB = b2clMul_Rotate(qB, (float2)(thisJoint.b.weldJointData.localAnchorB[0] - thisJoint.b.weldJointData.localCenterB[0], thisJoint.b.weldJointData.localAnchorB[1] - thisJoint.b.weldJointData.localCenterB[1]));

	float mA = thisJoint.b.weldJointData.invMassA, mB = thisJoint.b.weldJointData.invMassB;
	float iA = thisJoint.b.weldJointData.invIA, iB = thisJoint.b.weldJointData.invIB;

	// TODO: positionError, angularError

	b2clMat33 K;
	K.ex[0] = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.ey[0] = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.ez[0] = -rA.y * iA - rB.y * iB;
	K.ex[1] = K.ey[0];
	K.ey[1] = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	K.ez[1] = rA.x * iA + rB.x * iB;
	K.ex[2] = K.ez[0];
	K.ey[2] = K.ez[1];
	K.ez[2] = iA + iB;

	if (thisJoint.b.weldJointData.frequencyHz > 0.0f)
	{
		float2 C1 =  cB + rB - cA - rA;

		float2 P = -b2clMat33Solve22(K, C1);

		cA -= mA * P;
		aA -= iA * b2clCross_VV(rA, P);

		cB += mB * P;
		aB += iB * b2clCross_VV(rB, P);
	}
	else
	{
		float2 C1 =  cB + rB - cA - rA;
		float C2 = aB - aA - thisJoint.b.weldJointData.referenceAngle;

		float3 C = (float3)(C1.x, C1.y, C2);
	
		float3 impulse = -b2clMat33Solve(K, C);
		float2 P = (float2)(impulse.x, impulse.y);

		cA -= mA * P;
		aA -= iA * (b2clCross_VV(rA, P) + impulse.z);

		cB += mB * P;
		aB += iB * (b2clCross_VV(rB, P) + impulse.z);
	}

	positionA.cx = cA.x;
	positionA.cy = cA.y;
	positionA.a = aA;
	positionB.cx = cB.x;
	positionB.cy = cB.y;
	positionB.a = aB;
	positions[thisJoint.indexA] = positionA;
	positions[thisJoint.indexB] = positionB;
	
}

__kernel void SolveMouseJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;

    if(jointIndex >= offset + colorLength) 
		return;
}

__kernel void SolveFrictionJointPositionConstraint(
                                    __global clb2Position* positions,
									__global b2clJoint* jointList,
									const unsigned int offset,
									const unsigned int colorLength
									// __global testData* testBuffer
									)
{
	unsigned int jointIndex = get_global_id(0) + offset;

    if(jointIndex >= offset + colorLength) 
		return;
}

__kernel void SolveSDVelocity(
                                 __global clb2Velocity* velocities,
                                 __global clb2Contact* contacts,
								 __global clb2Impulse* impulses, 
                                 __global clb2Points* points,
								 const __global b2clBodyStatic* bodyStaticListBuffer,
								 int sdContactNum,
								 unsigned int colorOffset,
								 unsigned int colorLength
                                 
								 )

{
	int contactIndex = get_global_id(0);
	if (contactIndex >= colorLength) return ; 
	contactIndex = contactIndex + colorOffset; 
	if (contactIndex >= sdContactNum) return ; 


    clb2Contact thisContact = contacts[contactIndex];
    clb2Impulse thisImpulse = impulses[contactIndex];
    clb2Points thisPoints = points[contactIndex];
    clb2Velocity thisVelA = velocities[thisContact.indexA];
    clb2Velocity thisVelB = velocities[thisContact.indexB];

	

    // Calculate the tangent impulse for the first contact point
    float2 dv;
    dv.x = thisVelB.vx-thisVelB.w * thisPoints.rB1.y-thisVelA.vx+thisVelA.w * thisPoints.rA1.y;
    dv.y = thisVelB.vy+thisVelB.w * thisPoints.rB1.x-thisVelA.vy-thisVelA.w * thisPoints.rA1.x;
    
    float vp = dv.x*thisContact.normal.y-dv.y*thisContact.normal.x;
    float lambda = thisPoints.tangentMass1 * (-vp);
    
    float maxFriction = thisContact.friction * thisImpulse.normalImpulse1;
    float newImpulse=clamp(thisImpulse.tangentImpulse1+lambda,-maxFriction,maxFriction);
    lambda=newImpulse-thisImpulse.tangentImpulse1;
    thisImpulse.tangentImpulse1=newImpulse;


    
    float2 P; 
    P.x = lambda*thisContact.normal.y;
    P.y = -lambda*thisContact.normal.x;
    
    thisVelA.vx = thisVelA.vx-thisContact.invMassA * P.x; 
    thisVelA.vy = thisVelA.vy-thisContact.invMassA * P.y; 
    thisVelA.w = thisVelA.w-thisContact.invIA * (thisPoints.rA1.x * P.y - thisPoints.rA1.y * P.x); 
    
    thisVelB.vx = thisVelB.vx+thisContact.invMassB * P.x; 
    thisVelB.vy = thisVelB.vy+thisContact.invMassB * P.y; 
    thisVelB.w= thisVelB.w+thisContact.invIB * (thisPoints.rB1.x * P.y - thisPoints.rB1.y * P.x);
 
    // Calculate the tangent impulse for the second contact point if there is the second contact point
    if(thisPoints.normalMass2>=0)
	{
        dv.x = thisVelB.vx-thisVelB.w * thisPoints.rB2.y-thisVelA.vx+thisVelA.w * thisPoints.rA2.y;
        dv.y = thisVelB.vy+thisVelB.w * thisPoints.rB2.x-thisVelA.vy-thisVelA.w * thisPoints.rA2.x;
        
        vp = dv.x*thisContact.normal.y-dv.y*thisContact.normal.x;
        lambda = thisPoints.tangentMass2 * (-vp);
        
        maxFriction = thisContact.friction * thisImpulse.normalImpulse2;
        newImpulse=clamp(thisImpulse.tangentImpulse2+lambda,-maxFriction,maxFriction);
        lambda=newImpulse-thisImpulse.tangentImpulse2;
        thisImpulse.tangentImpulse2=newImpulse;
        
        P.x = lambda*thisContact.normal.y;
        P.y = -lambda*thisContact.normal.x;
        
        thisVelA.vx = thisVelA.vx-thisContact.invMassA * P.x; 
        thisVelA.vy = thisVelA.vy-thisContact.invMassA * P.y; 
        thisVelA.w = thisVelA.w-thisContact.invIA * (thisPoints.rA2.x * P.y - thisPoints.rA2.y * P.x); 
        
        thisVelB.vx = thisVelB.vx+thisContact.invMassB * P.x; 
        thisVelB.vy = thisVelB.vy+thisContact.invMassB * P.y; 
        thisVelB.w= thisVelB.w+thisContact.invIB * (thisPoints.rB2.x * P.y - thisPoints.rB2.y * P.x);
		
    }
    
    // Calculate the normal impulse for the first contact point
    dv.x = thisVelB.vx-thisVelB.w * thisPoints.rB1.y-thisVelA.vx+thisVelA.w * thisPoints.rA1.y;
    dv.y = thisVelB.vy+thisVelB.w * thisPoints.rB1.x-thisVelA.vy-thisVelA.w * thisPoints.rA1.x;
   

	

    vp = dv.x*thisContact.normal.x+dv.y*thisContact.normal.y;
	
	
    lambda = -thisPoints.normalMass1 * (vp-thisPoints.velocityBias1);

	 
    newImpulse=fmax(thisImpulse.normalImpulse1+lambda,0);
    lambda=newImpulse-thisImpulse.normalImpulse1;
	   
    thisImpulse.normalImpulse1=newImpulse;
    
    P.x = lambda*thisContact.normal.x;
    P.y = lambda*thisContact.normal.y;    
	



    thisVelA.vx = thisVelA.vx-thisContact.invMassA * P.x; 
    thisVelA.vy = thisVelA.vy-thisContact.invMassA * P.y; 
    thisVelA.w = thisVelA.w-thisContact.invIA * (thisPoints.rA1.x * P.y - thisPoints.rA1.y * P.x); 
 
    
    thisVelB.vx = thisVelB.vx+thisContact.invMassB * P.x; 
    thisVelB.vy = thisVelB.vy+thisContact.invMassB * P.y; 
    thisVelB.w= thisVelB.w+thisContact.invIB * (thisPoints.rB1.x * P.y - thisPoints.rB1.y * P.x);
	thisVelB.w = 0; 

    
    // Calculate the normal impulse for the second contact point if there is the second contact point
    if(thisPoints.normalMass2>=0)
	{
        dv.x = thisVelB.vx-thisVelB.w * thisPoints.rB2.y-thisVelA.vx+thisVelA.w * thisPoints.rA2.y;
        dv.y = thisVelB.vy+thisVelB.w * thisPoints.rB2.x-thisVelA.vy-thisVelA.w * thisPoints.rA2.x;

        vp = dv.x*thisContact.normal.x+dv.y*thisContact.normal.y;
        lambda = -thisPoints.normalMass2 * (vp-thisPoints.velocityBias2);
    
        newImpulse=fmax(thisImpulse.normalImpulse2+lambda,0);
        lambda=newImpulse-thisImpulse.normalImpulse2;
        thisImpulse.normalImpulse2=newImpulse;
        
        P.x = lambda*thisContact.normal.x;
        P.y = lambda*thisContact.normal.y;
        
        thisVelA.vx = thisVelA.vx-thisContact.invMassA * P.x; 
        thisVelA.vy = thisVelA.vy-thisContact.invMassA * P.y; 
        thisVelA.w = thisVelA.w-thisContact.invIA * (thisPoints.rA2.x * P.y - thisPoints.rA2.y * P.x); 

        
        thisVelB.vx = thisVelB.vx+thisContact.invMassB * P.x; 
        thisVelB.vy = thisVelB.vy+thisContact.invMassB * P.y; 
        thisVelB.w = thisVelB.w+thisContact.invIB * (thisPoints.rB2.x * P.y - thisPoints.rB2.y * P.x);
		

		 thisVelB.w = 0; 
		 
    }
    	
   
    // Copy the results back to the buffers
    impulses[contactIndex] = thisImpulse;
	velocities[thisContact.indexA] = thisVelA;
	velocities[thisContact.indexB] = thisVelB;
	//if ( bodyStaticListBuffer[thisContact.indexA].m_type == 2)   velocities[thisContact.indexA]=thisVelA;
	//if ( bodyStaticListBuffer[thisContact.indexA].m_type == 2)   velocities[thisContact.indexA]=thisVelB;
}


__kernel void SolveSDPosition(
                                      __global clb2Position* positions,
                                      __global clb2Manifold* manifolds,
                                      __global clb2Contact* contacts,
								 const __global b2clBodyStatic* bodyStaticListBuffer,
								 int sdContactNum, 
								 unsigned int colorOffset, 
								 unsigned int colorLength
                                      )
{
	int contactIndex = get_global_id(0);
	if (contactIndex >= colorLength) return ; 
	contactIndex = contactIndex + colorOffset ; 
	if (contactIndex >= sdContactNum) return ; 

    clb2Contact thisContact=contacts[contactIndex];
    clb2Manifold thisManifold=manifolds[contactIndex];
    int indexA=thisContact.indexA;
    int indexB=thisContact.indexB;
    clb2Position thisPosA=positions[indexA];
    clb2Position thisPosB=positions[indexB];

    
    clb2PositionSolverManifold thisPositionManifold;
    
    clb2Transform xfA,xfB;
    float sina, cosa;
    
    sina = sincos(thisPosA.a, &cosa);
    xfA.rotation.s = sina;
    xfA.rotation.c = cosa;
    //xfA.rotation.s = native_sin(thisPosA.a);
    //xfA.rotation.c = native_cos(thisPosA.a);
    //xfA.rotation.c = cos_wrapper(thisPosA.a);
    
    sina = sincos(thisPosB.a, &cosa);
    xfB.rotation.s = sina;
    xfB.rotation.c = cosa;
    //xfB.rotation.s = native_sin(thisPosB.a);
    //xfB.rotation.c = native_cos(thisPosB.a);
    //xfB.rotation.c = cos_wrapper(thisPosB.a);
    
    xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
    xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y);  
    
    if(thisManifold.type==0)
        computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==1)
        computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    else if(thisManifold.type==2)
        computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,1);
    
    float2 dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
    float2 dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
    
    float C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
    
    float rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
    float rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
    
    float K=thisContact.invMassA + thisContact.invMassB + thisContact.invIA*rnA*rnA + thisContact.invIB*rnB*rnB;
    
    float2 P=max(-C/K,0.0f)*thisPositionManifold.normal;
 
    
    thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
    thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
    thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 
    
    thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
    thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
    thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);
    
    if(thisManifold.pointCount==2){
        
 	    sina = sincos(thisPosA.a, &cosa);
    	xfA.rotation.s = sina;
    	xfA.rotation.c = cosa;
        //xfA.rotation.s = native_sin(thisPosA.a);
        //xfA.rotation.c = native_cos(thisPosA.a);
        //xfA.rotation.c = cos_wrapper(thisPosA.a);
        
 	    sina = sincos(thisPosB.a, &cosa);
    	xfB.rotation.s = sina;
    	xfB.rotation.c = cosa;
        //xfB.rotation.s = native_sin(thisPosB.a);
        //xfB.rotation.c = native_cos(thisPosB.a);
        //xfB.rotation.c = cos_wrapper(thisPosB.a);
        
        xfA.translation=(float2)(thisPosA.cx,thisPosA.cy) - (float2)(xfA.rotation.c * thisManifold.localCenterA.x - xfA.rotation.s * thisManifold.localCenterA.y, xfA.rotation.s * thisManifold.localCenterA.x + xfA.rotation.c * thisManifold.localCenterA.y);
        xfB.translation=(float2)(thisPosB.cx,thisPosB.cy) - (float2)(xfB.rotation.c * thisManifold.localCenterB.x - xfB.rotation.s * thisManifold.localCenterB.y, xfB.rotation.s * thisManifold.localCenterB.x + xfB.rotation.c * thisManifold.localCenterB.y); 
        
        if(thisManifold.type==0)
            computePositionSolverManifoldCircles(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==1)
            computePositionSolverManifoldFaceA(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        else if(thisManifold.type==2)
            computePositionSolverManifoldFaceB(&thisManifold,&thisPositionManifold,&xfA,&xfB,2);
        
        dA=thisPositionManifold.point - (float2)(thisPosA.cx,thisPosA.cy);
        dB=thisPositionManifold.point - (float2)(thisPosB.cx,thisPosB.cy);
        
        C=clamp(b2_baumgarte*(thisPositionManifold.separation+b2_linearSlop),-b2_maxLinearCorrection,0.0f);
        
        rnA=dA.x*thisPositionManifold.normal.y - dA.y*thisPositionManifold.normal.x;
        rnB=dB.x*thisPositionManifold.normal.y - dB.y*thisPositionManifold.normal.x;
        
        K=thisContact.invMassA + thisContact.invMassB + thisContact.invIA*rnA*rnA + thisContact.invIB*rnB*rnB;
        
        P=max(-C/K,0.0f)*thisPositionManifold.normal;
        

        thisPosA.cx=thisPosA.cx-thisContact.invMassA*P.x;
        thisPosA.cy=thisPosA.cy-thisContact.invMassA*P.y;
        thisPosA.a = thisPosA.a-thisContact.invIA * (dA.x * P.y - dA.y * P.x); 
        
        thisPosB.cx = thisPosB.cx+thisContact.invMassB * P.x; 
        thisPosB.cy = thisPosB.cy+thisContact.invMassB * P.y; 
        thisPosB.a = thisPosB.a+thisContact.invIB * (dB.x * P.y - dB.y * P.x);
		
    }
	


	positions[indexA] = thisPosA ; 
	positions[indexB] = thisPosB ; 
   // if ( bodyStaticListBuffer[indexA].m_type == 2)   positions[indexA]=thisPosA;
   // if ( bodyStaticListBuffer[indexB].m_type == 2)  positions[indexB]=thisPosB;
}

inline float b2clMixFr(float friction1, float friction2)
{
	return sqrt(friction1 * friction2);
}
inline float b2clMixRes(float restitution1, float restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}
__kernel void CollectStaticDynamicPairKernel (
                                 __global clb2Contact* contacts, // output
                                 __global clb2Points* points, // output
                                 __global clb2Manifold* manifoldBuffer, // output
								__global clb2Impulse* impulses, 
								 const __global clb2Velocity* velocities,
						         const __global clb2Position* positions,
						         const __global b2clFixtureStatic* fixtureStaticListBuffer,
								const __global b2clBodyStatic* bodyStaticListBuffer,
								const __global b2clPolygonShape* polyGlobal,
								const __global b2clTransform* xfGlobal,
								const __global clb2SDContact* contactDataBuffer,
								const __global clb2SDManifold* manifoldDataBuffer,
								const unsigned int contactCount

) 
{
	unsigned int contactIndex = get_global_id(0) ;
	if (contactIndex >= contactCount) return ; 
    clb2Contact thisContact;
    clb2Points currentPoints;
    clb2Manifold currentManifolds;

	int mfElmSize = 10 ; 
	int contactElmSize = 4 ; 

	int4 currentIndices; 

	clb2SDContact sdcontact = contactDataBuffer[contactIndex];
	
	
   // currentIndices.x = contactDataBuffer[contactIndex*contactElmSize+0]; currentIndices.y = contactDataBuffer[contactIndex*contactElmSize+1];currentIndices.z = contactDataBuffer[contactIndex*contactElmSize+2];currentIndices.w = contactDataBuffer[contactIndex*contactElmSize+3];
   currentIndices.x = sdcontact.fixtureAIndex; currentIndices.y = sdcontact.fixtureBIndex;
   currentIndices.z = sdcontact.bodyAIndex   ; currentIndices.w = sdcontact.bodyBIndex;
	float frictionA = fixtureStaticListBuffer[currentIndices.x].m_friction;
	float frictionB = fixtureStaticListBuffer[currentIndices.y].m_friction;
	thisContact.friction = b2clMixFr(frictionA, frictionB);

	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[currentIndices.z];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[currentIndices.w];
	thisContact.invMassA = bodyStaticA.m_invMass;
	thisContact.invMassB = bodyStaticB.m_invMass;
	thisContact.invIA = bodyStaticA.m_invI;
	thisContact.invIB = bodyStaticB.m_invI;
	thisContact.indexA = currentIndices.z;
	thisContact.indexB = currentIndices.w;

	

	//float manifold[20];
	//for (int i = 0 ; i < mfElmSize ; i ++ ) {manifold[i] = manifoldDataBuffer[contactIndex*mfElmSize+i]; }
	clb2SDManifold manifold = manifoldDataBuffer[contactIndex]; 

	b2clTransform xfA = xfGlobal[currentIndices.z];
	b2clTransform xfB = xfGlobal[currentIndices.w];
	float radiusA = polyGlobal[currentIndices.x].m_radius;
	float radiusB = polyGlobal[currentIndices.y].m_radius;
    float2 worldManifoldPoints[b2cl_maxManifoldPoints];
	float2 normal;
	int type = manifold.type ; 
	switch ( type)
	{
	case 0:
		{
			normal = (float2)(1.0, 0.0);
			//float2 pointA = b2clMul_Transform(&xfA,  (float2) (manifold[2], manifold[3]));
			//float2 pointB = b2clMul_Transform(&xfB, (float2) (manifold[6], manifold[7]));
			float2 pointA = b2clMul_Transform(&xfA,  (float2) (manifold.localPointX, manifold.localPointY));
			float2 pointB = b2clMul_Transform(&xfB, (float2) (manifold.point0X, manifold.point0Y));		
			if (b2clDistanceSquared(pointA, pointB) > b2_epsilon * b2_epsilon)
			{
				normal = normalize(pointB - pointA);
			}

			float2 cA = pointA + radiusA * normal;
			float2 cB = pointB - radiusB * normal;
			worldManifoldPoints[0] = 0.5f * (cA + cB);
		}
		break;
	case 1:
		{
			//normal = b2clMul_Rotate(xfA.q, (float2) (manifold[4], manifold[5])    );
			//float2 planePoint = b2clMul_Transform(&xfA, (float2) (manifold[2], manifold[3])  );
			normal = b2clMul_Rotate(xfA.q, (float2) (manifold.localNormalX, manifold.localNormalY)    );
			float2 planePoint = b2clMul_Transform(&xfA, (float2) (manifold.localPointX, manifold.localPointY)  );		
			//for (int i = 0; i < manifold[1]; ++i)
			for (int i = 0 ; i < manifold.pointCount; ++i )
			{
				//float2 clipPoint = b2clMul_Transform(&xfB, (float2) (manifold[6+2*i], manifold[7+2*i])    );
				float2 clipPoint ; 
				if (i == 0) clipPoint = b2clMul_Transform(&xfB, (float2) (manifold.point0X, manifold.point0Y)    );
				else clipPoint = b2clMul_Transform(&xfB, (float2) (manifold.point1X, manifold.point1Y)    );
					
				float2 cA = clipPoint + (radiusA - b2clDot(clipPoint - planePoint, normal)) * normal;
				float2 cB = clipPoint - radiusB * normal;
				worldManifoldPoints[i] = 0.5f * (cA + cB);
				//worldManifoldPoints[i] = clipPoint;
			}
			break;
		}
	case 2:
		{
			//normal = b2clMul_Rotate(xfB.q, (float2) (manifold[4], manifold[5])   );
			//float2 planePoint = b2clMul_Transform(&xfB, (float2) (manifold[2], manifold[3])     );
			normal = b2clMul_Rotate(xfB.q, (float2) (manifold.localNormalX, manifold.localNormalY)   );
			float2 planePoint = b2clMul_Transform(&xfB, (float2) (manifold.localPointX, manifold.localPointY)     );

			//for (int i = 0; i < manifold[1]; ++i)
			for (int i = 0; i < manifold.pointCount; ++i)
			{
			//	float2 clipPoint = b2clMul_Transform(&xfA, (float2) (manifold[6+2*i], manifold[7+2*i])       );
				float2 clipPoint ; 
				if (i == 0) clipPoint = b2clMul_Transform(&xfB, (float2) (manifold.point0X, manifold.point0Y)    );
				else clipPoint = b2clMul_Transform(&xfB, (float2) (manifold.point1X, manifold.point1Y)    );
				float2 cB = clipPoint + (radiusB - b2clDot(clipPoint - planePoint, normal)) * normal;
				float2 cA = clipPoint - radiusA * normal;
				worldManifoldPoints[i] = 0.5f * (cA + cB);
			}

			// Ensure normal points from A to B.
			normal = -normal;
			break;
		}
	}



	thisContact.normal = normal;

	clb2Position positionA = positions[currentIndices.z];
	clb2Position positionB = positions[currentIndices.w];
	currentPoints.rA1 = worldManifoldPoints[0] - (float2)(positionA.cx, positionA.cy);
	currentPoints.rB1 = worldManifoldPoints[0] - (float2)(positionB.cx, positionB.cy);

	float rnA = b2clCross_VV(currentPoints.rA1, normal);
	float rnB = b2clCross_VV(currentPoints.rB1, normal);
	float kNormal = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rnA * rnA + thisContact.invIB * rnB * rnB;
	currentPoints.normalMass1 = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

	float2 tangent = b2clCross_VS(normal, 1.0f);
	float rtA = b2clCross_VV(currentPoints.rA1, tangent);
	float rtB = b2clCross_VV(currentPoints.rB1, tangent);
	float kTangent = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rtA * rtA + thisContact.invIB * rtB * rtB;
	currentPoints.tangentMass1 = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;

	// Setup a velocity bias for restitution.
	clb2Velocity velocityA = velocities[currentIndices.z];
	clb2Velocity velocityB = velocities[currentIndices.w];
	float2 vA = (float2)(velocityA.vx, velocityA.vy);
	float2 vB = (float2)(velocityB.vx, velocityB.vy);
	float wA = velocityA.w;
	float wB = velocityB.w;
	float restitutionA = fixtureStaticListBuffer[currentIndices.x].m_restitution;
	float restitutionB = fixtureStaticListBuffer[currentIndices.y].m_restitution;
	float mixedrestitution = b2clMixRes(restitutionA, restitutionB);
	currentPoints.velocityBias1 = 0.0f;
	float vRel = b2clDot(normal, vB + b2clCross_SV(wB, currentPoints.rB1) - vA - b2clCross_SV(wA, currentPoints.rA1));
	if (vRel < -b2cl_velocityThreshold)
	{
		currentPoints.velocityBias1 = -mixedrestitution * vRel;
	}
  //  currentManifolds.localNormal = (float2) (manifold[4], manifold[5]);
  //  currentManifolds.localPoint = (float2) (manifold[2], manifold[3]);
  //  currentManifolds.type=manifold[0];
  //  currentManifolds.pointCount=manifold[1];
 //   currentManifolds.localPoints1=(float2) (manifold[6], manifold[7]);
    currentManifolds.localNormal = (float2) (manifold.localNormalX, manifold.localNormalY);
    currentManifolds.localPoint = (float2) (manifold.localPointX, manifold.localPointY);
    currentManifolds.type=manifold.type;
    currentManifolds.pointCount=manifold.pointCount;
    currentManifolds.localPoints1=(float2) (manifold.point0X, manifold.point0Y);
    currentManifolds.radiusA=radiusA;
    currentManifolds.radiusB=radiusB;
    currentManifolds.localCenterA=bodyStaticA.m_localCenter;
    currentManifolds.localCenterB=bodyStaticB.m_localCenter;
	//if (manifold[1]>1)
	if (manifold.pointCount>1)
	{
		currentPoints.rA2 = worldManifoldPoints[1] - (float2)(positionA.cx, positionA.cy);
		currentPoints.rB2 = worldManifoldPoints[1] - (float2)(positionB.cx, positionB.cy);

		float rnA = b2clCross_VV(currentPoints.rA2, normal);
		float rnB = b2clCross_VV(currentPoints.rB2, normal);
		float kNormal = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rnA * rnA + thisContact.invIB * rnB * rnB;
		currentPoints.normalMass2 = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

		float2 tangent = b2clCross_VS(normal, 1.0f);
		float rtA = b2clCross_VV(currentPoints.rA2, tangent);
		float rtB = b2clCross_VV(currentPoints.rB2, tangent);
		float kTangent = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rtA * rtA + thisContact.invIB * rtB * rtB;
		currentPoints.tangentMass2 = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;

		currentPoints.velocityBias2 = 0.0f;
		float vRel = b2clDot(normal, vB + b2clCross_SV(wB, currentPoints.rB2) - vA - b2clCross_SV(wA, currentPoints.rA2));
		if (vRel < -b2cl_velocityThreshold)
		{
			currentPoints.velocityBias2 = -mixedrestitution * vRel;
		}

		//currentManifolds.localPoints2=(float2) (manifold[8], manifold[9]);
		currentManifolds.localPoints2=(float2) (manifold.point1X, manifold.point1Y);
	}
	else
	{
		currentPoints.rA2 = -1;
		currentPoints.rB2 = -1;
		currentPoints.normalMass2 = -1;
		currentPoints.tangentMass2 = -1;
		currentPoints.velocityBias2 = -1;
	}



 //clb2Impulse thisImpulse = impulses[contactIndex];
 clb2Impulse thisImpulse;
 thisImpulse.tangentImpulse1=thisImpulse.normalImpulse1 =  thisImpulse.tangentImpulse2=thisImpulse.normalImpulse2 = 0 ;
 impulses[contactIndex] = thisImpulse ; 


// printf ("thisContactIndexA: %d, thisContact.indexB: %d \n", thisContact.indexA, thisContact.indexB); 

	contacts[contactIndex] = thisContact;

	points[contactIndex].rA1 = currentPoints.rA1;
	points[contactIndex].rB1 = currentPoints.rB1;
	points[contactIndex].normalMass1 = currentPoints.normalMass1;
	points[contactIndex].tangentMass1 = currentPoints.tangentMass1;
	points[contactIndex].velocityBias1 = currentPoints.velocityBias1;

	points[contactIndex].rA2 = currentPoints.rA2;
	points[contactIndex].rB2 = currentPoints.rB2;
	points[contactIndex].normalMass2 = currentPoints.normalMass2;
	points[contactIndex].tangentMass2 = currentPoints.tangentMass2;
	points[contactIndex].velocityBias2 = currentPoints.velocityBias2;
	
	 
    manifoldBuffer[contactIndex] = currentManifolds;

	
}

__kernel void InitStaticDynamicBodyKernel (
	__global clb2Position* positions,
	__global b2clTransform* xfGlobal,
	const __global clb2SDBody* sdBodyData,
	int bodyNum
)
{


	int index = get_global_id(0);
	if (index >= bodyNum) return ; 

	int bodyElmSize = 12 ; 
	//float value[20] ; 
	//memcpy ( value, &(sdBodyData[index*8], sizeof(float)*8);
	//for (int i = 0 ; i < bodyElmSize ; i ++) {value[i] = sdBodyData[index*bodyElmSize+i]; }
	clb2SDBody value = sdBodyData[index]; 
	//int bodyIndex = (int)value[0] ;
	int bodyIndex = value.bodyIndex ;  
	clb2Position pos ; 
	b2clTransform t ; 


	//pos.cx = value[1] ; 
    //pos.cy = value[2] ; 
	//pos.a = value[3] ;
	//t.p= (float2) (value[4], value[5]);
	//t.q = (float2) (value[6],value[7]) ; 

	pos.cx = value.posX; pos.cy = value.posY ; pos.a = value.posAngle;
	t.p= (float2) (value.xfX, value.xfY);
	t.q = (float2) (value.xfS, value.xfC); 


	clb2Position prevPos = positions[bodyIndex]; 
	positions[bodyIndex] = pos ; 
	xfGlobal[bodyIndex] = t ; 

}

__kernel void syncSDBody (
                                __global clb2Velocity* velocities,
                                __global clb2Position* positions,
								__global b2clTransform* xfGlobal, 
								__global b2clBodyDynamic* bodyDynamicListBuffer,
								const __global b2clBodyStatic* bodyStaticListBuffer,
								__global clb2SDBody* bodyDataArray, 
								const float stepdt, 
								int bodyNum 
)
{
    int arrayIndex=get_global_id(0);
    
    if(arrayIndex>=bodyNum) return;
	int bodyElmSize = 12 ; 

   // int bodyIndex = bodyDataArray[arrayIndex* bodyElmSize+0]; 


   clb2SDBody sdbody = bodyDataArray[arrayIndex] ; 
   int bodyIndex = sdbody.bodyIndex; 

	//float dt = bodyDataArray[arrayIndex*bodyElmSize+8]*stepdt; 
	float dt = sdbody.alpha * stepdt ; 
// Integrate position. 

    clb2Position thisPosition=positions[bodyIndex];
    clb2Velocity thisVelocity=velocities[bodyIndex];
    
    float2 translation = (float2)(dt*thisVelocity.vx, dt*thisVelocity.vy);
    float translationLength=sqrt(dot(translation,translation));
    if(translationLength>b2_maxTranslation){
        thisVelocity.vx *= (b2_maxTranslation/translationLength);
		thisVelocity.vy *= (b2_maxTranslation/translationLength);
    }
    float rotation = dt*thisVelocity.w;
    float rotationMagnitude = fabs(rotation);

    if(rotationMagnitude>b2_maxRotation){
        thisVelocity.w *= (b2_maxRotation/rotationMagnitude);
    }
    
    thisPosition.cx += dt*thisVelocity.vx;
	thisPosition.cy += dt*thisVelocity.vy;
    thisPosition.a += dt*thisVelocity.w;
    
	

    positions[bodyIndex] = thisPosition;
    velocities[bodyIndex] = thisVelocity;


// Syncronize

	b2clBodyStatic bs = bodyStaticListBuffer[bodyIndex];
	__global b2clBodyDynamic* bd = bodyDynamicListBuffer + bodyIndex;

	float2 c;
	c.x = thisPosition.cx;
	c.y = thisPosition.cy;
	float a = thisPosition.a;

	// synchronize xf for each body
	float2 p, q;
	q.x = sin(a);
	q.y = cos(a);
	p = c - b2clMul_Rotate(q, bs.m_localCenter);
	xfGlobal[bodyIndex].p = p;
	xfGlobal[bodyIndex].q = q;

	bd->m_sweep.c = c;
	bd->m_sweep.a = a;

	//float value[20] ; 
	//value[1] = c.x; value[2] = c.y ; value[3] = a ; value[4] = p.x ; value[5] = p.y ; value[6] = q.x; value[7] = q.y ; 
	//value[9] = thisVelocity.vx ; 	value[10] = thisVelocity.vy ; value[11] = thisVelocity.w ; 

	sdbody.posX = c.x ; sdbody.posY = c.y ; sdbody.posAngle = a ; sdbody.xfX = p.x ; sdbody.xfY = p.y ; sdbody.xfS = q.x ; sdbody.xfC = q.y ; 
	sdbody.velocityX = thisVelocity.vx; sdbody.velocityY = thisVelocity.vy; sdbody.velocityAngular = thisVelocity.w ; 
	bodyDataArray[arrayIndex] = sdbody ; 

  //  for (int i = 1 ; i < bodyElmSize ; i ++) {
//		bodyDataArray[arrayIndex*bodyElmSize+i] = value[i] ; 
//	}


}