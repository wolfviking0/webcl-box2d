
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

__kernel void SolveVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global clb2Contact* contacts,
                                 __global clb2Impulse* impulses,
                                 __global clb2Points* points,
                                 const unsigned int offset,
                                 const unsigned int length)
{
    unsigned int contactIndex = get_global_id(0)+offset;

    if(contactIndex>=offset+length) return;

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
    if(thisPoints.normalMass2>=0){
        
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
    
    // Calculate the normal impulse for the second contact point if there is the second contact point
    if(thisPoints.normalMass2>=0){
        
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
    }
    
    // Copy the results back to the buffers
    impulses[contactIndex] = thisImpulse;
    velocities[thisContact.indexA] = thisVelA;
    velocities[thisContact.indexB] = thisVelB;
}


