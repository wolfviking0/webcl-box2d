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
#define b2cl_maxManifoldPoints	2

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
#define b2cl_maxPolygonVertices	8

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
//#define b2cl_velocityThreshold		0.01f
#define b2cl_velocityThreshold		1.0f

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
#define b2cl_aabbMultiplier		2.0f

#define	b2_maxFloat	FLT_MAX
#define	b2_epsilon FLT_EPSILON
#define b2_pi 3.14159265359f
#define b2_maxTranslation 2.0f
#define b2_maxRotation 0.5f*b2_pi
#define b2_baumgarte 0.2f
#define b2_linearSlop 0.005f
#define b2_angularSlop			(2.0f / 180.0f * b2_pi)
#define b2_polygonRadius		(2.0f * b2_linearSlop)
#define b2_maxLinearCorrection 0.2f
#define b2_maxAngularCorrection		(8.0f / 180.0f * b2_pi)

#ifndef MAXFLOAT
#define MAXFLOAT      3.402823466e+38F
#endif


/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
typedef struct b2clContactFeature
{
	uchar indexA;		///< Feature index on shapeA
	uchar indexB;		///< Feature index on shapeB
	uchar typeA;		///< The feature type on shapeA
	uchar typeB;		///< The feature type on shapeB
} b2clContactFeature;

/// Contact ids to facilitate warm starting.
typedef union b2clContactID
{
	b2clContactFeature cf;
	uint key;					///< Used to quickly compare contact ids.
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
	float2 localPoint;		///< usage depends on manifold type
	float normalImpulse;	///< the non-penetration impulse
	float tangentImpulse;	///< the friction impulse
	b2clContactID id;			///< uniquely identifies a contact point between two shapes
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
	float2 localNormal;								///< not use for Type::e_points
	float2 localPoint;								///< usage depends on manifold type
	b2clManifoldPoint points[b2cl_maxManifoldPoints];	///< the points of contact
	int type;
	int pointCount;								///< the number of manifold points
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
	float2 localCenter;	///< local center of mass position
	float2 c0, c;		///< center world positions
	float a0, a;		///< world angles

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
	float2 m_localCenter;	///< local center of mass position
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
	b2clSweep m_sweep;		// the swept motion for CCD

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
	b2clMat33 mass;			// effective mass for point-to-point constraint.
	float motorMass;	// effective mass for motor/limit angular constraint.
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
	float metric;		///< length or area
	short count;
	unsigned char indexA[3];	///< vertices on shape A
	unsigned char indexB[3];	///< vertices on shape B
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
	float2 pointA;		///< closest point on shapeA
	float2 pointB;		///< closest point on shapeB
	float ndistance;
	int iterations;	///< number of GJK iterations used
} b2clDistanceOutput;

typedef struct
{
	b2clDistanceProxy proxyA;
	b2clDistanceProxy proxyB;
	b2clSweep sweepA;
	b2clSweep sweepB;
	float tMax;		// defines sweep interval [0, tMax]
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
	float2 wA;		// support point in proxyA
	float2 wB;		// support point in proxyB
	float2 w;		// wB - wA
	float a;		// barycentric coordinate for closest point
	int indexA;	// wA index
	int indexB;	// wB index
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

	out->ex[0] =  det * d;	out->ey[0] = -det * b; out->ex[2] = 0.0f;
	out->ex[1] = -det * c;	out->ey[1] =  det * a; out->ey[2] = 0.0f;
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
	out->ex[0] =  det * d;	out->ey[0] = -det * b;
	out->ex[1] = -det * c;	out->ey[1] =  det * a;
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

//==============================================================================================
// Structures
//==============================================================================================

// This structure is used to keep track of the best separating axis.
typedef struct
{
	int type;
	int index;
	float separation;
} b2clEPAxis;

// This holds polygon B expressed in frame A.
typedef struct
{
	float2 vertices[b2cl_maxPolygonVertices];
	float2 normals[b2cl_maxPolygonVertices];
	int count;
} b2clTempPolygon;

// Reference face used for clipping
typedef struct
{
	int i1, i2;
	
	float2 v1, v2;
	
	float2 normal;
	
	float2 sideNormal1;
	float sideOffset1;
	
	float2 sideNormal2;
	float sideOffset2;
} b2clReferenceFace;

//==============================================================================================
// Sub-functions
//==============================================================================================

// Find the separation between poly1 and poly2 for a give edge normal on poly1.
static float b2clEdgeSeparation(const b2clPolygonShape* poly1, const b2clTransform* xf1, int edge1,
							  const b2clPolygonShape* poly2, const b2clTransform* xf2/*, float4 *test*/)
{
	const float2* vertices1 = poly1->m_vertices;
	const float2* normals1 = poly1->m_normals;

	int count2 = poly2->m_vertexCount;
	const float2* vertices2 = poly2->m_vertices;

	//b2Assert(0 <= edge1 && edge1 < poly1->m_vertexCount);

	// Convert normal from poly1's frame into poly2's frame.
	float2 normal1World = b2clMul_Rotate(xf1->q, normals1[edge1]);
	float2 normal1 = b2clMulT_Rotate(xf2->q, normal1World);

	// Find support vertex on poly2 for -normal.
	int index = 0;
	float minDot = MAXFLOAT;

	for (int i = 0; i < count2; ++i)
	{
		float dotProduct = dot(vertices2[i], normal1);
		if (dotProduct < minDot)
		{
			minDot = dotProduct;
			index = i;
		}
	}

	float2 v1 = b2clMul_Transform(xf1, vertices1[edge1]);
	float2 v2 = b2clMul_Transform(xf2, vertices2[index]);
	float separation = dot(v2 - v1, normal1World);
	//(*test).xy = vertices2[index];
	//(*test).zw = index;
	return separation;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
float b2clFindMaxSeparation(int* edgeIndex,
							const b2clPolygonShape* poly1, const b2clTransform* xf1,
							const b2clPolygonShape* poly2, const b2clTransform* xf2
							/*, float4 *test*/)
{
	int count1 = poly1->m_vertexCount;
	const float2* normals1 = poly1->m_normals;

	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	float2 d = b2clMul_Transform(xf2, poly2->m_centroid) - b2clMul_Transform(xf1, poly1->m_centroid);
	float2 dLocal1 = b2clMulT_Rotate(xf1->q, d);

	//(*test).x = poly1->m_centroid.x;
	//(*test).y = poly1->m_centroid.y;
	//(*test).z = poly2->m_centroid.x;
	//(*test).w = poly2->m_centroid.y;

	// Find edge normal on poly1 that has the largest projection onto d.
	int edge = 0;
	float maxDot = -MAXFLOAT;
	for (int i = 0; i < count1; ++i)
	{
		float dotProduct = dot(normals1[i], dLocal1);
		if (dotProduct > maxDot)
		{
			maxDot = dotProduct;
			edge = i;
		}
	}

	//float4 temp;
	// Get the separation for the edge normal.
	float s = b2clEdgeSeparation(poly1, xf1, edge, poly2, xf2/*, &temp*/);

	// Check the separation for the previous edge normal.
	int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
	float sPrev = b2clEdgeSeparation(poly1, xf1, prevEdge, poly2, xf2/*, &temp*/);

	// Check the separation for the next edge normal.
	int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
	float sNext = b2clEdgeSeparation(poly1, xf1, nextEdge, poly2, xf2/*, &temp*/);

	// Find the best edge and the search direction.
	int bestEdge;
	float bestSeparation;
	int increment;
	if (sPrev > s && sPrev > sNext)
	{
		increment = -1;
		bestEdge = prevEdge;
		bestSeparation = sPrev;
	}
	else if (sNext > s)
	{
		increment = 1;
		bestEdge = nextEdge;
		bestSeparation = sNext;
	}
	else
	{
		*edgeIndex = edge;
		return s;
	}

	// Perform a local search for the best edge normal.
	for ( ; ; )
	{
		if (increment == -1)
			edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
		else
			edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

		s = b2clEdgeSeparation(poly1, xf1, edge, poly2, xf2/*, &temp*/);

		if (s > bestSeparation)
		{
			bestEdge = edge;
			bestSeparation = s;
		}
		else
		{
			break;
		}
	}

	*edgeIndex = bestEdge;

	return bestSeparation;
}

static void b2clFindIncidentEdge(b2clClipVertex c[2],
							 const b2clPolygonShape* poly1, const b2clTransform* xf1, int edge1,
							 const b2clPolygonShape* poly2, const b2clTransform* xf2)
{
	const float2* normals1 = poly1->m_normals;

	int count2 = poly2->m_vertexCount;
	const float2* vertices2 = poly2->m_vertices;
	const float2* normals2 = poly2->m_normals;

	//b2Assert(0 <= edge1 && edge1 < poly1->m_vertexCount);

	// Get the normal of the reference edge in poly2's frame.
	float2 normal1 = b2clMulT_Rotate(xf2->q, b2clMul_Rotate(xf1->q, normals1[edge1]));

	// Find the incident edge on poly2.
	int index = 0;
	float minDot = MAXFLOAT;
	for (int i = 0; i < count2; ++i)
	{
		float dotProduct = dot(normal1, normals2[i]);
		if (dotProduct < minDot)
		{
			minDot = dotProduct;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int i1 = index;
	int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = b2clMul_Transform(xf2, vertices2[i1]);
	c[0].id.cf.indexA = (uint)edge1;
	c[0].id.cf.indexB = (uint)i1;
	c[0].id.cf.typeA = 1;
	c[0].id.cf.typeB = 0;

	c[1].v = b2clMul_Transform(xf2, vertices2[i2]);
	c[1].id.cf.indexA = (uint)edge1;
	c[1].id.cf.indexB = (uint)i2;
	c[1].id.cf.typeA = 1;
	c[1].id.cf.typeB = 0;
}

// Sutherland-Hodgman clipping.
int b2clClipSegmentToLine(b2clClipVertex vOut[2], const b2clClipVertex vIn[2],
							const float2 normal, float offset, int vertexIndexA)
{
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	float distance0 = dot(normal, vIn[0].v) - offset;
	float distance1 = dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

		// VertexA is hitting edgeB.
		vOut[numOut].id.cf.indexA = vertexIndexA;
		vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
		vOut[numOut].id.cf.typeA = 0;
		vOut[numOut].id.cf.typeB = 1;
		++numOut;
	}

	return numOut;
}

inline b2clEPAxis b2clComputeEdgeSeparation(bool m_front, b2clTempPolygon *m_polygonB, float2 m_normal, float2 m_v1)
{
	b2clEPAxis axis;
	axis.type = 1; //b2EPAxis::e_edgeA;
	axis.index = m_front ? 0 : 1;
	axis.separation = MAXFLOAT;
	
	for (int i = 0; i < m_polygonB->count; ++i)
	{
		float s = b2clDot(m_normal, m_polygonB->vertices[i] - m_v1);
		if (s < axis.separation)
		{
			axis.separation = s;
		}
	}
	
	return axis;
}

inline b2clEPAxis b2clComputePolygonSeparation(float2 m_normal, b2clTempPolygon *m_polygonB, float2 m_v1, float2 m_v2, float2 m_upperLimit, float2 m_lowerLimit, float m_radius)
{
	b2clEPAxis axis;
	axis.type = 0; //b2EPAxis::e_unknown;
	axis.index = -1;
	axis.separation = -MAXFLOAT;

	float2 perp = (float2)(-m_normal.y, m_normal.x);

	for (int i = 0; i < m_polygonB->count; ++i)
	{
		float2 n = -m_polygonB->normals[i];
		
		float s1 = b2clDot(n, m_polygonB->vertices[i] - m_v1);
		float s2 = b2clDot(n, m_polygonB->vertices[i] - m_v2);
		float s = min(s1, s2);
		
		if (s > m_radius)
		{
			// No collision
			axis.type = 2; //b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
			return axis;
		}
		
		// Adjacency
		if (b2clDot(n, perp) >= 0.0f)
		{
			if (b2clDot(n - m_upperLimit, m_normal) < -b2_angularSlop)
			{
				continue;
			}
		}
		else
		{
			if (b2clDot(n - m_lowerLimit, m_normal) < -b2_angularSlop)
			{
				continue;
			}
		}
		
		if (s > axis.separation)
		{
			axis.type = 2; //b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
		}
	}
	
	return axis;
}

//==============================================================================================
// Main kernel function
//==============================================================================================

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
__kernel void b2clCollidePolygons(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
					  const __global b2clPolygonShape* polyGlobal, 
					  const __global b2clTransform* xfGlobal,
					  const __global int4* global_indices,
					  const __global int* pair_indices,
					  const int maxContactNum,
					  const int contactNum)
{
    unsigned int i = get_global_id(0);

	if (i>=contactNum)
		return;

	b2clManifold manifold/* = manifolds[i]*/;

	manifold.pointCount = 0;
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape polyA, polyB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*2]; // 2 is contact type for polygon-polygon
	index = global_indices[pair_index];
	polyA = polyGlobal[index.x];
	polyB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = polyA.m_bIsSensor || polyB.m_bIsSensor;

	float totalRadius = polyA.m_radius + polyB.m_radius;

	//float4 test;

	int edgeA = 0;
	float separationA = b2clFindMaxSeparation(&edgeA, &polyA, &xfA, &polyB, &xfB/*, &test*/);

	if (separationA > totalRadius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	int edgeB = 0;
	float separationB = b2clFindMaxSeparation(&edgeB, &polyB, &xfB, &polyA, &xfA/*, &test*/);
	if (separationB > totalRadius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	const b2clPolygonShape* poly1;	// reference polygon
	const b2clPolygonShape* poly2;	// incident polygon
	b2clTransform *xf1, *xf2;
	int edge1;		// reference edge
	uint flip;
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;

	if (separationB > k_relativeTol * separationA + k_absoluteTol)
	{
		poly1 = &polyB;
		poly2 = &polyA;
		xf1 = &xfB;
		xf2 = &xfA;
		edge1 = edgeB;
		manifold.type = 2;
		flip = 1;
	}
	else
	{
		poly1 = &polyA;
		poly2 = &polyB;
		xf1 = &xfA;
		xf2 = &xfB;
		edge1 = edgeA;
		manifold.type = 1;
		flip = 0;
	}

	b2clClipVertex incidentEdge[2];
	b2clFindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	int count1 = poly1->m_vertexCount;
	const float2* vertices1 = poly1->m_vertices;

	int iv1 = edge1;
	int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	float2 v11 = vertices1[iv1];
	float2 v12 = vertices1[iv2];

	float2 localTangent = v12 - v11;
	localTangent = normalize(localTangent);
	
	float2 localNormal = b2clCross_VS(localTangent, 1.0f);
	float2 planePoint = 0.5f * (v11 + v12);

	float2 tangent = b2clMul_Rotate(xf1->q, localTangent);
	float2 normal = b2clCross_VS(tangent, 1.0f);
	
	v11 = b2clMul_Transform(xf1, v11);
	v12 = b2clMul_Transform(xf1, v12);

	// Face offset.
	float frontOffset = dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	float sideOffset1 = -dot(tangent, v11) + totalRadius;
	float sideOffset2 = dot(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.
	b2clClipVertex clipPoints1[2];
	b2clClipVertex clipPoints2[2];
	int np;

	// Clip to box side 1
	np = b2clClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

	if (np < 2)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	// Clip to negative box side 1
	np = b2clClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < 2)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	// Now clipPoints2 contains the clipped points.
	manifold.localNormal = localNormal;
	manifold.localPoint = planePoint;

	int pointCount = 0;
	for (int k = 0; k < b2cl_maxManifoldPoints; ++k)
	{
		float separation = dot(normal, clipPoints2[k].v) - frontOffset;

		if (separation <= totalRadius)
		{
			b2clManifoldPoint* cp = manifold.points + pointCount;
			cp->localPoint = b2clMulT_Transform(xf2, clipPoints2[k].v);
			cp->id = clipPoints2[k].id;
			if (flip)
			{
				// Swap features
				b2clContactFeature cf = cp->id.cf;
				cp->id.cf.indexA = cf.indexB;
				cp->id.cf.indexB = cf.indexA;
				cp->id.cf.typeA = cf.typeB;
				cp->id.cf.typeB = cf.typeA;
			}
			++pointCount;
		}
	}

	manifold.pointCount = pointCount;

	//for (int k = 0; k < manifold.pointCount; ++k)
	//{
	//	b2clManifoldPoint* mp2 = manifold.points + k;
	//	mp2->normalImpulse = 0.0f;
	//	mp2->tangentImpulse = 0.0f;
	//	int key2 = mp2->id.key;

	//	b2clManifold oldManifold = manifolds[i];
	//	for (int j = 0; j < oldManifold.pointCount; ++j)
	//	{
	//		b2clManifoldPoint* mp1 = oldManifold.points + j;

	//		if (mp1->id.key == key2)
	//		{
	//			mp2->normalImpulse = mp1->normalImpulse;
	//			mp2->tangentImpulse = mp1->tangentImpulse;
	//			break;
	//		}
	//	}
	//	manifold.test.x = oldManifold.pointCount;
	//}

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = pointCount>0 ? !bSensor : 0;
}

__kernel void b2clCollideCircles(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
					  const __global b2clPolygonShape* polyGlobal, 
					  const __global b2clTransform* xfGlobal,
					  const __global int4* global_indices,
					  const __global int* pair_indices,
					  const int maxContactNum,
					  const int contactNum)
{
    unsigned int i = get_global_id(0);

	if (i>=contactNum)
		return;

	b2clManifold manifold/* = manifolds[i]*/;

	manifold.pointCount = 0;
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape circleA, circleB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i]; // 0 is contact type for circle-circle
	index = global_indices[pair_index];
	circleA = polyGlobal[index.x];
	circleB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = circleA.m_bIsSensor || circleB.m_bIsSensor;

	float totalRadius = circleA.m_radius + circleB.m_radius;
	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	float2 d = b2clMul_Transform(&xfA, circleA.m_centroid) - b2clMul_Transform(&xfB, circleB.m_centroid);
	float distSqr = b2clDot(d, d);

	if (distSqr > totalRadius * totalRadius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	manifold.type = 0; // b2Manifold::e_circles
	manifold.localPoint = circleA.m_centroid;
	manifold.localNormal = 0;
	manifold.pointCount = 1;

	manifold.points[0].localPoint = circleB.m_centroid;
	manifold.points[0].id.key = 0;

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
}

__kernel void b2clCollidePolygonAndCircle(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
								  const __global b2clPolygonShape* polyGlobal, 
								  const __global b2clTransform* xfGlobal,
								  const __global int4* global_indices,
								  const __global int* pair_indices,
								  const int maxContactNum,
								  const int contactNum)
{
    unsigned int i = get_global_id(0);

	if (i>=contactNum)
		return;

	b2clManifold manifold/* = manifolds[i]*/;

	manifold.pointCount = 0;
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape polyA, circleB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*1]; // 1 is contact type for polygon-circle
	index = global_indices[pair_index];
	polyA = polyGlobal[index.x];
	circleB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = polyA.m_bIsSensor || circleB.m_bIsSensor;

	// Compute circle position in the frame of the polygon.
	float2 c = b2clMul_Transform(&xfB, circleB.m_centroid);
	float2 cLocal = b2clMulT_Transform(&xfA, c);

	// Find the min separating edge.
	int normalIndex = 0;
	float separation = -b2_maxFloat;
	float radius = polyA.m_radius + circleB.m_radius;
	int vertexCount = polyA.m_vertexCount;
	const float2* vertices = polyA.m_vertices;
	const float2* normals = polyA.m_normals;

	for (int i = 0; i < vertexCount; ++i)
	{
		float s = b2clDot(normals[i], cLocal - vertices[i]);

		if (s > radius)
		{
			// Early out.
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			//printf("early out, pair index: %d\n", pair_index);
			return;
		}

		if (s > separation)
		{
			separation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	int vertIndex1 = normalIndex;
	int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	float2 v1 = vertices[vertIndex1];
	float2 v2 = vertices[vertIndex2];

	// If the center is inside the polygon ...
	if (separation < b2_epsilon)
	{
		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normals[normalIndex];
		manifold.localPoint = 0.5f * (v1 + v2);
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;

		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = !bSensor;
		return;
	}

	// Compute barycentric coordinates
	float u1 = b2clDot(cLocal - v1, v2 - v1);
	float u2 = b2clDot(cLocal - v2, v1 - v2);
	if (u1 <= 0.0f)
	{
		if (b2clDistanceSquared(cLocal, v1) > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}

		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normalize(cLocal - v1);
		manifold.localPoint = v1;
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;
	}
	else if (u2 <= 0.0f)
	{
		if (b2clDistanceSquared(cLocal, v2) > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}

		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normalize(cLocal - v2);
		manifold.localPoint = v2;
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;
	}
	else
	{
		float2 faceCenter = 0.5f * (v1 + v2);
		float separation = b2clDot(cLocal - faceCenter, normals[vertIndex1]);
		if (separation > radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}

		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normals[vertIndex1];
		manifold.localPoint = faceCenter;
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;
	}

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
}

__kernel void b2clCollideEdgeAndCircle(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
								  const __global b2clPolygonShape* polyGlobal, 
								  const __global b2clTransform* xfGlobal,
								  const __global int4* global_indices,
								  const __global int* pair_indices,
								  const int maxContactNum,
								  const int contactNum)
{
    unsigned int i = get_global_id(0);

	if (i>=contactNum)
		return;

	b2clManifold manifold/* = manifolds[i]*/;

	manifold.pointCount = 0;
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape edgeA, circleB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*3]; // 3 is contact type for edge-circle
	index = global_indices[pair_index];
	edgeA = polyGlobal[index.x];
	circleB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = edgeA.m_bIsSensor || circleB.m_bIsSensor;

	// Compute circle in frame of edge
	float2 Q = b2clMulT_Transform(&xfA, b2clMul_Transform(&xfB, circleB.m_centroid));
	
	float2 A = edgeA.m_vertices[0], B = edgeA.m_vertices[1];
	float2 e = B - A;
	
	// Barycentric coordinates
	float u = b2clDot(e, B - Q);
	float v = b2clDot(e, Q - A);
	
	float radius = edgeA.m_radius + circleB.m_radius;
	
	b2clContactFeature cf;
	cf.indexB = 0;
	cf.typeB = 0; //b2ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0.0f)
	{
		float2 P = A;
		float2 d = Q - P;
		float dd = b2clDot(d, d);
		if (dd > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}
		
		// Is there an edge connected to A?
		if (edgeA.m_centroid.x) // m_centroid.x is used for m_hasVertex0
		{
			float2 A1 = edgeA.m_vertices[2]; // m_vertices[2] is used for m_vertex0
			float2 B1 = A;
			float2 e1 = B1 - A1;
			float u1 = b2clDot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0f)
			{
				manifolds[pair_index] = manifold;
				manifoldBinaryBits[pair_index] = 0;
				return;
			}
		}
		
		cf.indexA = 0;
		cf.typeA = 0; //b2ContactFeature::e_vertex;
		manifold.pointCount = 1;
		manifold.type = 0; //b2Manifold::e_circles;
		manifold.localNormal = 0;
		manifold.localPoint = P;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB.m_centroid;

		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = !bSensor;
		return;
	}
	
	// Region B
	if (u <= 0.0f)
	{
		float2 P = B;
		float2 d = Q - P;
		float dd = b2clDot(d, d);
		if (dd > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}
		
		// Is there an edge connected to B?
		if (edgeA.m_centroid.y) // m_centroid.y is used for m_hasVertex3
		{
			float2 B2 = edgeA.m_vertices[3];
			float2 A2 = B;
			float2 e2 = B2 - A2;
			float v2 = b2clDot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0f)
			{
				manifolds[pair_index] = manifold;
				manifoldBinaryBits[pair_index] = 0;
				return;
			}
		}
		
		cf.indexA = 1;
		cf.typeA = 0; //b2ContactFeature::e_vertex;
		manifold.pointCount = 1;
		manifold.type = 0; //b2Manifold::e_circles;
		manifold.localNormal = 0;
		manifold.localPoint = P;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB.m_centroid;

		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = !bSensor;
		return;
	}
	
	// Region AB
	float den = b2clDot(e, e);
	float2 P = (1.0f / den) * (u * A + v * B);
	float2 d = Q - P;
	float dd = b2clDot(d, d);
	if (dd > radius * radius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	float2 n = (float2)(-e.y, e.x);
	if (b2clDot(n, Q - A) < 0.0f)
	{
		float2 nn = (float2)(-n.x, -n.y);
		n = nn;
	}
	n = normalize(n);
	
	cf.indexA = 0;
	cf.typeA = 1; //b2ContactFeature::e_face;
	manifold.pointCount = 1;
	manifold.type = 1; //b2Manifold::e_faceA;
	manifold.localNormal = n;
	manifold.localPoint = A;
	manifold.points[0].id.key = 0;
	manifold.points[0].id.cf = cf;
	manifold.points[0].localPoint = circleB.m_centroid;

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
}

__kernel void b2clCollideEdgeAndPolygon(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
								  const __global b2clPolygonShape* polyGlobal, 
								  const __global b2clTransform* xfGlobal,
								  const __global int4* global_indices,
								  const __global int* pair_indices,
								  const int maxContactNum,
								  const int contactNum)
{
    unsigned int i = get_global_id(0);

	if (i>=contactNum)
		return;

	b2clManifold manifold/* = manifolds[i]*/;

	manifold.pointCount = 0;
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape edgeA, polygonB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*4]; // 4 is contact type for edge-polygon
	index = global_indices[pair_index];
	edgeA = polyGlobal[index.x];
	polygonB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = edgeA.m_bIsSensor || polygonB.m_bIsSensor;

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
	b2clTempPolygon m_polygonB;

	b2clTransform m_xf;
	float2 m_centroidB;
	float2 m_v0, m_v1, m_v2, m_v3;
	float2 m_normal0, m_normal1, m_normal2;
	float2 m_normal;
	int m_type1, m_type2;
	float2 m_lowerLimit, m_upperLimit;
	float m_radius;
	bool m_front;

	m_xf = b2clMulT_TwoTransform(&xfA, &xfB);
	
	m_centroidB = b2clMul_Transform(&m_xf, polygonB.m_centroid);
	
	m_v0 = edgeA.m_vertices[2]; // m_vertices[2] is used for m_vertex0
	m_v1 = edgeA.m_vertices[0]; // m_vertices[0] is used for m_vertex1
	m_v2 = edgeA.m_vertices[1]; // m_vertices[1] is used for m_vertex2
	m_v3 = edgeA.m_vertices[3]; // m_vertices[3] is used for m_vertex3
	
	bool hasVertex0 = edgeA.m_centroid.x; // m_centroid.x is used for m_hasVertex0
	bool hasVertex3 = edgeA.m_centroid.y; // m_centroid.y is used for m_hasVertex3
	
	float2 edge1 = m_v2 - m_v1;
	edge1 = normalize(edge1);
	m_normal1 = (float2)(edge1.y, -edge1.x);
	float offset1 = b2clDot(m_normal1, m_centroidB - m_v1);
	float offset0 = 0.0f, offset2 = 0.0f;
	bool convex1 = false, convex2 = false;
	
	// Is there a preceding edge?
	if (hasVertex0)
	{
		float2 edge0 = m_v1 - m_v0;
		edge0 = normalize(edge0);
		m_normal0 = (float2)(edge0.y, -edge0.x);
		convex1 = b2clCross_VV(edge0, edge1) >= 0.0f;
		offset0 = b2clDot(m_normal0, m_centroidB - m_v0);
	}
	
	// Is there a following edge?
	if (hasVertex3)
	{
		float2 edge2 = m_v3 - m_v2;
		edge2 = normalize(edge2);
		m_normal2 = (float2)(edge2.y, -edge2.x);
		convex2 = b2clCross_VV(edge1, edge2) > 0.0f;
		offset2 = b2clDot(m_normal2, m_centroidB - m_v2);
	}
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		if (convex1 && convex2)
		{
			m_front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex1)
		{
			m_front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex2)
		{
			m_front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal0;
			}
		}
		else
		{
			m_front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex0)
	{
		if (convex1)
		{
			m_front = offset0 >= 0.0f || offset1 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else
		{
			m_front = offset0 >= 0.0f && offset1 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex3)
	{
		if (convex2)
		{
			m_front = offset1 >= 0.0f || offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
		}
		else
		{
			m_front = offset1 >= 0.0f && offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = m_normal1;
			}
		}		
	}
	else
	{
		m_front = offset1 >= 0.0f;
		if (m_front)
		{
			m_normal = m_normal1;
			m_lowerLimit = -m_normal1;
			m_upperLimit = -m_normal1;
		}
		else
		{
			m_normal = -m_normal1;
			m_lowerLimit = m_normal1;
			m_upperLimit = m_normal1;
		}
	}
	
	// Get polygonB in frameA
	m_polygonB.count = polygonB.m_vertexCount;
	for (int i = 0; i < polygonB.m_vertexCount; ++i)
	{
		m_polygonB.vertices[i] = b2clMul_Transform(&m_xf, polygonB.m_vertices[i]);
		m_polygonB.normals[i] = b2clMul_Rotate(m_xf.q, polygonB.m_normals[i]);
	}
	
	m_radius = 2.0f * b2_polygonRadius;
	
	manifold.pointCount = 0;
	
	b2clEPAxis edgeAxis = b2clComputeEdgeSeparation(m_front, &m_polygonB, m_normal, m_v1);
	
	// If no valid normal can be found than this edge should not collide.
	if (edgeAxis.type == 0 /*b2EPAxis::e_unknown*/) // ??? Never true?
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	if (edgeAxis.separation > m_radius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	b2clEPAxis polygonAxis = b2clComputePolygonSeparation(m_normal, &m_polygonB, m_v1, m_v2, m_upperLimit, m_lowerLimit, m_radius);
	if (polygonAxis.type != 0 /*b2EPAxis::e_unknown*/ && polygonAxis.separation > m_radius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	// Use hysteresis for jitter reduction.
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;
	
	b2clEPAxis primaryAxis;
	if (polygonAxis.type == 0 /*b2EPAxis::e_unknown*/)
	{
		primaryAxis = edgeAxis;
	}
	else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}
	
	b2clClipVertex ie[2];
	b2clReferenceFace rf;
	if (primaryAxis.type == 1 /*b2EPAxis::e_edgeA*/)
	{
		manifold.type = 1; //b2Manifold::e_faceA;
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		int bestIndex = 0;
		float bestValue = b2clDot(m_normal, m_polygonB.normals[0]);
		for (int i = 1; i < m_polygonB.count; ++i)
		{
			float value = b2clDot(m_normal, m_polygonB.normals[i]);
			if (value < bestValue)
			{
				bestValue = value;
				bestIndex = i;
			}
		}
		
		int i1 = bestIndex;
		int i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;
		
		ie[0].v = m_polygonB.vertices[i1];
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = i1;
		ie[0].id.cf.typeA = 1; //b2ContactFeature::e_face;
		ie[0].id.cf.typeB = 0; //b2ContactFeature::e_vertex;
		
		ie[1].v = m_polygonB.vertices[i2];
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = i2;
		ie[1].id.cf.typeA = 1; //b2ContactFeature::e_face;
		ie[1].id.cf.typeB = 0; //b2ContactFeature::e_vertex;
		
		if (m_front)
		{
			rf.i1 = 0;
			rf.i2 = 1;
			rf.v1 = m_v1;
			rf.v2 = m_v2;
			rf.normal = m_normal1;
		}
		else
		{
			rf.i1 = 1;
			rf.i2 = 0;
			rf.v1 = m_v2;
			rf.v2 = m_v1;
			rf.normal = -m_normal1;
		}		
	}
	else
	{
		manifold.type = 2; //b2Manifold::e_faceB;
		
		ie[0].v = m_v1;
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = primaryAxis.index;
		ie[0].id.cf.typeA = 0; //b2ContactFeature::e_vertex;
		ie[0].id.cf.typeB = 1; //b2ContactFeature::e_face;
		
		ie[1].v = m_v2;
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = primaryAxis.index;		
		ie[1].id.cf.typeA = 0; //b2ContactFeature::e_vertex;
		ie[1].id.cf.typeB = 1; //b2ContactFeature::e_face;
		
		rf.i1 = primaryAxis.index;
		rf.i2 = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
		rf.v1 = m_polygonB.vertices[rf.i1];
		rf.v2 = m_polygonB.vertices[rf.i2];
		rf.normal = m_polygonB.normals[rf.i1];
	}
	
	rf.sideNormal1 = (float2)(rf.normal.y, -rf.normal.x);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = b2clDot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = b2clDot(rf.sideNormal2, rf.v2);
	
	// Clip incident edge against extruded edge1 side edges.
	b2clClipVertex clipPoints1[2];
	b2clClipVertex clipPoints2[2];
	int np;
	
	// Clip to box side 1
	np = b2clClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
	
	if (np < b2cl_maxManifoldPoints)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	// Clip to negative box side 1
	np = b2clClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
	
	if (np < b2cl_maxManifoldPoints)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == 1 /*b2EPAxis::e_edgeA*/)
	{
		manifold.localNormal = rf.normal;
		manifold.localPoint = rf.v1;
	}
	else
	{
		manifold.localNormal = polygonB.m_normals[rf.i1];
		manifold.localPoint = polygonB.m_vertices[rf.i1];
	}
	
	int pointCount = 0;
	for (int i = 0; i < b2cl_maxManifoldPoints; ++i)
	{
		float separation;
		
		separation = b2clDot(rf.normal, clipPoints2[i].v - rf.v1);
		
		if (separation <= m_radius)
		{
			b2clManifoldPoint* cp = manifold.points + pointCount;
			
			if (primaryAxis.type == 1 /*b2EPAxis::e_edgeA*/)
			{
				cp->localPoint = b2clMulT_Transform(&m_xf, clipPoints2[i].v);
				cp->id = clipPoints2[i].id;
			}
			else
			{
				cp->localPoint = clipPoints2[i].v;
				cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
				cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
				cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
				cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
			}
			
			++pointCount;
		}
	}
	
	manifold.pointCount = pointCount;
	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
}

__kernel void b2clCompactForOneContact(
					const __global int *CompactIn_data, // input
					__global int *CompactOut_data, // output
					__global int *numValidData // output
					)
{
	*numValidData = CompactIn_data[0];
	if (*numValidData)
		CompactOut_data[0] = 0;
}