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
//#include <Box2D/Common/OpenCL/b2CLNarrowPhase.cl>
#if 0
uint FloatFlip(float fl)
{
	uint f = as_uint(fl);
	uint mask = -(int)(f >> 31) | 0x80000000;
	return f ^ mask;
}

float IFloatFlip(uint f)
{
	uint mask = ((f >> 31) - 1) | 0x80000000;
	uint fl = f ^ mask;
	return as_float(fl);
}

enum SeparationType
{
	e_points,
	e_faceA,
	e_faceB
};

typedef struct
{
	const b2clDistanceProxy* m_proxyA;
	const b2clDistanceProxy* m_proxyB;
	b2clSweep m_sweepA, m_sweepB;
	int m_type;
	float2 m_localPoint;
	float2 m_axis;
} b2clSeparationFunction;

float b2clSeparationFunctionInitialize(b2clSeparationFunction* sf, const b2clSimplexCache* cache,
	const b2clDistanceProxy* proxyA, const b2clSweep* sweepA,
	const b2clDistanceProxy* proxyB, const b2clSweep* sweepB,
	float t1)
{
	sf->m_proxyA = proxyA;
	sf->m_proxyB = proxyB;
	int count = cache->count;

	sf->m_sweepA = *sweepA;
	sf->m_sweepB = *sweepB;

	b2clTransform xfA, xfB;
	b2clSweepGetTransform(&sf->m_sweepA, &xfA, t1);
	b2clSweepGetTransform(&sf->m_sweepB, &xfB, t1);

	//printf ("cache->indexA: %d cache->indexB: %d \n ", cache->indexA[0], cache->indexA[1]);
	//printf ("cache->indexA: %d \n ", );
	if (count == 1)
	{
		sf->m_type = e_points;
		float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, cache->indexA[0]);
		float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, cache->indexB[0]);
		float2 pointA = b2clMul_Transform(&xfA, localPointA);
		float2 pointB = b2clMul_Transform(&xfB, localPointB);
		sf->m_axis = pointB - pointA;
		float s = b2clNormalize(&sf->m_axis);
		return s;
	}
	else if (cache->indexA[0] == cache->indexA[1])
	{
		// Two points on B and one on A.
		sf->m_type = e_faceB;
		float2 localPointB1 = b2clDistanceProxyGetVertex(proxyB, cache->indexB[0]);
		float2 localPointB2 = b2clDistanceProxyGetVertex(proxyB, cache->indexB[1]);

		sf->m_axis = b2clCross_VS(localPointB2 - localPointB1, 1.0f);
		sf->m_axis = normalize(sf->m_axis);
		float2 normal = b2clMul_Rotate(xfB.q, sf->m_axis);

		sf->m_localPoint = 0.5f * (localPointB1 + localPointB2);
		float2 pointB = b2clMul_Transform(&xfB, sf->m_localPoint);

		float2 localPointA = b2clDistanceProxyGetVertex(proxyA, cache->indexA[0]);
		float2 pointA = b2clMul_Transform(&xfA, localPointA);

		float s = b2clDot(pointA - pointB, normal);
		if (s < 0.0f)
		{
			sf->m_axis = -sf->m_axis;
			s = -s;
		}
		return s;
	}
	else
	{
		// Two points on A and one or two points on B.
		sf->m_type = e_faceA;
		float2 localPointA1 = b2clDistanceProxyGetVertex(sf->m_proxyA, cache->indexA[0]);
		float2 localPointA2 = b2clDistanceProxyGetVertex(sf->m_proxyA, cache->indexA[1]);
			
		sf->m_axis = b2clCross_VS(localPointA2 - localPointA1, 1.0f);
		sf->m_axis = normalize(sf->m_axis);
		float2 normal = b2clMul_Rotate(xfA.q, sf->m_axis);

		sf->m_localPoint = 0.5f * (localPointA1 + localPointA2);
		float2 pointA = b2clMul_Transform(&xfA, sf->m_localPoint);

		float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, cache->indexB[0]);
		float2 pointB = b2clMul_Transform(&xfB, localPointB);

		float s = b2clDot(pointB - pointA, normal);
		if (s < 0.0f)
		{
			sf->m_axis = -sf->m_axis;
			s = -s;
		}
		return s;
	}
}

float b2clSeparationFunctionFindMinSeparation(b2clSeparationFunction* sf, int* indexA, int* indexB, float t, int numLoops)
{
	b2clTransform xfA, xfB;
	
	b2clSweepGetTransform(&sf->m_sweepA, &xfA, t);
	b2clSweepGetTransform(&sf->m_sweepB, &xfB, t);

	
	switch (sf->m_type)
	{
	case e_points:
		{
			float2 axisA = b2clMulT_Rotate(xfA.q,  sf->m_axis);
			float2 axisB = b2clMulT_Rotate(xfB.q, -sf->m_axis);

			*indexA = b2clDistanceProxyGetSupport(sf->m_proxyA, axisA);
			*indexB = b2clDistanceProxyGetSupport(sf->m_proxyB, axisB);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, *indexA);
			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, *indexB);
				
			float2 pointA = b2clMul_Transform(&xfA, localPointA);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);

			float separation = b2clDot(pointB - pointA, sf->m_axis);
			return separation;
		}

	case e_faceA:
		{
		   // if (numLoops == 36) {
		//		printf ("got in numLoops == 36 \n"); 
		//	}
		 //   if (numLoops == 36) return ; 
			float2 normal = b2clMul_Rotate(xfA.q, sf->m_axis);
			float2 pointA = b2clMul_Transform(&xfA, sf->m_localPoint);

			float2 axisB = b2clMulT_Rotate(xfB.q, -normal);
				
			*indexA = -1;
			*indexB = b2clDistanceProxyGetSupport(sf->m_proxyB, axisB);

			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, *indexB);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);

			float separation = b2clDot(pointB - pointA, normal);
			return separation;
		}

	case e_faceB:
		{
		    
			float2 normal = b2clMul_Rotate(xfB.q, sf->m_axis);
		//	if (numLoops == 36) {
		//	printf ("sf->m_localPoint for 36: %f \n", sf->m_localPoint.y); 
		//	}
		//	if (numLoops == 36) return ; 
			float2 pointB = b2clMul_Transform(&xfB, sf->m_localPoint);
			
			float2 axisA = b2clMulT_Rotate(xfA.q, -normal);

			*indexB = -1;
			*indexA = b2clDistanceProxyGetSupport(sf->m_proxyA, axisA);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, *indexA);
			float2 pointA = b2clMul_Transform(&xfA, localPointA);

			float separation = b2clDot(pointA - pointB, normal);
			return separation;
		}

	default:
		*indexA = -1;
		*indexB = -1;
		return 0.0f;
	}
}

float b2clSeparationFunctionEvaluate(b2clSeparationFunction* sf, int indexA, int indexB, float t)
{
	b2clTransform xfA, xfB;
	b2clSweepGetTransform(&sf->m_sweepA, &xfA, t);
	b2clSweepGetTransform(&sf->m_sweepB, &xfB, t);

	switch (sf->m_type)
	{
	case e_points:
		{
			float2 axisA = b2clMulT_Rotate(xfA.q,  sf->m_axis);
			float2 axisB = b2clMulT_Rotate(xfB.q, -sf->m_axis);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, indexA);
			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, indexB);

			float2 pointA = b2clMul_Transform(&xfA, localPointA);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);
			float separation = b2clDot(pointB - pointA, sf->m_axis);

			return separation;
		}

	case e_faceA:
		{
			float2 normal = b2clMul_Rotate(xfA.q, sf->m_axis);
			float2 pointA = b2clMul_Transform(&xfA, sf->m_localPoint);

			float2 axisB = b2clMulT_Rotate(xfB.q, -normal);

			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, indexB);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);

			float separation = b2clDot(pointB - pointA, normal);
			return separation;
		}

	case e_faceB:
		{
			float2 normal = b2clMul_Rotate(xfB.q, sf->m_axis);
			float2 pointB = b2clMul_Transform(&xfB, sf->m_localPoint);

			float2 axisA = b2clMulT_Rotate(xfA.q, -normal);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, indexA);
			float2 pointA = b2clMul_Transform(&xfA, localPointA);

			float separation = b2clDot(pointA - pointB, normal);
			return separation;
		}

	default:
		return 0.0f;
	}
}

float SimplexGetMetric(b2clSimplex* simplex)
{
	switch (simplex->m_count)
	{
	case 0:
		return 0.0;

	case 1:
		return 0.0f;

	case 2:
		return b2clDistance(simplex->m_v1.w, simplex->m_v2.w);

	case 3:
		return b2clCross_VV(simplex->m_v2.w - simplex->m_v1.w, simplex->m_v3.w - simplex->m_v1.w);

	default:
		return 0.0f;
	}
}

void SimplexReadCache(b2clSimplex* simplex, const b2clSimplexCache* cache,
					const b2clDistanceProxy* proxyA, const b2clTransform* transformA,
					const b2clDistanceProxy* proxyB, const b2clTransform* transformB)
{
		
	// Copy data from cache.
	simplex->m_count = cache->count;
	b2clSimplexVertex* vertices = &simplex->m_v1;
	for (int i = 0; i < simplex->m_count; ++i)
	{
		b2clSimplexVertex* v = vertices + i;
		v->indexA = cache->indexA[i];
		v->indexB = cache->indexB[i];
		float2 wALocal = proxyA->m_vertices[v->indexA];
		float2 wBLocal = proxyB->m_vertices[v->indexB];
		v->wA = b2clMul_Transform(transformA, wALocal);
		v->wB = b2clMul_Transform(transformB, wBLocal);
		v->w = v->wB - v->wA;
		v->a = 0.0f;
	}

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if (simplex->m_count > 1)
	{
		float metric1 = cache->metric;
		float metric2 = SimplexGetMetric(simplex);
		if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < b2_epsilon)
		{
			// Reset the simplex.
			simplex->m_count = 0;
		}
	}

	// If the cache is empty or invalid ...
	if (simplex->m_count == 0)
	{
		b2clSimplexVertex* v = vertices + 0;
		v->indexA = 0;
		v->indexB = 0;
		float2 wALocal = proxyA->m_vertices[0];
		float2 wBLocal = proxyB->m_vertices[0];
		v->wA = b2clMul_Transform(transformA, wALocal);
		v->wB = b2clMul_Transform(transformB, wBLocal);
		v->w = v->wB - v->wA;
		simplex->m_count = 1;
	}
}

void SimplexWriteCache(b2clSimplex* simplex, b2clSimplexCache* cache)
{
	cache->metric = SimplexGetMetric(simplex);
	cache->count = (short)(simplex->m_count);
	const b2clSimplexVertex* vertices = &simplex->m_v1;
	for (int i = 0; i < simplex->m_count; ++i)
	{
		cache->indexA[i] = (unsigned char)(vertices[i].indexA);
		cache->indexB[i] = (unsigned char)(vertices[i].indexB);
	}
}

float2 SimplexGetSearchDirection(b2clSimplex* simplex)
{
	switch (simplex->m_count)
	{
	case 1:
		return -simplex->m_v1.w;

	case 2:
		{
			float2 e12 = simplex->m_v2.w - simplex->m_v1.w;
			float sgn = b2clCross_VV(e12, -simplex->m_v1.w);
			if (sgn > 0.0f)
			{
				// Origin is left of e12.
				return b2clCross_SV(1.0f, e12);
			}
			else
			{
				// Origin is right of e12.
				return b2clCross_VS(e12, 1.0f);
			}
		}

	default:
		return (float2)(0.0f, 0.0f);
	}
}

float2 SimplexGetClosestPoint(b2clSimplex* simplex)
{
	switch (simplex->m_count)
	{
	case 0:
		return (float2)(0.0f, 0.0f);

	case 1:
		return simplex->m_v1.w;

	case 2:
		return simplex->m_v1.a * simplex->m_v1.w + simplex->m_v2.a * simplex->m_v2.w;

	case 3:
		return (float2)(0.0f, 0.0f);

	default:
		return (float2)(0.0f, 0.0f);
	}
}

void SimplexGetWitnessPoints(b2clSimplex* simplex, float2* pA, float2* pB)
{
	switch (simplex->m_count)
	{
	case 0:
		break;

	case 1:
		*pA = simplex->m_v1.wA;
		*pB = simplex->m_v1.wB;
		break;

	case 2:
		*pA = simplex->m_v1.a * simplex->m_v1.wA + simplex->m_v2.a * simplex->m_v2.wA;
		*pB = simplex->m_v1.a * simplex->m_v1.wB + simplex->m_v2.a * simplex->m_v2.wB;
		break;

	case 3:
		*pA = simplex->m_v1.a * simplex->m_v1.wA + simplex->m_v2.a * simplex->m_v2.wA + simplex->m_v3.a * simplex->m_v3.wA;
		*pB = *pA;
		break;

	default:
		break;
	}
}

void SimplexSolve2(b2clSimplex* simplex)
{
	float2 w1 = simplex->m_v1.w;
	float2 w2 = simplex->m_v2.w;
	float2 e12 = w2 - w1;

	// w1 region
	float d12_2 = -b2clDot(w1, e12);
	if (d12_2 <= 0.0f)
	{
		// a2 <= 0, so we clamp it to 0
		simplex->m_v1.a = 1.0f;
		simplex->m_count = 1;
		return;
	}

	// w2 region
	float d12_1 = b2clDot(w2, e12);
	if (d12_1 <= 0.0f)
	{
		// a1 <= 0, so we clamp it to 0
		simplex->m_v2.a = 1.0f;
		simplex->m_count = 1;
		simplex->m_v1 = simplex->m_v2;
		return;
	}

	// Must be in e12 region.
	float inv_d12 = 1.0f / (d12_1 + d12_2);
	simplex->m_v1.a = d12_1 * inv_d12;
	simplex->m_v2.a = d12_2 * inv_d12;
	simplex->m_count = 2;
}

void SimplexSolve3(b2clSimplex* simplex)
{
	float2 w1 = simplex->m_v1.w;
	float2 w2 = simplex->m_v2.w;
	float2 w3 = simplex->m_v3.w;

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	float2 e12 = w2 - w1;
	float w1e12 = b2clDot(w1, e12);
	float w2e12 = b2clDot(w2, e12);
	float d12_1 = w2e12;
	float d12_2 = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	float2 e13 = w3 - w1;
	float w1e13 = b2clDot(w1, e13);
	float w3e13 = b2clDot(w3, e13);
	float d13_1 = w3e13;
	float d13_2 = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	float2 e23 = w3 - w2;
	float w2e23 = b2clDot(w2, e23);
	float w3e23 = b2clDot(w3, e23);
	float d23_1 = w3e23;
	float d23_2 = -w2e23;
	
	// Triangle123
	float n123 = b2clCross_VV(e12, e13);

	float d123_1 = n123 * b2clCross_VV(w2, w3);
	float d123_2 = n123 * b2clCross_VV(w3, w1);
	float d123_3 = n123 * b2clCross_VV(w1, w2);

	// w1 region
	if (d12_2 <= 0.0f && d13_2 <= 0.0f)
	{
		simplex->m_v1.a = 1.0f;
		simplex->m_count = 1;
		return;
	}

	// e12
	if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
	{
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		simplex->m_v1.a = d12_1 * inv_d12;
		simplex->m_v2.a = d12_2 * inv_d12;
		simplex->m_count = 2;
		return;
	}

	// e13
	if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
	{
		float inv_d13 = 1.0f / (d13_1 + d13_2);
		simplex->m_v1.a = d13_1 * inv_d13;
		simplex->m_v3.a = d13_2 * inv_d13;
		simplex->m_count = 2;
		simplex->m_v2 = simplex->m_v3;
		return;
	}

	// w2 region
	if (d12_1 <= 0.0f && d23_2 <= 0.0f)
	{
		simplex->m_v2.a = 1.0f;
		simplex->m_count = 1;
		simplex->m_v1 = simplex->m_v2;
		return;
	}

	// w3 region
	if (d13_1 <= 0.0f && d23_1 <= 0.0f)
	{
		simplex->m_v3.a = 1.0f;
		simplex->m_count = 1;
		simplex->m_v1 = simplex->m_v3;
		return;
	}

	// e23
	if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
	{
		float inv_d23 = 1.0f / (d23_1 + d23_2);
		simplex->m_v2.a = d23_1 * inv_d23;
		simplex->m_v3.a = d23_2 * inv_d23;
		simplex->m_count = 2;
		simplex->m_v1 = simplex->m_v3;
		return;
	}

	// Must be in triangle123
	float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
	simplex->m_v1.a = d123_1 * inv_d123;
	simplex->m_v2.a = d123_2 * inv_d123;
	simplex->m_v3.a = d123_3 * inv_d123;
	simplex->m_count = 3;
}

// Compute the closest points between two shapes. Supports any combination of:
// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
// On the first call set b2SimplexCache.count to zero.
void b2clShapeDistance(b2clDistanceOutput* output, b2clSimplexCache* cache, const b2clDistanceInput* input)
{

    
	const b2clDistanceProxy* proxyA = &input->proxyA;
	const b2clDistanceProxy* proxyB = &input->proxyB;

	b2clTransform transformA = input->transformA;
	b2clTransform transformB = input->transformB;

	// Initialize the simplex.
	b2clSimplex simplex;
	SimplexReadCache(&simplex, cache, proxyA, &transformA, proxyB, &transformB);

	// Get simplex vertices as an array.
	b2clSimplexVertex* vertices = &simplex.m_v1;
	const int k_maxIters = 20;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	int saveA[3], saveB[3];
	int saveCount = 0;

	float2 closestPoint = SimplexGetClosestPoint(&simplex);
	float distanceSqr1 = b2clLengthSquared(closestPoint);
	float distanceSqr2 = distanceSqr1;

	// Main iteration loop.
	int iter = 0;
	while (iter < k_maxIters)
	{
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.m_count;
		for (int i = 0; i < saveCount; ++i)
		{
			saveA[i] = vertices[i].indexA;
			saveB[i] = vertices[i].indexB;
		}

		switch (simplex.m_count)
		{
		case 1:
			break;

		case 2:
			SimplexSolve2(&simplex);
			break;

		case 3:
			SimplexSolve3(&simplex);
			break;

		default:
			break;
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.m_count == 3)
		{
			break;
		}

		// Compute closest point.
		float2 p = SimplexGetClosestPoint(&simplex);
		distanceSqr2 = b2clLengthSquared(p);

		// Ensure progress
		if (distanceSqr2 >= distanceSqr1)
		{
			//break;
		}
		distanceSqr1 = distanceSqr2;

		// Get search direction.
		float2 d = SimplexGetSearchDirection(&simplex);

		// Ensure the search direction is numerically fit.
		if (b2clLengthSquared(d) < b2_epsilon * b2_epsilon)
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		b2clSimplexVertex* vertex = vertices + simplex.m_count;
		vertex->indexA = b2clDistanceProxyGetSupport(proxyA, b2clMulT_Rotate(transformA.q, -d));
		vertex->wA = b2clMul_Transform(&transformA, (b2clDistanceProxyGetVertex(proxyA, vertex->indexA)));

		//float2 wBLocal;
		vertex->indexB = b2clDistanceProxyGetSupport(proxyB, b2clMulT_Rotate(transformB.q, d));
		vertex->wB = b2clMul_Transform(&transformB, (b2clDistanceProxyGetVertex(proxyB, vertex->indexB)));
		vertex->w = vertex->wB - vertex->wA;

		// Iteration count is equated to the number of support point calls.
		++iter;

		// Check for duplicate support points. This is the main termination criteria.
		bool duplicate = false;
		for (int i = 0; i < saveCount; ++i)
		{
			if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is ok and needed.
		++simplex.m_count;
	}
	
	// Prepare output.
	SimplexGetWitnessPoints(&simplex, &output->pointA, &output->pointB);
	//float dis = b2clDistance(output->pointA, output->pointB);
	output->ndistance = b2clDistance(output->pointA, output->pointB);
	output->iterations = iter;
	
	// Cache the simplex.
	SimplexWriteCache(&simplex, cache);
	
	// Apply radii if requested.
	if (input->useRadii)
	{
		float rA = proxyA->m_radius;
		float rB = proxyB->m_radius;

		if (output->ndistance > rA + rB && output->ndistance > b2_epsilon)
		{
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output->ndistance -= rA + rB;
			float2 normal = normalize(output->pointB - output->pointA);
			output->pointA += rA * normal;
			output->pointB -= rB * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			float2 p = 0.5f * (output->pointA + output->pointB);
			output->pointA = p;
			output->pointB = p;
			output->ndistance = 0.0f;
		}
	}
}

void b2clTimeOfImpact(b2clTOIOutput* output, const b2clTOIInput* input, int numLoops)
{
	output->state = e_unknown;
	output->t = input->tMax;

	b2clDistanceProxy proxyA = input->proxyA;
	b2clDistanceProxy proxyB = input->proxyB;

	b2clSweep sweepA = input->sweepA;
	b2clSweep sweepB = input->sweepB;

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	b2clSweepNormalize(&sweepA);
	b2clSweepNormalize(&sweepB);

	float tMax = input->tMax;

	float totalRadius = proxyA.m_radius + proxyB.m_radius;
	float target = max(b2_linearSlop, totalRadius - 3.0f * b2_linearSlop);
	float tolerance = 0.25f * b2_linearSlop;

	float t1 = 0.0f;
	const int k_maxIterations = 20;	// TODO_ERIN b2Settings
	int iter = 0;

	// Prepare input for distance query.
	b2clSimplexCache cache;
	cache.count = 0;
	b2clDistanceInput distanceInput;
	distanceInput.proxyA = proxyA;
	distanceInput.proxyB = proxyB;
	distanceInput.useRadii = false;


	

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	//for(;;)
	//Warning: I can not use for(;;). Otherwise, clBuildProgram will return error! 
	for (int i = 0 ; i < 100 ; i ++ )
	//while(1)
	//for (;;)
	{
		b2clTransform xfA, xfB;
		b2clSweepGetTransform(&sweepA, &xfA, t1);
		b2clSweepGetTransform(&sweepB, &xfB, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.transformA = xfA;
		distanceInput.transformB = xfB;
		b2clDistanceOutput distanceOutput;

		b2clShapeDistance(&distanceOutput, &cache, &distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.ndistance <= 0.0f)
		{
			// Failure!
			output->state = e_overlapped;
			output->t = 0.0f;
			break;
		}

		if (distanceOutput.ndistance < target + tolerance)
		{
			// Victory!
			output->state = e_touching;
			output->t = t1;
			break;
		}

		// Initialize the separating axis.
		b2clSeparationFunction fcn;
		b2clSeparationFunctionInitialize(&fcn, &cache, &proxyA, &sweepA, &proxyB, &sweepB, t1);

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		bool done = false;
		float t2 = tMax;
		int pushBackIter = 0;
		
		for (;;)
		{
			// Find the deepest point at t2. Store the witness point indices.
			int indexA, indexB;
			//if (numLoops == 36 && ) return ; 
			float s2 = b2clSeparationFunctionFindMinSeparation(&fcn, &indexA, &indexB, t2, numLoops);

			// Is the final configuration separated?
			if (s2 > target + tolerance)
			{
				// Victory!
				output->state = e_separated;
				output->t = tMax;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			float s1 = b2clSeparationFunctionEvaluate(&fcn, indexA, indexB, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < target - tolerance)
			{
				output->state = e_failed;
				output->t = t1;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= target + tolerance)
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output->state = e_touching;
				output->t = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			int rootIterCount = 0;
			float a1 = t1, a2 = t2;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				float t;
				if (rootIterCount & 1)
				{
					// Secant rule to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
				}
				else
				{
					// Bisection to guarantee progress.
					t = 0.5f * (a1 + a2);
				}

				float s = b2clSeparationFunctionEvaluate(&fcn, indexA, indexB, t);

				if (b2clAbs(s - target) < tolerance)
				{
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target)
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}

				++rootIterCount;

				if (rootIterCount == 50)
				{
					break;
				}
			}

			++pushBackIter;

			if (pushBackIter == b2cl_maxPolygonVertices)
			{
				break;
			}
		}

		++iter;

		if (done)
		{
			break;
		}

		if (iter == k_maxIterations)
		{
			// Root finder got stuck. Semi-victory.
			output->state = e_failed;
			output->t = t1;
			break;
		}
	}
	//printf ("output.t: %f \n", output->t); 
}

__kernel void b2clComputeTOI(const __global int4* globalIndices,
	int numContacts,
	const __global b2clBodyStatic* bodyStaticListBuffer,
	const __global b2clBodyDynamic* bodyDynamicListBuffer,
	const __global b2clPolygonShape* polyGlobal,
	__global unsigned int* contactTOIs,
	__global int* contactIndices,
	int resetAlpha0,
	__global int* toiTimesBuffer,
	int numLoops 
	)
{

 
	int i = get_global_id(0);

	if (i >= numContacts)
		return;

	float alpha = 1.0f;

	contactIndices[i] = i;
	contactTOIs[i] = FloatFlip(alpha);

	// TODO: Disabled contact checking
	/*
	// Is this contact disabled?
	if (c->IsEnabled() == false)
	{
		continue;
	}
	*/

	
	int4 currentIndices = globalIndices[i];

	b2clBodyStatic bA = bodyStaticListBuffer[currentIndices.z];
	b2clBodyStatic bB = bodyStaticListBuffer[currentIndices.w];

	b2clPolygonShape shapeA = polyGlobal[currentIndices.x];
	b2clPolygonShape shapeB = polyGlobal[currentIndices.y];

	// Is there a sensor?
	if (shapeA.m_bIsSensor || shapeB.m_bIsSensor)
		return;
   if (toiTimesBuffer[i] > 8 ) return ; 
	int typeA = bA.m_type;
	int typeB = bB.m_type;

	//printf ("Contact id: %d, TypeA: %d TypeB: %d \n", i, typeA, typeB);
	
	// Is at least one body active (awake and dynamic or kinematic)?
	if (typeA == 0 && typeB == 0) // 0 : b2_staticBody
	{
		return;
	}
	 
	bool collideA = bA.m_bIsBullet || typeA != 2; // 2 : b2_dynamicBody
	bool collideB = bB.m_bIsBullet || typeB != 2; // 2 : b2_dynamicBody

	// Are these two non-bullet dynamic bodies?
	if (collideA == false && collideB == false)
	{
		return;
	}

	b2clBodyDynamic bdA = bodyDynamicListBuffer[currentIndices.z];
	b2clBodyDynamic bdB = bodyDynamicListBuffer[currentIndices.w];

	//printf ("Contact id: %d: Index A: %d , Index B: %d \n", i , currentIndices.z , currentIndices.w); 

	if (resetAlpha0)
	{	
		bdA.m_sweep.alpha0 = 0.0f;
		bdB.m_sweep.alpha0 = 0.0f;
	}

	// Compute the TOI for this contact.
	// Put the sweeps onto the same time interval.
	float alpha0 = bdA.m_sweep.alpha0;


	if (bdA.m_sweep.alpha0 < bdB.m_sweep.alpha0)
	{
		alpha0 = bdB.m_sweep.alpha0;
		b2clSweepAdvance(&bdA.m_sweep, alpha0);
	}
	else if (bdB.m_sweep.alpha0 < bdA.m_sweep.alpha0)
	{
		alpha0 = bdA.m_sweep.alpha0;
		b2clSweepAdvance(&bdB.m_sweep, alpha0);
	}


	// Compute the time of impact in interval [0, minTOI]
	b2clTOIInput input;
	b2clDistanceProxySet(&input.proxyA, &shapeA);
	b2clDistanceProxySet(&input.proxyB, &shapeB);
	input.sweepA = bdA.m_sweep;
	input.sweepB = bdB.m_sweep;
	input.tMax = 1.0f;

	float test1 = bdA.m_sweep.c.y ;
	float test2 = bdB.m_sweep.c.y ;  
	
	//if (numLoops == 36 && i == 0 ) return ; 

	b2clTOIOutput output;
	b2clTimeOfImpact(&output, &input , numLoops) ; 				

	
	// Beta is the fraction of the remaining portion of the .

	float beta = output.t;
		//printf ("beta %f, \n",  beta); 
	if (output.state == e_touching)
	{
		alpha = min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
		//printf ("MyAlpha %f \n", alpha); 
		//printf ("if alpha0 %f, \n",  alpha0); 
	}
	else
	{
		alpha = 1.0f;
	}
	//printf ("Contact id: %d: Index A: %d , alpha: %f \n", i , currentIndices.z , alpha); 
	 
	contactTOIs[i] = FloatFlip(alpha);
}





void b2clOnePairCollidePolygons(b2clManifold* manifold, const b2clPolygonShape* polyA, const b2clTransform* xfA, const b2clPolygonShape* polyB, const b2clTransform* xfB){
/*
	manifold->pointCount = 0 ;
	// Not sure if I need them yet.  
	//manifold.points[0].normalImpulse = 0;
    //manifold.points[0].tangentImpulse = 0;
    //manifold.points[1].normalImpulse = 0;
    //manifold.points[1].tangentImpulse = 0;
	float totalRadius = polyA->m_radius + polyB->m_radius; 
	int edgeA = 0 ; 
	float separationA = b2clFindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
	if (separationA > totalRadius) return ; 
	int edgeB = 0 ; 
	float separationB = b2clFindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
	if (separationB > totalRadius) return;

	const b2clPolygonShape* poly1;	// reference polygon
	const b2clPolygonShape* poly2;	// incident polygon
	const b2clTransform *xf1;
	const b2clTransform *xf2;
	int edge1;		// reference edge
	uint flip;
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;
	if (separationB > k_relativeTol * separationA + k_absoluteTol)
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		manifold->type = 2;
		flip = 1;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		manifold->type = 1;
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
		return;
	}

	// Clip to negative box side 1
	np = b2clClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < 2)
	{
		return;
	}
    manifold->localNormal = localNormal;
	manifold->localPoint = planePoint;
	int pointCount = 0;
	for (int k = 0; k < b2cl_maxManifoldPoints; ++k)
	{
		float separation = dot(normal, clipPoints2[k].v) - frontOffset;

		if (separation <= totalRadius)
		{
			b2clManifoldPoint* cp = manifold->points + pointCount;
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

	manifold->pointCount = pointCount;
*/
}

void ContactEvaluate (b2clManifold* manifold, const b2clPolygonShape* polyA, const b2clTransform* xfA, const b2clPolygonShape* polyB, const b2clTransform* xfB) {

    int contactType = manifold->type ; 
	switch (contactType)
			{
			case 0: // circle-circle
				//collideKernel = collideCirclesKernel;
				break;
			case 1: // circle-polygon
				//collideKernel = collidePolygonAndCircleKernel;
				break;
			case 2: // polygon-polygon
				b2clOnePairCollidePolygons ( manifold, polyA, xfA, polyB, xfB ) ; 
				break;
			case 3: // edge-circle
				//collideKernel = collideEdgeAndCircleKernel;
				break;
			case 4: // edge-polygon
				//collideKernel = collideEdgeAndPolygonKernel;
				break;
			default:
				//printf("Error! OpenCL: ContactEvaluate!\n");
				return; 
			}
}
void ContactUpdate  (b2clManifold* manifold, const b2clPolygonShape* polyA, const b2clTransform* xfA, const b2clPolygonShape* polyB, const b2clTransform* xfB) {
	b2clManifold oldManifold = *manifold; 
	// Re-enable this contact ; Do I need to do this?
	bool touching = false ; 
	// wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag; Do I need this? 
	//Not implemented for sensor
	//bool sensorA = m_fixtureA->IsSensor(); bool sensorB = m_fixtureB->IsSensor(); bool sensor = sensorA || sensorB; 
	//if (sensor){}
	ContactEvaluate ( manifold, polyA, xfA, polyB, xfB ) ;  
	touching = manifold->pointCount > 0 ; 
	for (int i = 0; i < manifold->pointCount; ++i)
	{
		b2clManifoldPoint* mp2 = manifold->points + i;
		mp2->normalImpulse = 0.0f;
		mp2->tangentImpulse = 0.0f;
		b2clContactID id2 = mp2->id;

		for (int j = 0; j < oldManifold.pointCount; ++j)
		{
			b2clManifoldPoint* mp1 = oldManifold.points + j;
			if (mp1->id.key == id2.key)
			{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					break;
			}
		}
	}




}
void b2clBodySynchronizeTransform ( b2clTransform* xf, b2clSweep* sweep ) {
	float sina, cosa;
	sina = sincos(sweep->a, &cosa);
	xf->q.x = sina;
	xf->q.y = cosa;
	//xf->q.x = sin (sweep->a);
	//xf->q.y = cos (sweep->a);
	//xf->q.y = cos_wrapper(sweep->a);
	xf->p = sweep->c - b2clMul_Rotate (xf->q, sweep->localCenter); 
}


//void b2clAdvanceOneBody 

__kernel void b2clAdvanceBodiesKernel(const __global int4* globalIndices,
	int numContacts,
	const __global b2clBodyStatic* bodyStaticListBuffer,
	__global b2clBodyDynamic* bodyDynamicListBuffer,
	const __global b2clPolygonShape* polyGlobal,
	__global b2clTransform* xfGlobal,
	__global b2clManifold* manifoldList ,  
	__global unsigned int* contactTOIs,
	__global int* contactIndices){

	int idxContact = get_global_id(0);

	// Currently, we only handle one contact.
	if (idxContact >= 1) return;
	idxContact = contactIndices[idxContact] ; 
	float minAlpha = IFloatFlip (contactTOIs[idxContact]);  
	int4 currentIndices = globalIndices[idxContact] ; 
	b2clBodyDynamic bA = bodyDynamicListBuffer[currentIndices.z];
	b2clBodyDynamic bB = bodyDynamicListBuffer[currentIndices.w];
	b2clPolygonShape polyA = polyGlobal[currentIndices.z];
	b2clPolygonShape polyB = polyGlobal[currentIndices.w]; 
	b2clManifold manifold = manifoldList[idxContact]; 

	//b2clSweep backupSweepA = bA.m_sweep;
	//b2clSweep backupSweepB = bB.m_sweep; 
	b2clTransform xfA, xfB ; 

	b2clBodySweepAdvance (&bA, &xfA, minAlpha);
	b2clBodySweepAdvance (&bB, &xfB, minAlpha);

	ContactUpdate ( &manifold, &polyA, &xfA, &polyB, &xfB ) ; 
	if (manifold.pointCount == 0) {
		//bA.m_sweep = backupSweepA ; 
		//bB.m_sweep = backupSweepB ; 
		//b2clBodySynchronizeTransform ( &xfA, &bA.m_sweep) ; 
		//b2clBodySynchronizeTransform ( &xfB, &bB.m_sweep);
		return ; 
	}

    bodyDynamicListBuffer[currentIndices.z] = bA;
	bodyDynamicListBuffer[currentIndices.w] = bB;
	xfGlobal[currentIndices.z] = xfA ; 
	xfGlobal[currentIndices.w] = xfB; 

	//minContact->Update(m_contactManager.m_contactListener);

}




__kernel void b2clMarkConnectedContactsKernel(const __global int4* globalIndices,
	int numContacts,
	const __global b2clBodyStatic* bodyStaticListBuffer,
	__global b2clBodyDynamic* bodyDynamicListBuffer,
	const __global b2clPolygonShape* polyGlobal,
	__global b2clTransform* xfGlobal,
	__global b2clManifold* manifoldList ,  
	__global unsigned int* contactTOIs,
	__global int* contactIndices,
	__global int* isConnectedBuffer, 
	int seedIndex ) {

	int idxContact = get_global_id(0) ; 
	if (idxContact >= numContacts) return ; 
	idxContact = contactIndices[idxContact]; 
	seedIndex = contactIndices[seedIndex]; 
	if (idxContact == seedIndex) {isConnectedBuffer[idxContact] = 1 ; return ; }
	int4 seedIndices = globalIndices[seedIndex];
	int4 currentIndices = globalIndices[idxContact]; 

	bool findA = false; bool findB = false ; int idxShare, idxOther ; 
	if (seedIndices.z == currentIndices.z || seedIndices.z == currentIndices.w) findA = true ;  
	if (seedIndices.w == currentIndices.w || seedIndices.w == currentIndices.w) findB = true ;
	if (findA == false && findB == false ) return ; 
    if (findA == true) {idxShare = seedIndices.z; idxOther = (seedIndices.z == currentIndices.z)? currentIndices.w: currentIndices.z;}
	else {idxShare = seedIndices.w; idxOther = (seedIndices.w == currentIndices.w)? currentIndices.z:currentIndices.w; }

	// Has this contact already been added to the island? 
	if (isConnectedBuffer[idxContact] == 1) return ; 

	


	b2clBodyStatic staticShare = bodyStaticListBuffer[idxShare];
	b2clBodyStatic staticOther = bodyStaticListBuffer[idxOther];

	// Only add static, kinematic, or bullet bodies.
	// b2_dynamicbody = 2
	if (staticOther.m_type == 2 && staticShare.m_bIsBullet == 0 && staticOther.m_bIsBullet == 0 ) return ; 
	//To do: skip Sensors: 

	// Tentatively advance the body to the TOI.
	float minAlpha = IFloatFlip (contactTOIs[seedIndex]);  
	b2clBodyDynamic bA = bodyDynamicListBuffer[currentIndices.z];
	b2clBodyDynamic bB = bodyDynamicListBuffer[currentIndices.w];
	b2clTransform xfA = xfGlobal[currentIndices.z];
	b2clTransform xfB = xfGlobal[currentIndices.w];
	b2clPolygonShape polyA = polyGlobal[currentIndices.z];
	b2clPolygonShape polyB = polyGlobal[currentIndices.w]; 
	b2clManifold manifold = manifoldList[idxContact];
	
    b2clBodyDynamic* bodyOther; b2clTransform* xfOther;
	if (idxOther == currentIndices.z) {bodyOther = &bA; xfOther = &xfA;}
	else {bodyOther =&bB; xfOther = &xfB;}; 
	
	b2clBodySweepAdvance (bodyOther, xfOther, minAlpha);
	ContactUpdate ( &manifold, &polyA, &xfA, &polyB, &xfB ) ; 
	if (manifold.pointCount == 0) {
		return ; 
	}
    bodyDynamicListBuffer[idxOther] = *bodyOther;
	xfGlobal[idxOther] = *xfOther ;  
	isConnectedBuffer[idxContact] = 1 ; 


}
#endif


__kernel void b2clsyncMovedBodyKernel (
 const __global float* movedBodyBuffer, 
int numBodies, 
__global b2clTransform* xfGlobal,
__global clb2Velocity* velocityBuffer,
__global clb2Position* positionBuffer
)
{
  int idxThread = get_global_id (0);
  if (idxThread >= numBodies) return; 

  b2clTransform xf;
  clb2Velocity vel;
  clb2Position pos;

  int offset = idxThread* 11 ; 
  //float testFloat = movedBodyBuffer[offset+0]; 
    
  int idxBody = (int) movedBodyBuffer[offset+0]; 
  vel.vx = movedBodyBuffer[offset+1];
  vel.vy = movedBodyBuffer[offset+2];
  vel.w = movedBodyBuffer[offset+3];
  pos.cx = movedBodyBuffer[offset+4];
  pos.cy = movedBodyBuffer[offset+5];
  pos.a  = movedBodyBuffer[offset+6];
  xf.p.x = movedBodyBuffer[offset+7];
  xf.p.y = movedBodyBuffer[offset+8];
  xf.q.x = movedBodyBuffer[offset+9];
  xf.q.y = movedBodyBuffer[offset+10];
 

  xfGlobal[idxBody] = xf ; 
  velocityBuffer[idxBody] = vel ; 
  positionBuffer[idxBody] = pos ; 

}