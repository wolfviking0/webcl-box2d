
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

typedef struct 
{
	float m_min[2];
	float m_max[2];
	uchar m_sType;
	uchar m_bType;
	//uchar mask[2];
} b2clAABB;

//http://stereopsis.com/radix.html

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

bool TestAabbAgainstAabb2(const b2clAABB* aabb1, __local const b2clAABB* aabb2)
{
	bool overlap = true;
	overlap = (aabb1->m_min[0] > aabb2->m_max[0] || aabb1->m_max[0] < aabb2->m_min[0]) ? false : overlap;
	overlap = (aabb1->m_min[1] > aabb2->m_max[1] || aabb1->m_max[1] < aabb2->m_min[1]) ? false : overlap;
	return overlap;
}

//bool TestAabbAgainstAabb2GlobalGlobal(__global const b2clAABB* aabb1, __global const b2clAABB* aabb2)
bool TestAabbAgainstAabb2GlobalGlobal(const b2clAABB* aabb1, __global const b2clAABB* aabb2)
{
	bool overlap = true;
	//if ((aabb1->mask[0] ^ aabb2->mask[0]) && (aabb1->mask[0] ^ aabb2->mask[1]) &&
	//	(aabb1->mask[1] ^ aabb2->mask[0]) && (aabb1->mask[1] ^ aabb2->mask[1]))
	//	return false;
	overlap = (aabb1->m_min[0] > aabb2->m_max[0] || aabb1->m_max[0] < aabb2->m_min[0]) ? false : overlap;
	overlap = (aabb1->m_min[1] > aabb2->m_max[1] || aabb1->m_max[1] < aabb2->m_min[1]) ? false : overlap;
	return overlap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ShouldCollideJoints
//
// Determines whether bodies A and B should collide or not.
// If A and B are connected by a joint which does not allow collisions between connected bodies,
// this function returns false.
////////////////////////////////////////////////////////////////////////////////////////////////////
bool ShouldCollideJoints(int bodyA, int bodyB, const __global int* connectedBodyIndicesA, const __global int* connectedBodyIndicesB)
{
	int bodyIndex;
	const __global int* connectedBodyIndices;
	if (bodyA < bodyB)
	{
		bodyIndex = bodyB;
		connectedBodyIndices = connectedBodyIndicesA;
	}
	else
	{
		bodyIndex = bodyA;
		connectedBodyIndices = connectedBodyIndicesB;
	}

	for (int i = 0; i < MAX_CONNECTED_BODY_INDICES; ++i)
	{
		if (connectedBodyIndices[i] == bodyIndex)
			return false;
		if (connectedBodyIndices[i] == -1)
			return true;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ShouldCollide
//
// Determines whether bodies A and B should collide or not from their category bits, mask bits, and group indices.
// It follows the collision filtering rule in the Box2D manual 6.2.
////////////////////////////////////////////////////////////////////////////////////////////////////
bool ShouldCollide(ushort categoryBitsA, ushort maskBitsA, short groupIndexA, 
				   ushort categoryBitsB, ushort maskBitsB, short groupIndexB)
{
	if (groupIndexA == groupIndexB && groupIndexA != 0)
	{
		return groupIndexA > 0;
	}

	bool collide = (maskBitsA & categoryBitsB) != 0 && (categoryBitsA & maskBitsB) != 0;
	return collide;
}

__kernel void ComputeAABBs(
		const __global b2clPolygonShape* polyGlobal,
		const __global b2clBodyStatic* bodyStaticListBuffer,
		const __global b2clTransform* xfGlobal,
		__global b2clAABB* AABBs, // output
		const __global int* shapeToBodyMap,
		int numShapes)
{
	int i = get_global_id(0);

	if (i>=numShapes)
		return;

	b2clPolygonShape poly = polyGlobal[i];
	int j = shapeToBodyMap[i];
	b2clTransform xf = xfGlobal[j];
	b2clBodyStatic bs = bodyStaticListBuffer[j];

	const float2* vertices = poly.m_vertices;

	float2 lower = b2clMul_Transform(&xf, vertices[0]);
	float2 upper = lower;

	if (poly.m_type>0) // edge (m_type==1) or polygon (m_type==2)
	{
		float2 v = b2clMul_Transform(&xf, vertices[1]);
		lower = b2clMin(lower, v);
		upper = b2clMax(upper, v);
	}

	// In Box2D 2.1.2, polygon is m_type==1
	if (poly.m_type==2) // polygon (m_type==2)
	{
		for (int k = 2; k < poly.m_vertexCount; ++k)
		{
			float2 v = b2clMul_Transform(&xf, vertices[k]);
			lower = b2clMin(lower, v);
			upper = b2clMax(upper, v);
		}
	}

	float r = poly.m_radius;

	AABBs[i].m_min[0] = lower.x - r;
	AABBs[i].m_min[1] = lower.y - r;
	AABBs[i].m_max[0] = upper.x + r;
	AABBs[i].m_max[1] = upper.y + r;
	
	AABBs[i].m_sType = poly.m_type;
	AABBs[i].m_bType = bs.m_type;
}

__kernel void ComputeAABBsTOI(
		const __global b2clPolygonShape* polyGlobal,
		const __global b2clBodyStatic* bodyStaticListBuffer,
	    const __global b2clBodyDynamic* bodyDynamicListBuffer,
		const __global b2clTransform* xfGlobal,
		__global b2clAABB* AABBs, // output
		const __global int* shapeToBodyMap,
		int numShapes)
{
	int i = get_global_id(0);

	if (i>=numShapes)
		return;

	b2clPolygonShape poly = polyGlobal[i];
	int j = shapeToBodyMap[i];
	b2clTransform xf = xfGlobal[j];
	b2clBodyStatic bs = bodyStaticListBuffer[j];
	b2clBodyDynamic bd = bodyDynamicListBuffer[j];

	const float2* vertices = poly.m_vertices;

	float2 lower = b2clMul_Transform(&xf, vertices[0]);
	float2 upper = lower;

	if (poly.m_type>0) // edge (m_type==1) or polygon (m_type==2)
	{
		float2 v = b2clMul_Transform(&xf, vertices[1]);
		lower = b2clMin(lower, v);
		upper = b2clMax(upper, v);
	}

	// In Box2D 2.1.2, polygon is m_type==1
	if (poly.m_type==2) // polygon (m_type==2)
	{
		for (int k = 2; k < poly.m_vertexCount; ++k)
		{
			float2 v = b2clMul_Transform(&xf, vertices[k]);
			lower = b2clMin(lower, v);
			upper = b2clMax(upper, v);
		}
	}

	float r = poly.m_radius;

	// synchronize xf for each body
	float a0, a;
	float2 c0, c;
	float2 p0, q0, p, q, d;

	a0 = bd.m_sweep.a0;
	a = bd.m_sweep.a;
	c0 = bd.m_sweep.c0;
	c = bd.m_sweep.c;

	float sina, cosa;
	
	sina = sincos(a0, &cosa);
	q0.x = sina;
	q0.y = cosa;
	//q0.x = /*native_*/sin(a0);
	//q0.y = /*native_*/cos(a0);
	//q0.y = cos_wrapper(a0);
	p0 = c0 - b2clMul_Rotate(q0, bs.m_localCenter);

	sina = sincos(a, &cosa);
	q.x = sina;
	q.y = cosa;
	//q.x = /*native_*/sin(a);
	//q.y = /*native_*/cos(a);
	//q.y = cos_wrapper(a);
	p = c - b2clMul_Rotate(q, bs.m_localCenter);

	d = (p - p0) * b2cl_aabbMultiplier;

	//if (!(d.x<0.0f) && !(d.x>=0.0f))
	//{
	//	printf("%d, %f\n", i, d.x);
	//	printf("a0: %f, c0: (%f, %f), a: %f, c: (%f, %f)\n", a0, c0.x, c0.y, a, c.x, c.y);
	//}
	
	// Extend AABB with d
	if (d.x < 0.0f)
	{
		lower.x += d.x;
	}
	else
	{
		upper.x += d.x;
	}

	if (d.y < 0.0f)
	{
		lower.y += d.y;
	}
	else
	{
		upper.y += d.y;
	}

	AABBs[i].m_min[0] = lower.x - r;
	AABBs[i].m_min[1] = lower.y - r;
	AABBs[i].m_max[0] = upper.x + r;
	AABBs[i].m_max[1] = upper.y + r;

	//if (i==0)
	//{
	//	printf("AABB %d: (%f, %f)-(%f, %f)\n", i, lower.x, lower.y, upper.x, upper.y);
	//	printf("d %d: (%f, %f)\n", i, d.x, d.y);
	//}

	AABBs[i].m_sType = poly.m_type;
	AABBs[i].m_bType = bs.m_type;
}

__kernel void PrepareSumVariance( __global const b2clAABB* AABBs, 
			__global float4* sum, 
			//__global float4* sum2, 
			int numAabbs)
{
	int i = get_global_id(0);
	if (i>numAabbs)
		return;
	float2 s, s2;
	s.x = (AABBs[i].m_max[0]+AABBs[i].m_min[0])*0.5f;
	s.y = (AABBs[i].m_max[1]+AABBs[i].m_min[1])*0.5f;
	s2 = s*s;
	sum[i] = (float4)(s, s2);
	//sum2[i] = s*s;	
}

__kernel void PrepareSumVarianceNoFloat4( __global const b2clAABB* AABBs, 
			__global float* sumX, 
			__global float* sumY, 
			__global float* sumX2, 
			__global float* sumY2, 
			int numAabbs)
{
	int i = get_global_id(0);
	if (i>numAabbs)
		return;
	float2 s, s2;
	s.x = (AABBs[i].m_max[0]+AABBs[i].m_min[0])*0.5f;
	s.y = (AABBs[i].m_max[1]+AABBs[i].m_min[1])*0.5f;
	s2 = s*s;
	sumX[i] = s.x;
	sumY[i] = s.y;
	sumX2[i] = s2.x;
	sumY2[i] = s2.y;
}

__kernel void InitSortingKeys(__global const b2clAABB* AABBs, 
		__global uint* keysAABB, // output
		__global uint* indicesAABB, // output
		int numAabbs,
		int axis, 
		int sortCount)
{
	int i = get_global_id(0);
	if (i>=numAabbs)
	{
		if (i<sortCount)
			keysAABB[i] = 0;
		return;
	}

	keysAABB[i] = FloatFlip(AABBs[i].m_max[axis]); // use max_x/y for sorting (descending), convert it to int
	indicesAABB[i] = i;
}

__kernel void ComputePairs( 
		const __global b2clAABB* aabbs, 
//		__global b2clAABB* test, 
		const __global b2clBodyStatic* bodyStaticListBuffer, // only need body type here, should extract it later
		const __global b2clPolygonShape* shapeListBuffer,
		__global int4* globalIndices, // output
		volatile __global int* pairIndices, // output
		const __global uint* indicesAABB,
		const __global int* shapeToBodyMap,
		volatile __global int* pairCounts, // output
		volatile __global int* pairTotalCount, // output
		int numObjects, 
		int axis, 
		int maxPairs)
{
	int i = get_global_id(0);

	if (i>=numObjects)
		return;

	int shape_i, shape_j, body_i, body_j, type_i, type_j;
	b2clAABB aabb_i, aabb_j;
	shape_i = indicesAABB[i];
	body_i = shapeToBodyMap[shape_i];
	aabb_i = aabbs[shape_i];
	type_i = aabbs[shape_i].m_bType;

	//if (i==0)
	//{
	//	printf("aabb[0]: (%f, %f)-(%f, %f)\n", aabbs[0].m_min[0], aabbs[0].m_min[1], aabbs[0].m_max[0], aabbs[0].m_max[1]);
	//}

//	int localPairCount = 0;
//	
//#define max_pair_per_shape 10
//	int4 localPairIndicesBuffer[max_pair_per_shape]; // assume there will be less than max_pair_per_shape pair for each shape

	unsigned short categoryBitsA = shapeListBuffer[shape_i].m_filter.categoryBits;
	unsigned short maskBitsA = shapeListBuffer[shape_i].m_filter.maskBits;
	short groupIndexA = shapeListBuffer[shape_i].m_filter.groupIndex;

	for (int j=i+1; j<numObjects; j++)
	{
		shape_j = indicesAABB[j];
		//aabb_j = aabbs[shape_j];
		if(aabb_i.m_min[axis] > (aabbs[shape_j].m_max[axis])) 
		{
			break;
		}

		body_j = shapeToBodyMap[shape_j];
		type_j = aabbs[shape_j].m_bType;

		if (body_i==body_j)
			continue;

		bool bIsIntersection = TestAabbAgainstAabb2GlobalGlobal(&aabb_i, &aabbs[shape_j]) && (type_i==2 || type_j==2);

		// check joint collide connected
		bIsIntersection &= ShouldCollideJoints(body_i, body_j, bodyStaticListBuffer[body_i].m_connectedBodyIndices, bodyStaticListBuffer[body_j].m_connectedBodyIndices);

		// check user filtering
		unsigned short categoryBitsB = shapeListBuffer[shape_j].m_filter.categoryBits;
		unsigned short maskBitsB = shapeListBuffer[shape_j].m_filter.maskBits;
		short groupIndexB = shapeListBuffer[shape_j].m_filter.groupIndex;
		bIsIntersection &= ShouldCollide(categoryBitsA, maskBitsA, groupIndexA, categoryBitsB, maskBitsB, groupIndexB);

        int4 currentPairIndices;
        int c_type;
        int curPair, curTotalPair;
		if (bIsIntersection)
		{
			//printf("i: %d, shape_i: %d, shape_j: %d\n", i, shape_i, shape_j);
			//printf("\tAABB[%d]: (%f, %f)-(%f, %f)\n", shape_i, aabbs[shape_i].m_min[0], aabbs[shape_i].m_min[1], aabbs[shape_i].m_max[0], aabbs[shape_i].m_max[1]);
			//printf("\tAABB[%d]: (%f, %f)-(%f, %f)\n", shape_j, aabbs[shape_j].m_min[0], aabbs[shape_j].m_min[1], aabbs[shape_j].m_max[0], aabbs[shape_j].m_max[1]);

			currentPairIndices.x = shape_i;
			currentPairIndices.y = shape_j;
			currentPairIndices.z = body_i;
			currentPairIndices.w = body_j;

			int s_type_i = aabb_i.m_sType;
			int s_type_j = aabbs[shape_j].m_sType;

			// Set c_type according to s_type_i and s_type_j
			// Suppose only have circle (0), edge (1), and polygon (2) at this time
			// 0: circle-circle
			// 1: circle-polygon (A is polygon and B is circle)
			// 2: polygon-polygon
			// 3: edge-circle (A is edge and B is circle)
			// 4: edge-polygon (A is edge and B is polygon)
			// note that edge-edge is NOT supported in Box2D
			if (s_type_i==0) // A is circle
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 0;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 3;
					// swap the two shpaes to make sure A is edge and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 1;
					// swap the two shpaes to make sure A is polygon and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
			}
			else if (s_type_i==1) // A is edge
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 3;
				}
				else if (s_type_j==1) // B is edge
				{
					// This should never happen
				}
				else // B is polygon
				{
					c_type = 4;
				}
			}
			else // A is polygon
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 1;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 4;
					// swap the two shpaes to make sure A is edge and B is polygon
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 2;
				}
			}

			// the following two atomic_inc may have some problem?
			// check it later!!!
			curPair = atomic_inc(pairCounts+c_type);
			curTotalPair = atomic_inc(pairTotalCount);
			if (curTotalPair<maxPairs)
			{
				globalIndices[curTotalPair] = currentPairIndices; //flush to main memory
				pairIndices[maxPairs*c_type+curPair] = curTotalPair; //flush to main memory
			}
			//printf("pairCount: %d, shape_i: %d, shape_j: %d, body_i: %d, body_j: %d\n", curPair, shape_i, shape_j, body_i, body_j);
		}
	}
	//int curPair = atomic_add(pairCount, localPairCount);
	//if (curPair+localPairCount<maxPairs)
	//{
	//	for (int k=0; k<localPairCount; k++)
	//	{
	//		pairIndices[curPair+k] = localPairIndicesBuffer[k]; //flush to main memory
	//	}
	//}
}

__kernel void ComputePairsNoAtomic( 
		const __global b2clAABB* aabbs, 
		const __global b2clBodyStatic* bodyStaticListBuffer, // only need body type here, should extract it later
		const __global b2clPolygonShape* shapeListBuffer,
		__global int4* globalIndices, // output
		__global int* pairIndices, // output
		__global int* pairIndicesBinaryBits, // output
		const __global uint* indicesAABB,
		const __global int* shapeToBodyMap,
		int numObjects, 
		int axis, 
		int maxPairsPerFixture,
		int maxPairs)
{
	int i = get_global_id(0);

	if (i>=numObjects)
		return;

	int base_address = i*maxPairsPerFixture;

	int shape_i, shape_j, body_i, body_j, type_i, type_j;
	shape_i = indicesAABB[i];
	body_i = shapeToBodyMap[shape_i];
	type_i = aabbs[shape_i].m_bType;

//	int localPairCount = 0;
//	
//#define max_pair_per_shape 10
//	int4 localPairIndicesBuffer[max_pair_per_shape]; // assume there will be less than max_pair_per_shape pair for each shape

	unsigned short categoryBitsA = shapeListBuffer[shape_i].m_filter.categoryBits;
	unsigned short maskBitsA = shapeListBuffer[shape_i].m_filter.maskBits;
	short groupIndexA = shapeListBuffer[shape_i].m_filter.groupIndex;

	int num_pairs = 0;
	// currently only support 5 types of contacts
	int type_counts[5];
	for (int k=0; k<5; k++)
		type_counts[k] = 0;

	for (int j=i+1; j<numObjects; j++)
	{
		shape_j = indicesAABB[j];
		if(aabbs[shape_i].m_min[axis] > (aabbs[shape_j].m_max[axis])) 
		{
			break;
		}

		body_j = shapeToBodyMap[shape_j];
		type_j = aabbs[shape_j].m_bType;

		if (body_i==body_j)
			continue;

		bool bIsIntersection = 1;//TestAabbAgainstAabb2GlobalGlobal(&aabbs[shape_i], &aabbs[shape_j]) && (type_i==2 || type_j==2);

		// check joint collide connected
		bIsIntersection &= ShouldCollideJoints(body_i, body_j, bodyStaticListBuffer[body_i].m_connectedBodyIndices, bodyStaticListBuffer[body_j].m_connectedBodyIndices);

		// check user filtering
		unsigned short categoryBitsB = shapeListBuffer[shape_j].m_filter.categoryBits;
		unsigned short maskBitsB = shapeListBuffer[shape_j].m_filter.maskBits;
		short groupIndexB = shapeListBuffer[shape_j].m_filter.groupIndex;
		bIsIntersection &= ShouldCollide(categoryBitsA, maskBitsA, groupIndexA, categoryBitsB, maskBitsB, groupIndexB);

        int4 currentPairIndices;
        int c_type;
		if (bIsIntersection)
		{
			currentPairIndices.x = shape_i;
			currentPairIndices.y = shape_j;
			currentPairIndices.z = body_i;
			currentPairIndices.w = body_j;

			int s_type_i = aabbs[shape_i].m_sType;
			int s_type_j = aabbs[shape_j].m_sType;

			// Set c_type according to s_type_i and s_type_j
			// Suppose only have circle (0), edge (1), and polygon (2) at this time
			// 0: circle-circle
			// 1: circle-polygon (A is polygon and B is circle)
			// 2: polygon-polygon
			// 3: edge-circle (A is edge and B is circle)
			// 4: edge-polygon (A is edge and B is polygon)
			// note that edge-edge is NOT supported in Box2D
			if (s_type_i==0) // A is circle
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 0;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 3;
					// swap the two shpaes to make sure A is edge and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 1;
					// swap the two shpaes to make sure A is polygon and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
			}
			else if (s_type_i==1) // A is edge
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 3;
				}
				else if (s_type_j==1) // B is edge
				{
					// This should never happen
				}
				else // B is polygon
				{
					c_type = 4;
				}
			}
			else // A is polygon
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 1;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 4;
					// swap the two shpaes to make sure A is edge and B is polygon
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 2;
				}
			}

			if (num_pairs<maxPairs)
			{
				globalIndices[base_address + num_pairs] = currentPairIndices;
				pairIndices[maxPairs*c_type + base_address + type_counts[c_type]] = base_address + num_pairs;
				pairIndicesBinaryBits[maxPairs*c_type + base_address + type_counts[c_type]] = 1;
				num_pairs++;
				type_counts[c_type]++;
			}
		}
	}
}

#define WORKGROUP_SIZE 256
//#define WORKGROUP_SIZE 1024

__kernel void computePairsLocalMemory( 
		__global const b2clAABB* aabbs, 
		const __global b2clBodyStatic* bodyStaticListBuffer, // only need body type here, should extract it later
		const __global b2clPolygonShape* shapeListBuffer,
		__global int4* globalIndices, // output
		__global int4* pairIndices, // output
		const __global uint* indicesAABB,
		const __global int* shapeToBodyMap,
		volatile __global int* pairCount, // output
		volatile __global int* pairTotalCount, // output
		int numObjects, 
		int axis, 
		int maxPairs)
{
	int i = get_global_id(0);

	//if (i>=numObjects)
	//	return;

	int localId = get_local_id(0);

	__local int numActiveWgItems[1];
	__local int breakRequest[1];
	__local int localShapeI[WORKGROUP_SIZE*2]; // = indicesAABB[i];
	__local b2clAABB localAabbs[WORKGROUP_SIZE*2]; // = aabbs[shape_i];
	__local int localBodyI[WORKGROUP_SIZE*2]; // = shapeToBodyMap[shape_i];
	__local int localBodyType[WORKGROUP_SIZE*2]; // = bodyStaticListBuffer[shapeToBodyMap[shape_i]].m_type;

	if (localId==0)
	{
		numActiveWgItems[0] = 0;
		breakRequest[0] = 0;
	}
	int localCount=0;
	int block=0;

	int shape_0, shape_i, body_i, shape_i2, body_i2;
	b2clAABB myAabb;
	int myShapeI, myBodyI, myType;

	// load shape_i
	shape_0 = indicesAABB[0];
	shape_i = i<numObjects ? indicesAABB[i] : shape_0;
	myShapeI = shape_i;
	localShapeI[localId] = myShapeI;
	shape_i2 = (i+WORKGROUP_SIZE)<numObjects ? indicesAABB[i+WORKGROUP_SIZE]: shape_0;
	localShapeI[localId+WORKGROUP_SIZE] = shape_i2;

	// load AABB
	myAabb = aabbs[shape_i];
	float testValue = 	myAabb.m_max[axis];
	localAabbs[localId] = myAabb;
	localAabbs[localId+WORKGROUP_SIZE] = aabbs[shape_i2];

	// load body_i
	body_i = shapeToBodyMap[shape_i];
	myBodyI = body_i;
	localBodyI[localId] = myBodyI;
	body_i2 = shapeToBodyMap[shape_i2];
	localBodyI[localId+WORKGROUP_SIZE] = body_i2;

	// load type_i
	myType = bodyStaticListBuffer[body_i].m_type;
	localBodyType[localId] = myType;
	localBodyType[localId+WORKGROUP_SIZE] = bodyStaticListBuffer[body_i2].m_type;

	barrier(CLK_LOCAL_MEM_FENCE);
	atomic_inc(numActiveWgItems);
	barrier(CLK_LOCAL_MEM_FENCE);
	int localBreak = 0;
	
	int j=i+1;
	do
	{
		barrier(CLK_LOCAL_MEM_FENCE);
	
		if (j<numObjects)
		{
	  		if(testValue < (localAabbs[localCount+localId+1].m_min[axis])) 
			{
				if (!localBreak)
				{
					atomic_inc(breakRequest);
					localBreak = 1;
				}
			}
		}
		
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if (j>=numObjects && !localBreak)
		{
			atomic_inc(breakRequest);
			localBreak = 1;
		}
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if (!localBreak)
		{
			bool bIsIntersection = TestAabbAgainstAabb2(&myAabb,&localAabbs[localCount+localId+1]) &&
				(myType!=0 || localBodyType[localCount+localId+1]!=0);
			if (bIsIntersection)
			{
				int4 currentPairIndices;
				currentPairIndices.x = myShapeI;
				currentPairIndices.y = localShapeI[localCount+localId+1];
				currentPairIndices.z = myBodyI;
				currentPairIndices.w = localBodyI[localCount+localId+1];
				int curPair = atomic_inc (pairCount);
				if (curPair<maxPairs)
				{
						pairIndices[curPair] = currentPairIndices; //flush to main memory
				}
			}
		}
		
		barrier(CLK_LOCAL_MEM_FENCE);

		localCount++;
		if (localCount==WORKGROUP_SIZE)
		{
			localCount = 0;
			block+=WORKGROUP_SIZE;			

			// load shape_i
			shape_i = (i+block)<numObjects ? indicesAABB[i+block] : shape_0;
			localShapeI[localId] = shape_i;
			shape_i2 = (i+block+WORKGROUP_SIZE)<numObjects ? indicesAABB[i+block+WORKGROUP_SIZE]: shape_0;
			localShapeI[localId+WORKGROUP_SIZE] = shape_i2;

			// load AABB
			localAabbs[localId] = aabbs[shape_i];
			localAabbs[localId+WORKGROUP_SIZE] = aabbs[shape_i2];

			// load body_i
			body_i = shapeToBodyMap[shape_i];
			localBodyI[localId] = body_i;
			body_i2 = shapeToBodyMap[shape_i2];
			localBodyI[localId+WORKGROUP_SIZE] = body_i2;

			// load type_i
			localBodyType[localId] = bodyStaticListBuffer[body_i].m_type;
			localBodyType[localId+WORKGROUP_SIZE] = bodyStaticListBuffer[body_i2].m_type;
		}
		j++;
		
	} while (breakRequest[0]<numActiveWgItems[0]);
	
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ComputeAABBIntersection
//
// Kernel for AABBQuery user callback.
// Check intersections between the input AABB and existing AABBs.
// Indices of intersecting AABBs are stored in intersectingShapeIndices.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void ComputeAABBIntersection( 
		float4 inputAabb,
		const __global b2clAABB* aabbs, 
		const __global uint* indicesAABB,
		int numObjects,
		volatile __global int* intersectionCounts, // for each shape type
		volatile __global int* intersectionTotalCount,
		__global unsigned int* intersectingShapeIndices,
		__global unsigned int* intersectingshapeTypes)
{
	int i = get_global_id(0);

	if (i >= numObjects)
		return;

	unsigned int shape_i = indicesAABB[i];

	b2clAABB aabb;
	aabb.m_min[0] = inputAabb.x;
	aabb.m_min[1] = inputAabb.y;
	aabb.m_max[0] = inputAabb.z;
	aabb.m_max[1] = inputAabb.w;

	if(aabbs[shape_i].m_min[0] > aabb.m_max[0] || aabbs[shape_i].m_max[0] < aabb.m_min[0] ||
	   aabbs[shape_i].m_min[1] > aabb.m_max[1] || aabbs[shape_i].m_max[1] < aabb.m_min[1])
	   return;

	unsigned int s_type_i = aabbs[shape_i].m_sType;

	int curIntersection = atomic_inc(intersectionCounts + s_type_i);
	int curTotalIntersection = atomic_inc(intersectionTotalCount);

	intersectingShapeIndices[curTotalIntersection] = shape_i; 
	intersectingshapeTypes[curTotalIntersection] = s_type_i;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayCircleIntersection
//
// Kernel for RayCast user callback.
// Check intersection between the input ray and circle shapes.
// The algorithm is from b2CircleShape::RayCast()
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayCircleIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	unsigned int shapeIndex = shapeIndices[i];
	const b2clPolygonShape shape = shapeListBuffer[shapeIndex];
	unsigned int bodyIndex = shapeToBodyMap[shapeIndex];
	const b2clTransform xf = xfGlobal[bodyIndex];

	output->shapeIndex = shapeIndex;

	float2 position = b2clMul_Transform(&xf, shape.m_centroid);
	float2 s = (float2)(ray.x, ray.y) - position;
	float b = b2clDot(s, s) - shape.m_radius * shape.m_radius;

	// Solve quadratic equation.
	float2 r = (float2)(ray.z, ray.w) - (float2)(ray.x, ray.y);
	float c =  b2clDot(s, r);
	float rr = b2clDot(r, r);
	float sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if (sigma < 0.0f || rr < b2_epsilon)
	{
		output->isCollide = 0;
		return;
	}

	// Find the point of intersection of the line with the circle.
	float a = -(c + sqrt(sigma));

	// Is the intersection point on the segment?
	if (0.0f <= a && a <= 1.0f * rr)
	{
		a /= rr;
		output->fraction = a;
		float2 normal = normalize(s + a * r);
		output->normal[0] = normal.x;
		output->normal[1] = normal.y;
		output->isCollide = 1;
	}
	else
	{
		output->isCollide = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayEdgeIntersection
//
// Kernel for RayCast user callback.
// Check intersection between the input ray and edge shapes.
// The algorithm is from b2EdgeShape::RayCast()
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayEdgeIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	unsigned int shapeIndex = shapeIndices[i];
	const b2clPolygonShape shape = shapeListBuffer[shapeIndex];
	unsigned int bodyIndex = shapeToBodyMap[shapeIndex];
	const b2clTransform xf = xfGlobal[bodyIndex];

	output->shapeIndex = shapeIndex;

	// Put the ray into the edge's frame of reference.
	float2 p1 = b2clMulT_Rotate(xf.q, (float2)(ray.x, ray.y) - xf.p);
	float2 p2 = b2clMulT_Rotate(xf.q, (float2)(ray.z, ray.w) - xf.p);
	float2 d = p2 - p1;

	float2 v1 = shape.m_vertices[0];
	float2 v2 = shape.m_vertices[1];
	float2 e = v2 - v1;
	float2 normal = normalize((float2)(e.y, -e.x));

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	float numerator = b2clDot(normal, v1 - p1);
	float denominator = b2clDot(normal, d);

	if (denominator == 0.0f)
	{
		output->isCollide = 0;
		return;
	}

	float t = numerator / denominator;
	if (t < 0.0f || 1.0f < t)
	{
		output->isCollide = 0;
		return;
	}

	float2 q = p1 + t * d;

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	float2 r = v2 - v1;
	float rr = b2clDot(r, r);
	if (rr == 0.0f)
	{
		output->isCollide = 0;
		return;
	}

	float s = b2clDot(q - v1, r) / rr;
	if (s < 0.0f || 1.0f < s)
	{
		output->isCollide = 0;
		return;
	}

	output->fraction = t;
	if (numerator > 0.0f)
	{
		output->normal[0] = -normal.x;
		output->normal[1] = -normal.y;
	}
	else
	{
		output->normal[0] = normal.x;
		output->normal[1] = normal.y;
	}
	output->isCollide = 1;
	
	return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayPolygonIntersection
//
// Kernel for RayCast user callback.
// Check intersection between the input ray and polygon shapes.
// The algorithm is from b2PolygonShape::RayCast()
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayPolygonIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	unsigned int shapeIndex = shapeIndices[i];
	const b2clPolygonShape shape = shapeListBuffer[shapeIndex];
	unsigned int bodyIndex = shapeToBodyMap[shapeIndex];
	const b2clTransform xf = xfGlobal[bodyIndex];

	output->shapeIndex = shapeIndex;

	// Put the ray into the polygon's frame of reference.
	float2 p1 = b2clMulT_Rotate(xf.q, (float2)(ray.x, ray.y) - xf.p);
	float2 p2 = b2clMulT_Rotate(xf.q, (float2)(ray.z, ray.w) - xf.p);
	float2 d = p2 - p1;

	float lower = 0.0f, upper = 1.0f;

	int index = -1;

	for (int i = 0; i < shape.m_vertexCount; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float numerator = b2clDot(shape.m_normals[i], shape.m_vertices[i] - p1);
		float denominator = b2clDot(shape.m_normals[i], d);

		if (denominator == 0.0f)
		{	
			if (numerator < 0.0f)
			{
				output->isCollide = 0;
				return;
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < 0.0f && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0f && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if (upper < lower)
		{
			output->isCollide = 0;
			return;
		}
	}

	//b2Assert(0.0f <= lower && lower <= input.maxFraction);

	if (index >= 0)
	{
		output->fraction = lower;
		float2 normal = b2clMul_Rotate(xf.q, shape.m_normals[index]);
		output->normal[0] = normal.x;
		output->normal[1] = normal.y;
		output->isCollide = 1;
	}
	else
	{
		output->isCollide = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayChainIntersection
//
// Kernel for RayCast user callback.
// But this function is never called and edge shape is used for chains.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayChainIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	output->isCollide = 0;
}