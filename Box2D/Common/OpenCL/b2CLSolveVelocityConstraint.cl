
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

/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
inline float b2clMixFriction(float friction1, float friction2)
{
	return sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
inline float b2clMixRestitution(float restitution1, float restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

__kernel void InitializeBodyState(
                                 __global clb2Velocity* velocities, // input & output
                                 __global clb2Position* positions, // input & output
								 __global b2clTransform* xfGlobal, // output
                                 const unsigned int num_body,
					  const __global b2clBodyStatic* bodyStaticListBuffer,
					  __global b2clBodyDynamic* bodyDynamicListBuffer,
					  const float2 gravity,
					  const float h)
{
    unsigned int i = get_global_id(0);

    if(i >= num_body) return;

	b2clBodyStatic bs = bodyStaticListBuffer[i];
	b2clBodyDynamic bd = bodyDynamicListBuffer[i];

	float2 c;
	c.x = positions[i].cx;
	c.y = positions[i].cy;
	float a = positions[i].a;
	float2 v;
	v.x = velocities[i].vx;
	v.y = velocities[i].vy;
	float w = velocities[i].w;

	if (bs.m_type == 2)// b2_dynamicBody
	{
		// Integrate velocities.
		v += h * (bs.m_gravityScale * gravity + bs.m_invMass * bd.m_force);
		w += h * bs.m_invI * bd.m_torque;

		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Taylor expansion:
		// v2 = (1.0f - c * dt) * v1
		v *= b2clClamp(1.0f - h * bs.m_linearDamping, 0.0f, 1.0f);
		w *= b2clClamp(1.0f - h * bs.m_angularDamping, 0.0f, 1.0f);
	}

	
	positions[i].cx = c.x;
	positions[i].cy = c.y;
	positions[i].a = a;
	velocities[i].vx = v.x;
	velocities[i].vy = v.y;
	velocities[i].w = w;

	bd.m_sweep.c0 = bd.m_sweep.c;
	bd.m_sweep.a0 = bd.m_sweep.a;
	bodyDynamicListBuffer[i] = bd;
}

__kernel void InitializeBodyStateFirstFrame(
                                 __global clb2Velocity* velocities, // output
                                 __global clb2Position* positions, // output
                                 const unsigned int num_body,
					  const __global b2clBodyStatic* bodyStaticListBuffer,
					  __global b2clBodyDynamic* bodyDynamicListBuffer,
					  const float2 gravity,
					  const float h)
{
    unsigned int i = get_global_id(0);

    if(i >= num_body) return;

	b2clBodyStatic bs = bodyStaticListBuffer[i];
	b2clBodyDynamic bd = bodyDynamicListBuffer[i];

	float2 c = bd.m_sweep.c;
	float a = bd.m_sweep.a;
	float2 v = bd.m_linearVelocity;
	float w = bd.m_angularVelocity;

	if (bs.m_type == 2)// b2_dynamicBody
	{
		// Integrate velocities.
		v += h * (bs.m_gravityScale * gravity + bs.m_invMass * bd.m_force);
		w += h * bs.m_invI * bd.m_torque;

		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Taylor expansion:
		// v2 = (1.0f - c * dt) * v1
		v *= b2clClamp(1.0f - h * bs.m_linearDamping, 0.0f, 1.0f);
		w *= b2clClamp(1.0f - h * bs.m_angularDamping, 0.0f, 1.0f);
	}


	positions[i].cx = c.x;
	positions[i].cy = c.y;
	positions[i].a = a;
	velocities[i].vx = v.x;
	velocities[i].vy = v.y;
	velocities[i].w = w;

	bd.m_sweep.c0 = bd.m_sweep.c;
	bd.m_sweep.a0 = bd.m_sweep.a;
	bodyDynamicListBuffer[i] = bd;
}




__kernel void WarmStart(
                       __global clb2Velocity* velocities, // input and output
                       __global clb2Contact* contacts,
                       __global clb2Impulse* impulses,
                       __global clb2Points* points,
                       const unsigned int contactCount)
{
    unsigned int contactIndex = get_global_id(0);

	int indexA, indexB;
	float2 vA, vB;
	float wA, wB;
    if(contactIndex < contactCount)
	{
		clb2Contact thisContact = contacts[contactIndex];
		clb2Impulse thisImpulse = impulses[contactIndex];
		clb2Points thisPoints = points[contactIndex];

		indexA = thisContact.indexA;
		indexB = thisContact.indexB;
		float mA = thisContact.invMassA;
		float iA = thisContact.invIA;
		float mB = thisContact.invMassB;
		float iB = thisContact.invIB;

		vA.x = 0;
		vA.y = 0;
		wA = 0;
		vB.x = 0;
		vB.y = 0;
		wB = 0;

		float2 normal = thisContact.normal;
		float2 tangent = b2clCross_VS(normal, 1.0f);

	//printf("normalImpuls: %f, tangentImpulse: %f\n", thisImpulse.normalImpulse1, thisImpulse.tangentImpulse1);
		float2 P = thisImpulse.normalImpulse1 * normal + thisImpulse.tangentImpulse1 * tangent;
		wA -= iA * b2clCross_VV(thisPoints.rA1, P);
		vA -= mA * P;
		wB += iB * b2clCross_VV(thisPoints.rB1, P);
		vB += mB * P;

		if(thisPoints.normalMass2>=0)
		{
			P = thisImpulse.normalImpulse2 * normal + thisImpulse.tangentImpulse2 * tangent;
			wA -= iA * b2clCross_VV(thisPoints.rA2, P);
			vA -= mA * P;
			wB += iB * b2clCross_VV(thisPoints.rB2, P);
			vB += mB * P;
		}


	}

	// serialize all threads to make sure synchronization
	for (int k=0; k<contactCount; k++)
	{
		if (contactIndex==k)
		{
			velocities[indexA].vx += vA.x;
			velocities[indexA].vy += vA.y;
			velocities[indexA].w += wA;
			velocities[indexB].vx += vB.x;
			velocities[indexB].vy += vB.y;
			velocities[indexB].w += wB;
		}
		barrier(CLK_GLOBAL_MEM_FENCE);
	}
}

__kernel void WarmStartWithColoring(
                       __global clb2Velocity* velocities, // input and output
                       __global clb2Contact* contacts,
                       __global clb2Impulse* impulses,
                       __global clb2Points* points,
                       const unsigned int offset,
                       const unsigned int colorLength)
{
    unsigned int contactIndex = get_global_id(0) + offset;

	if(contactIndex>=offset+colorLength) return;

	int indexA, indexB;
	float2 vA, vB;
	float wA, wB;

	clb2Contact thisContact = contacts[contactIndex];
	clb2Impulse thisImpulse = impulses[contactIndex];
	clb2Points thisPoints = points[contactIndex];

	indexA = thisContact.indexA;
	indexB = thisContact.indexB;
	float mA = thisContact.invMassA;
	float iA = thisContact.invIA;
	float mB = thisContact.invMassB;
	float iB = thisContact.invIB;

	vA.x = 0;
	vA.y = 0;
	wA = 0;
	vB.x = 0;
	vB.y = 0;
	wB = 0;

	float2 normal = thisContact.normal;
	float2 tangent = b2clCross_VS(normal, 1.0f);

	float2 P = thisImpulse.normalImpulse1 * normal + thisImpulse.tangentImpulse1 * tangent;
	wA -= iA * b2clCross_VV(thisPoints.rA1, P);
	vA -= mA * P;
	wB += iB * b2clCross_VV(thisPoints.rB1, P);
	vB += mB * P;

	if(thisPoints.normalMass2>=0)
	{
		P = thisImpulse.normalImpulse2 * normal + thisImpulse.tangentImpulse2 * tangent;
		wA -= iA * b2clCross_VV(thisPoints.rA2, P);
		vA -= mA * P;
		wB += iB * b2clCross_VV(thisPoints.rB2, P);
		vB += mB * P;
	}

	velocities[indexA].vx += vA.x;
	velocities[indexA].vy += vA.y;
	velocities[indexA].w += wA;
	velocities[indexB].vx += vB.x;
	velocities[indexB].vy += vB.y;
	velocities[indexB].w += wB;
}


__kernel void WarmStartSplit(
                       __global clb2Velocity* velocities, // input and output
					   __global clb2Velocity* splitVelocities,
					   __global unsigned int* indexContact2BodySplitVelocities,
					   __global unsigned int* hasSplitVelocitiesContacts,
					   __global unsigned int* Contacts4EachBody, 
                       __global clb2Contact* contacts,
                       __global clb2Impulse* impulses,
                       __global clb2Points* points,
                       unsigned int numContact,
                       unsigned int numBody
					 )
{

    unsigned int indexThread = get_global_id (0); 


	unsigned int contactIndex = indexThread ;
	int maxContactNumPerBody = 20 ; 
	//unsigned int contactIndex = get_global_id(0);
	//if (contactIndex >= numContact) return ; 
	int indexA, indexB; float2 vA, vB; float wA, wB;

    if (contactIndex < numContact) {

		clb2Contact thisContact = contacts[contactIndex];
		clb2Impulse thisImpulse = impulses[contactIndex];
		clb2Points thisPoints = points[contactIndex];

		indexA = thisContact.indexA;
		indexB = thisContact.indexB;
		float mA = thisContact.invMassA;
		float iA = thisContact.invIA;
		float mB = thisContact.invMassB;
		float iB = thisContact.invIB;
		int numBodyA = Contacts4EachBody[indexA]; 
		int numBodyB = Contacts4EachBody[indexB];  mA *= numBodyA ; mB *= numBodyB ; 
		int isGround[2] ; isGround[0] = (mA == 0)? 1 :0 ; isGround[1] = (mB == 0) ? 1:0 ; 
		unsigned int indexContact2BodyA = indexContact2BodySplitVelocities[contactIndex*2+0];
		unsigned int indexContact2BodyB = indexContact2BodySplitVelocities[contactIndex*2+1];

		vA.x = 0; vA.y = 0; wA = 0;vB.x = 0; vB.y = 0; wB = 0;

		float2 normal = thisContact.normal;
		float2 tangent = b2clCross_VS(normal, 1.0f);

		float2 P = thisImpulse.normalImpulse1 * normal + thisImpulse.tangentImpulse1 * tangent;
		//printf ("this warmstartimpulse: %f \n", thisImpulse.normalImpulse1);
		//printf ("thisImpulse.normalImpulse1: %f \n", thisImpulse.normalImpulse1);
		wA -= iA * b2clCross_VV(thisPoints.rA1, P); vA -= mA * P;
		wB += iB * b2clCross_VV(thisPoints.rB1, P); vB += mB * P;
		
		//printf ("warm start thisImpulse.normalImpulse1 %f \n", thisImpulse.normalImpulse1 );
		
		

		if(thisPoints.normalMass2>=0)
		{
			P = thisImpulse.normalImpulse2 * normal + thisImpulse.tangentImpulse2 * tangent;
			wA -= iA * b2clCross_VV(thisPoints.rA2, P);
			vA -= mA * P;
			wB += iB * b2clCross_VV(thisPoints.rB2, P);
			vB += mB * P;
		}
		clb2Velocity splitVA, splitVB ; 
		
		
		//printf ("contact number: %d , value: %f \n", contactIndex,  wA);
		splitVA.vx = vA.x ; splitVA.vy = vA.y*1.0  ; splitVA.w = wA ; 
		splitVB.vx = vB.x ; splitVB.vy = vB.y*1.0  ; splitVB.w = wB ; 

		

	    if (!isGround[0]) {
		       // splitVelocities [indexA * maxContactNumPerBody + indexContact2BodyA] = splitVA ; 		
			   splitVelocities [indexA * maxContactNumPerBody + indexContact2BodyA] = splitVA ; 
			   //printf ("warmstart splitVB.vy:  %f, ", splitVA.vy);		
		}
	    if (!isGround[1]) {
		       splitVelocities [indexB * maxContactNumPerBody + indexContact2BodyB] = splitVB ;
			   //printf ("warmstart splitVB.vy:  %f, ", splitVB.vy);
		}
		

	}


	 	
}



__kernel void WarmStartSplitWithColoring(
                       __global clb2Velocity* velocities, // input and output
                       __global clb2Contact* contacts,
                       __global clb2Impulse* impulses,
					   __global unsigned int* numContacts4EachBody,
                       __global clb2Points* points,
                       const unsigned int offset,
                       const unsigned int colorLength)
{

    unsigned int contactIndex = get_global_id(0) + offset;

	if(contactIndex>=offset+colorLength) return;

	int indexA, indexB;
	float2 vA, vB;
	float wA, wB;

	clb2Contact thisContact = contacts[contactIndex];
	clb2Impulse thisImpulse = impulses[contactIndex];
	clb2Points thisPoints = points[contactIndex];

	indexA = thisContact.indexA;
	indexB = thisContact.indexB;
	float mA = thisContact.invMassA;
	float iA = thisContact.invIA;
	float mB = thisContact.invMassB;
	float iB = thisContact.invIB;

	unsigned int numContactsA = numContacts4EachBody[indexA];
	unsigned int numContactsB = numContacts4EachBody[indexB];

	vA.x = 0;
	vA.y = 0;
	wA = 0;
	vB.x = 0;
	vB.y = 0;
	wB = 0;

	float2 normal = thisContact.normal;
	float2 tangent = b2clCross_VS(normal, 1.0f);

	float2 P = thisImpulse.normalImpulse1 * normal + thisImpulse.tangentImpulse1 * tangent;
	//P /= 4.0;
	wA -= iA * b2clCross_VV(thisPoints.rA1, P);
	vA -= mA * P;
	wB += iB * b2clCross_VV(thisPoints.rB1, P);
	vB += mB * P;

	if(thisPoints.normalMass2>=0)
	{
		P = thisImpulse.normalImpulse2 * normal + thisImpulse.tangentImpulse2 * tangent;
		//P /=4.0 ; 
		wA -= iA * b2clCross_VV(thisPoints.rA2, P);
		vA -= mA * P;
		wB += iB * b2clCross_VV(thisPoints.rB2, P);
		vB += mB * P;
	}


	if (numContactsA == 0) {vA.x = vA.y = wA = 0 ; numContactsA = 1 ; }
	if (numContactsB == 0) {vB.x = vB.y = wB = 0 ; numContactsB = 1 ; }

	float fNumA = numContactsA ,fNumB = numContactsB ; 



	velocities[indexA].vx += vA.x;
	velocities[indexA].vy += vA.y;
	velocities[indexA].w += wA;
	velocities[indexB].vx += vB.x;
	velocities[indexB].vy += vB.y;
	velocities[indexB].w += wB;


}






__kernel void InitializeVelocityConstraint(
                                 __global clb2Contact* contacts, // output
                                 __global clb2Impulse* impulses, // output
                                 __global clb2Points* points, // output
                                 __global clb2Manifold* manifoldsForPositionSolver, // output
								 //__global uint* manifoldKeys, // output, keys to be sorted
								 //__global uint* globalIndices, // output, values to be sorted
                                 const unsigned int contactCount,
						const __global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clFixtureStatic* fixtureStaticListBuffer,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global int4* indices, //indices to bodies and fixtures
						const __global int* coloredContactIndexToContactIndexMap,
						const __global b2clPolygonShape* polyGlobal,
						const __global b2clTransform* xfGlobal,
						const __global b2clManifold* manifolds, // input
						const int warmStarting,
						const float dtRatio)
{
    unsigned int contactIndex = get_global_id(0);

    if(contactIndex >= contactCount) return;

    clb2Contact thisContact;
    clb2Points currentPoints;
    clb2Impulse currentImpulses;
    clb2Manifold currentManifolds;

    int globalIndex = coloredContactIndexToContactIndexMap[contactIndex];
	int4 currentIndices = indices[globalIndex];

	float frictionA = fixtureStaticListBuffer[currentIndices.x].m_friction;
	float frictionB = fixtureStaticListBuffer[currentIndices.y].m_friction;
	thisContact.friction = b2clMixFriction(frictionA, frictionB);

	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[currentIndices.z];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[currentIndices.w];

	//// for debug
	//if (bodyStaticA.m_type==2 && bodyStaticB.m_type==2)
	//	printf("Both bodies (%d and %d) of #%d contact are dynamic!!!\n", currentIndices.z, currentIndices.w, contactIndex);

	thisContact.invMassA = bodyStaticA.m_invMass;
	thisContact.invMassB = bodyStaticB.m_invMass;
	thisContact.invIA = bodyStaticA.m_invI;
	thisContact.invIB = bodyStaticB.m_invI;
	thisContact.indexA = currentIndices.z;
	thisContact.indexB = currentIndices.w;

	//extractd code from b2WorldManifold::Initialize
	b2clManifold manifold = manifolds[globalIndex];
	b2clTransform xfA = xfGlobal[currentIndices.z];
	b2clTransform xfB = xfGlobal[currentIndices.w];
	float radiusA = polyGlobal[currentIndices.x].m_radius;
	float radiusB = polyGlobal[currentIndices.y].m_radius;
	float2 worldManifoldPoints[b2cl_maxManifoldPoints];
	float2 normal;
	switch (manifold.type)
	{
	case 0:
		{
			normal = (float2)(1.0, 0.0);
			float2 pointA = b2clMul_Transform(&xfA, manifold.localPoint);
			float2 pointB = b2clMul_Transform(&xfB, manifold.points[0].localPoint);
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
			normal = b2clMul_Rotate(xfA.q, manifold.localNormal);
			float2 planePoint = b2clMul_Transform(&xfA, manifold.localPoint);
			
			for (int i = 0; i < manifold.pointCount; ++i)
			{
				float2 clipPoint = b2clMul_Transform(&xfB, manifold.points[i].localPoint);
				float2 cA = clipPoint + (radiusA - b2clDot(clipPoint - planePoint, normal)) * normal;
				float2 cB = clipPoint - radiusB * normal;
				worldManifoldPoints[i] = 0.5f * (cA + cB);
				//worldManifoldPoints[i] = clipPoint;
			}
			break;
		}
	case 2:
		{
			normal = b2clMul_Rotate(xfB.q, manifold.localNormal);
			float2 planePoint = b2clMul_Transform(&xfB, manifold.localPoint);

			for (int i = 0; i < manifold.pointCount; ++i)
			{
				float2 clipPoint = b2clMul_Transform(&xfA, manifold.points[i].localPoint);
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
	float mixedrestitution = b2clMixRestitution(restitutionA, restitutionB);

	currentPoints.velocityBias1 = 0.0f;
	float vRel = b2clDot(normal, vB + b2clCross_SV(wB, currentPoints.rB1) - vA - b2clCross_SV(wA, currentPoints.rA1));
	if (vRel < -b2cl_velocityThreshold)
	{
		currentPoints.velocityBias1 = -mixedrestitution * vRel;
	}

	// This part is only useful for first frame.
	// In later frames, impulse will be reset in ReadLastImpulse
	if (warmStarting)
	{
		currentImpulses.normalImpulse1 = dtRatio * manifold.points[0].normalImpulse;
		currentImpulses.tangentImpulse1 = dtRatio * manifold.points[0].tangentImpulse;
	}
	else
	{
		currentImpulses.normalImpulse1 = 0.0f;
		currentImpulses.tangentImpulse1 = 0.0f;
	}

    // Extract useful manifold information for the position constraint solver
    currentManifolds.localNormal = manifold.localNormal;
    currentManifolds.localPoint = manifold.localPoint;
    currentManifolds.type=manifold.type;
    currentManifolds.pointCount=manifold.pointCount;
    currentManifolds.localPoints1=manifold.points[0].localPoint;
    currentManifolds.radiusA=radiusA;
    currentManifolds.radiusB=radiusB;
    currentManifolds.localCenterA=bodyStaticA.m_localCenter;
    currentManifolds.localCenterB=bodyStaticB.m_localCenter;
    
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

		// This part is only useful for first frame.
		// In later frames, impulse will be reset in ReadLastImpulse
		if (warmStarting)
		{
			currentImpulses.normalImpulse2 = dtRatio * manifold.points[1].normalImpulse;
			currentImpulses.tangentImpulse2 = dtRatio * manifold.points[1].tangentImpulse;
		}
		else
		{
			currentImpulses.normalImpulse2 = 0.0f;
			currentImpulses.tangentImpulse2 = 0.0f;
		}

		currentManifolds.localPoints2=manifold.points[1].localPoint;
	}
	else
	{
		currentPoints.rA2 = -1;
		currentPoints.rB2 = -1;
		currentPoints.normalMass2 = -1;
		currentPoints.tangentMass2 = -1;
		currentPoints.velocityBias2 = -1;
		currentImpulses.normalImpulse2 = 0;
		currentImpulses.tangentImpulse2 = 0;
	}

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

	// This part is only useful for first frame.
	// In later frames, impulse will be reset in ReadLastImpulse
	impulses[contactIndex] = currentImpulses;
    
    manifoldsForPositionSolver[contactIndex] = currentManifolds;

	//if (currentIndices.x<currentIndices.y)
	//	manifoldKeys[contactIndex] = currentIndices.x<<16 | currentIndices.y; // suppose both indices are less than 65536 and thus fit in 16 bits.
	//else
	//	manifoldKeys[contactIndex] = currentIndices.y<<16 | currentIndices.x; // suppose both indices are less than 65536 and thus fit in 16 bits.
	//globalIndices[contactIndex] = globalIndex;
}


__kernel void CountContactNum4EachBodyConstraint (				 
                              volatile __global unsigned int* numContacts4EachBody,
							  __global unsigned int* indexContact2BodySplitVelocity,
							  const __global int4* indices, 
							  const __global int* coloredContactIndexToContactIndexMap,
							   const __global b2clBodyStatic* bodyStaticListBuffer,
							  unsigned int contactCount
                             ) 
{



 unsigned int contactIndex = get_global_id(0) ; 
	if (contactIndex >= contactCount) return ; 
	
    clb2Contact thisContact;
    int globalIndex = coloredContactIndexToContactIndexMap[contactIndex];
	int4 currentIndices = indices[globalIndex];
	thisContact.indexA = currentIndices.z;
	thisContact.indexB = currentIndices.w;
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[currentIndices.z];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[currentIndices.w];
	if (bodyStaticA.m_invMass != 0) 
       indexContact2BodySplitVelocity[contactIndex*2+0] =  atomic_inc ( numContacts4EachBody+thisContact.indexA );  
	if (bodyStaticB.m_invMass != 0 )
       indexContact2BodySplitVelocity[contactIndex*2+1] =  atomic_inc ( numContacts4EachBody+thisContact.indexB );

}

__kernel void InitializeVelocityConstraint_HasSplit(
                                 __global clb2Contact* contacts, // output
                                 __global clb2Impulse* impulses, // output
                                 __global clb2Points* points, // output
                                 __global clb2Manifold* manifoldsForPositionSolver, // output
								 //__global uint* manifoldKeys, // output, keys to be sorted
								 //__global uint* globalIndices, // output, values to be sorted
                                 const unsigned int contactCount,
						const __global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clFixtureStatic* fixtureStaticListBuffer,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global int4* indices, //indices to bodies and fixtures
						const __global int* coloredContactIndexToContactIndexMap,
						const __global b2clPolygonShape* polyGlobal,
						const __global b2clTransform* xfGlobal,
						const __global b2clManifold* manifolds, // input
						const int warmStarting,
						const float dtRatio,
						__global unsigned int* numContacts4EachBody, 
						__global unsigned int* indexContact2BodySplitVelocity
)
{
    unsigned int contactIndex = get_global_id(0) ; 
	if (contactIndex >= contactCount) return ; 
    clb2Contact thisContact;
    clb2Points currentPoints;
    clb2Impulse currentImpulses;
    clb2Manifold currentManifolds;

    int globalIndex = coloredContactIndexToContactIndexMap[contactIndex];
	int4 currentIndices = indices[globalIndex];

	float frictionA = fixtureStaticListBuffer[currentIndices.x].m_friction;
	float frictionB = fixtureStaticListBuffer[currentIndices.y].m_friction;
	thisContact.friction = b2clMixFriction(frictionA, frictionB);

	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[currentIndices.z];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[currentIndices.w];
	thisContact.invMassA = bodyStaticA.m_invMass;
	thisContact.invMassB = bodyStaticB.m_invMass;
	thisContact.invIA = bodyStaticA.m_invI;
	thisContact.invIB = bodyStaticB.m_invI;
	thisContact.indexA = currentIndices.z;
	thisContact.indexB = currentIndices.w;



	//extractd code from b2WorldManifold::Initialize
	b2clManifold manifold = manifolds[globalIndex];
	b2clTransform xfA = xfGlobal[currentIndices.z];
	b2clTransform xfB = xfGlobal[currentIndices.w];
	float radiusA = polyGlobal[currentIndices.x].m_radius;
	float radiusB = polyGlobal[currentIndices.y].m_radius;
	float2 worldManifoldPoints[b2cl_maxManifoldPoints];
	float2 normal;
	switch (manifold.type)
	{
	case 0:
		{
			normal = (float2)(1.0, 0.0);
			float2 pointA = b2clMul_Transform(&xfA, manifold.localPoint);
			float2 pointB = b2clMul_Transform(&xfB, manifold.points[0].localPoint);
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
			normal = b2clMul_Rotate(xfA.q, manifold.localNormal);
			float2 planePoint = b2clMul_Transform(&xfA, manifold.localPoint);
			
			for (int i = 0; i < manifold.pointCount; ++i)
			{
				float2 clipPoint = b2clMul_Transform(&xfB, manifold.points[i].localPoint);
				float2 cA = clipPoint + (radiusA - b2clDot(clipPoint - planePoint, normal)) * normal;
				float2 cB = clipPoint - radiusB * normal;
				worldManifoldPoints[i] = 0.5f * (cA + cB);
				//worldManifoldPoints[i] = clipPoint;
			}
			break;
		}
	case 2:
		{
			normal = b2clMul_Rotate(xfB.q, manifold.localNormal);
			float2 planePoint = b2clMul_Transform(&xfB, manifold.localPoint);

			for (int i = 0; i < manifold.pointCount; ++i)
			{
				float2 clipPoint = b2clMul_Transform(&xfA, manifold.points[i].localPoint);
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
	int numA = numContacts4EachBody[thisContact.indexA] ;  if (numA == 0) numA = 1;
	int numB = numContacts4EachBody[thisContact.indexB] ;  if (numB == 0) numB = 1; 

	float rnA = b2clCross_VV(currentPoints.rA1, normal);
	float rnB = b2clCross_VV(currentPoints.rB1, normal);

	/////////////////////////////////////
	//make invIB small
	//thisContact.invIA = thisContact.invIB = 1 ; 

	/////////////////////////////////////


	

	//printf ("thisContact.invIA: %f, \n" , thisContact.invIA); 
	//printf ("thisContact.invMassA: %f, \n" , thisContact.invMassA); 
	//printf ("kNormal: %f, \n" , kNormal);
	float kNormal = thisContact.invMassA*numA + thisContact.invMassB* numB + thisContact.invIA*numA * rnA * rnA + thisContact.invIB * numB * rnB * rnB;
	//float kNormal = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rnA * rnA + thisContact.invIB * rnB * rnB;
	currentPoints.normalMass1 = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
	//printf ("currentpoints.normal Mass1: %f, \n" , kNormal); 

	float2 tangent = b2clCross_VS(normal, 1.0f);
	float rtA = b2clCross_VV(currentPoints.rA1, tangent);
	float rtB = b2clCross_VV(currentPoints.rB1, tangent);
	float kTangent = thisContact.invMassA * numA + thisContact.invMassB * numB + thisContact.invIA * numA * rtA * rtA + thisContact.invIB * numB * rtB * rtB;
	//float kTangent = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rnA * rnA + thisContact.invIB * rnB * rnB;
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
	float mixedrestitution = b2clMixRestitution(restitutionA, restitutionB);

	currentPoints.velocityBias1 = 0.0f;
	float vRel = b2clDot(normal, vB + b2clCross_SV(wB, currentPoints.rB1) - vA - b2clCross_SV(wA, currentPoints.rA1));
	if (vRel < -b2cl_velocityThreshold)
	{
		currentPoints.velocityBias1 = -mixedrestitution * vRel;
	}

	// This part is only useful for first frame.
	// In later frames, impulse will be reset in ReadLastImpulse
	if (warmStarting)
	{
		currentImpulses.normalImpulse1 = dtRatio * manifold.points[0].normalImpulse;
		currentImpulses.tangentImpulse1 = dtRatio * manifold.points[0].tangentImpulse;
	}
	else
	{
		currentImpulses.normalImpulse1 = 0.0f;
		currentImpulses.tangentImpulse1 = 0.0f;
	}

    // Extract useful manifold information for the position constraint solver
    currentManifolds.localNormal = manifold.localNormal;
    currentManifolds.localPoint = manifold.localPoint;
    currentManifolds.type=manifold.type;
    currentManifolds.pointCount=manifold.pointCount;
    currentManifolds.localPoints1=manifold.points[0].localPoint;
    currentManifolds.radiusA=radiusA;
    currentManifolds.radiusB=radiusB;
    currentManifolds.localCenterA=bodyStaticA.m_localCenter;
    currentManifolds.localCenterB=bodyStaticB.m_localCenter;
    
	if (manifold.pointCount>1)
	{
		currentPoints.rA2 = worldManifoldPoints[1] - (float2)(positionA.cx, positionA.cy);
		currentPoints.rB2 = worldManifoldPoints[1] - (float2)(positionB.cx, positionB.cy);

		float rnA = b2clCross_VV(currentPoints.rA2, normal);
		float rnB = b2clCross_VV(currentPoints.rB2, normal);
		float kNormal = thisContact.invMassA * numA + thisContact.invMassB * numB + thisContact.invIA * numA * rnA * rnA + thisContact.invIB *numB * rnB * rnB;
		//float kNormal = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rnA * rnA + thisContact.invIB * rnB * rnB;
		currentPoints.normalMass2 = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

		float2 tangent = b2clCross_VS(normal, 1.0f);
		float rtA = b2clCross_VV(currentPoints.rA2, tangent);
		float rtB = b2clCross_VV(currentPoints.rB2, tangent);
		float kTangent = thisContact.invMassA * numA + thisContact.invMassB * numB + thisContact.invIA * numA * rtA * rtA + thisContact.invIB * numB * rtB * rtB;
		//float kTangent = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rnA * rnA + thisContact.invIB * rnB * rnB;
		currentPoints.tangentMass2 = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;

		currentPoints.velocityBias2 = 0.0f;
		float vRel = b2clDot(normal, vB + b2clCross_SV(wB, currentPoints.rB2) - vA - b2clCross_SV(wA, currentPoints.rA2));
		if (vRel < -b2cl_velocityThreshold)
		{
			currentPoints.velocityBias2 = -mixedrestitution * vRel;
		}

		// This part is only useful for first frame.
		// In later frames, impulse will be reset in ReadLastImpulse
		if (warmStarting)
		{
			currentImpulses.normalImpulse2 = dtRatio * manifold.points[1].normalImpulse;
			currentImpulses.tangentImpulse2 = dtRatio * manifold.points[1].tangentImpulse;
		}
		else
		{
			currentImpulses.normalImpulse2 = 0.0f;
			currentImpulses.tangentImpulse2 = 0.0f;
		}

		currentManifolds.localPoints2=manifold.points[1].localPoint;
	}
	else
	{
		currentPoints.rA2 = -1;
		currentPoints.rB2 = -1;
		currentPoints.normalMass2 = -1;
		currentPoints.tangentMass2 = -1;
		currentPoints.velocityBias2 = -1;
		currentImpulses.normalImpulse2 = 0;
		currentImpulses.tangentImpulse2 = 0;
	}

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

	// This part is only useful for first frame.
	// In later frames, impulse will be reset in ReadLastImpulse
	impulses[contactIndex] = currentImpulses;
    
    manifoldsForPositionSolver[contactIndex] = currentManifolds;

	//if (currentIndices.x<currentIndices.y)
	//	manifoldKeys[contactIndex] = currentIndices.x<<16 | currentIndices.y; // suppose both indices are less than 65536 and thus fit in 16 bits.
	//else
	//	manifoldKeys[contactIndex] = currentIndices.y<<16 | currentIndices.x; // suppose both indices are less than 65536 and thus fit in 16 bits.
	//globalIndices[contactIndex] = globalIndex;
}

__kernel void StoreImpulses(
                        __global b2clManifold* manifolds, // input and output
						__global uint* manifoldKeys, // output, keys to be sorted
						__global uint* globalIndices, // output, values to be sorted
                        const __global int* coloredContactIndexToContactIndexMap,
                        const __global clb2Impulse* impulses,
						const __global int4* indices, //indices to bodies and fixtures
                        const int contactCount)
{
    unsigned int contactIndex = get_global_id(0);

	if(contactIndex >= contactCount) return;
    
    int globalIndex = coloredContactIndexToContactIndexMap[contactIndex];
   
    b2clManifold manifold = manifolds[globalIndex];
    clb2Impulse currentImpulses = impulses[contactIndex];

    manifold.points[0].normalImpulse = currentImpulses.normalImpulse1;
    manifold.points[0].tangentImpulse = currentImpulses.tangentImpulse1;

	//printf ("Store Impulse:  %f \n", manifold.points[0].normalImpulse ) ; 
    
    if(manifold.pointCount>1)
	{
        manifold.points[1].normalImpulse = currentImpulses.normalImpulse2;
        manifold.points[1].tangentImpulse = currentImpulses.tangentImpulse2;
    }
	else
	{
        manifold.points[1].normalImpulse = 0;
        manifold.points[1].tangentImpulse = 0;
    }
    
	//printf("pointCount: %d, normalImpuls: %f, tangentImpulse: %f\n", manifold.pointCount, manifold.points[0].normalImpulse, manifold.points[0].tangentImpulse);
    manifolds[globalIndex] = manifold;

	int4 currentIndices = indices[globalIndex];

 
	if (currentIndices.x<currentIndices.y)
		manifoldKeys[contactIndex] = currentIndices.x<<16 | currentIndices.y; // suppose both indices are less than 65536 and thus fit in 16 bits.
	else
		manifoldKeys[contactIndex] = currentIndices.y<<16 | currentIndices.x; // suppose both indices are less than 65536 and thus fit in 16 bits.

	//if (currentIndices.z<currentIndices.w)
	//	manifoldKeys[contactIndex] = currentIndices.z<<16 | currentIndices.w; // suppose both indices are less than 65536 and thus fit in 16 bits.
	//else
	//	manifoldKeys[contactIndex] = currentIndices.w<<16 | currentIndices.z; // suppose both indices are less than 65536 and thus fit in 16 bits.
	globalIndices[contactIndex] = globalIndex;
}


__kernel void SolveVelocityConstraintComplex(
                                 __global clb2Velocity* velocities,
                                 __global clb2Contact* contacts,
                                 __global clb2Impulse* impulses,
                                 __global clb2Points* points,
								 //__global float* testArray, 
                                 const unsigned int offset,
                                 const unsigned int colorLength
                                 
								 )

{
    unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) return;

    clb2Contact thisContact = contacts[contactIndex];
    clb2Impulse thisImpulse = impulses[contactIndex];
    clb2Points thisPoints = points[contactIndex];
    clb2Velocity thisVelA = velocities[thisContact.indexA];
    clb2Velocity thisVelB = velocities[thisContact.indexB];

	//if (thisContact.indexA == 2 && thisContact.indexB == 1) {
	//	//printf ("2 and 1 , return " ); 
	//	return ; 
	//}



	// Computer K ; 
	b2clMat22 K ; 
	b2clMat22 normalMassMat ; 
	if (thisPoints.normalMass2 >=0 ) {
		   float rn1A = thisPoints.rA1.x * thisContact.normal.y - thisPoints.rA1.y * thisContact.normal.x ;   
		   float rn1B = thisPoints.rB1.x * thisContact.normal.y - thisPoints.rB1.y * thisContact.normal.x ;
		   float rn2A = thisPoints.rA2.x * thisContact.normal.y - thisPoints.rA2.y * thisContact.normal.x ;
		   float rn2B = thisPoints.rB2.x * thisContact.normal.y - thisPoints.rB2.y * thisContact.normal.x ;
		   float k11 = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rn1A * rn1A + thisContact.invIB * rn1B * rn1B ; 
		   float k22 = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rn2A * rn2A + thisContact.invIB * rn2B * rn2B ; 
		   float k12 = thisContact.invMassA + thisContact.invMassB + thisContact.invIA * rn1A * rn2A + thisContact.invIB * rn1B * rn2B ; 
		    
           float k_maxConditionNumber = 1000.0f ; 
		   bool ifcase = (  k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12) ) ; 
           if ( ifcase ) {
			K.ex[0] = k11 ; K.ex[1] = k12 ; 
			K.ey[0] = k12 ; K.ey[1] = k22 ; 
			b2clMat22GetInverse ( K , &normalMassMat ) ; 
		   }
		   else {
			thisPoints.normalMass2 = -1 ; 
		   }
    }



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

//float kTangent = thisContact.invMassA * numA + thisContact.invMassB * numB + thisContact.invIA * numA * rtA * rtA + thisContact.invIB * numB * rtB * rtB;

    
    float2 P; 
    P.x = lambda*thisContact.normal.y;
    P.y = -lambda*thisContact.normal.x;

	//if (thisContact.indexA == 1 && thisContact.indexB == 0) {
	//	printf ("indexA: %d \n", thisContact.indexA); 
	//}

	
    thisVelA.vx = thisVelA.vx-thisContact.invMassA * P.x; 
    thisVelA.vy = thisVelA.vy-thisContact.invMassA * P.y; 

	//printf ("after thisVelA.vy: %f \n", thisVelA.vy ); 
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
	if ( thisPoints.normalMass2 <0 ) {
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
	}
	else {

	       float2 a; 
		   a.x = thisImpulse.normalImpulse1;
		   a.y = thisImpulse.normalImpulse2;

           float2 dv1 , dv2 ; 
		   dv1.x = thisVelB.vx-thisVelB.w * thisPoints.rB1.y-thisVelA.vx+thisVelA.w * thisPoints.rA1.y;
		   dv1.y = thisVelB.vy+thisVelB.w * thisPoints.rB1.x-thisVelA.vy-thisVelA.w * thisPoints.rA1.x;
		   dv2.x = thisVelB.vx-thisVelB.w * thisPoints.rB2.y-thisVelA.vx+thisVelA.w * thisPoints.rA2.y;
           dv2.y = thisVelB.vy+thisVelB.w * thisPoints.rB2.x-thisVelA.vy-thisVelA.w * thisPoints.rA2.x;

		   float vn1 , vn2 ; 
		   vn1 = dv1.x*thisContact.normal.x+dv1.y*thisContact.normal.y;
		   vn2 = dv2.x*thisContact.normal.x+dv2.y*thisContact.normal.y;

		   float2 b ; 

		   b.x = vn1 - thisPoints.velocityBias1 ; 
		   b.y = vn2 - thisPoints.velocityBias2 ; 

		   float2 Ka  = b2clMat22Mul ( K , a ) ; 
		   b.x = b.x - Ka.x  ; b.y = b.y - Ka.y ; 
		   float k_errorTol = 1e-3f ; 

		   for (;;)
		   {
			float2 x = b2clMat22Mul (normalMassMat, b); x.x = 0 - x.x ; x.y = 0- x.y ; 
				
				if (x.x >= 0.0f && x.y >= 0.0f)
				{

				    //float2 vA, wA, vB, wB ; 

					//vA.x = thisVelA.vx ; vA.y = thisVelA.vy ; wA = thisVelA.w ; 
					//vB.x = thisVelB.vx ; vB.y = thisVelB.vy ; wB = thisVelB.w ;
					 
					// Get the incremental impulse
					float2 d ;
					d = x - a;  
				    
					// Apply incremental impulse
					float2 P1 , P2 ; 
					P1.x = d.x * thisContact.normal.x ; P1.y = d.x * thisContact.normal.y ; 
					P2.x = d.y * thisContact.normal.x ; P2.y = d.y * thisContact.normal.y ; 



					thisVelA.vx = thisVelA.vx - thisContact.invMassA * (P1.x + P2.x);
					thisVelA.vy = thisVelA.vy - thisContact.invMassA * (P1.y + P2.y);
					float cr_rA1P1 = b2clCross_VV (thisPoints.rA1, P1 ) ; 
					float cr_rA2P2 = b2clCross_VV (thisPoints.rA2, P2 ) ;
					thisVelA.w = thisVelA.w - thisContact.invIA * (cr_rA1P1 + cr_rA2P2); 

					thisVelB.vx = thisVelB.vx + thisContact.invMassB * (P1.x + P2.x);
					thisVelB.vy = thisVelB.vy + thisContact.invMassB * (P1.y + P2.y);
					float cr_rB1P1 = b2clCross_VV (thisPoints.rB1, P1 ) ;
					float cr_rB2P2 = b2clCross_VV (thisPoints.rB2, P2 ) ;
					thisVelB.w = thisVelB.w + thisContact.invIB * (cr_rB1P1 + cr_rB2P2); 

					//printf ("thisVelA.vy: %f \n", thisVelA.vy);

					thisImpulse.normalImpulse1 = x.x ; 
					thisImpulse.normalImpulse2 = x.y ; 
					break;
				}
				x.x = 0 - thisPoints.normalMass1 * b.x ; 
				x.y = 0.0f ;
				vn1 = 0.0f ; 
				vn2 = K.ex[1] * x.x + b.y ; 
				
				if (x.x >= 0.0f && vn2 >= 0.0f)
				{
					float2 d ;
					d = x - a;  
				    
					// Apply incremental impulse
					float2 P1 , P2 ; 
					P1.x = d.x * thisContact.normal.x ; P1.y = d.x * thisContact.normal.y ; 
					P2.x = d.y * thisContact.normal.x ; P2.y = d.y * thisContact.normal.y ; 



					thisVelA.vx = thisVelA.vx - thisContact.invMassA * (P1.x + P2.x);
					
					thisVelA.vy = thisVelA.vy - thisContact.invMassA * (P1.y + P2.y);
                    
					float cr_rA1P1 = b2clCross_VV (thisPoints.rA1, P1 ) ; 
					float cr_rA2P2 = b2clCross_VV (thisPoints.rA2, P2 ) ;
					thisVelA.w = thisVelA.w - thisContact.invIA * (cr_rA1P1 + cr_rA2P2); 

					thisVelB.vx = thisVelB.vx + thisContact.invMassB * (P1.x + P2.x);
					thisVelB.vy = thisVelB.vy + thisContact.invMassB * (P1.y + P2.y);
					float cr_rB1P1 = b2clCross_VV (thisPoints.rB1, P1 ) ;
					float cr_rB2P2 = b2clCross_VV (thisPoints.rB2, P2 ) ;
					thisVelB.w = thisVelB.w + thisContact.invIB * (cr_rB1P1 + cr_rB2P2); 

					//printf ("thisVelA.vy: %f \n", thisVelA.vy);

					thisImpulse.normalImpulse1 = x.x ; 
					thisImpulse.normalImpulse2 = x.y ; 
					break;
				}
				x.x = 0.0f;
				x.y = 0 - thisPoints.normalMass2 * b.y;
				vn1 = K.ey[1]*x.y + b.x ; 
				vn2 = 0.0f;
				if (x.y >= 0.0f && vn1 >= 0.0f)
				{
					float2 d ;
					d = x - a;  
				    
					// Apply incremental impulse
					float2 P1 , P2 ; 
					P1.x = d.x * thisContact.normal.x ; P1.y = d.x * thisContact.normal.y ; 
					P2.x = d.y * thisContact.normal.x ; P2.y = d.y * thisContact.normal.y ; 

					thisVelA.vx = thisVelA.vx - thisContact.invMassA * (P1.x + P2.x);
					thisVelA.vy = thisVelA.vy - thisContact.invMassA * (P1.y + P2.y);
					float cr_rA1P1 = b2clCross_VV (thisPoints.rA1, P1 ) ; 
					float cr_rA2P2 = b2clCross_VV (thisPoints.rA2, P2 ) ;
					thisVelA.w = thisVelA.w - thisContact.invIA * (cr_rA1P1 + cr_rA2P2); 

					thisVelB.vx = thisVelB.vx + thisContact.invMassB * (P1.x + P2.x);
					thisVelB.vy = thisVelB.vy + thisContact.invMassB * (P1.y + P2.y);
					float cr_rB1P1 = b2clCross_VV (thisPoints.rB1, P1 ) ;
					float cr_rB2P2 = b2clCross_VV (thisPoints.rB2, P2 ) ;
					thisVelB.w = thisVelB.w + thisContact.invIB * (cr_rB1P1 + cr_rB2P2); 

					//printf ("thisVelA.vy: %f \n", thisVelA.vy);

					thisImpulse.normalImpulse1 = x.x ; 
					thisImpulse.normalImpulse2 = x.y ; 
					break;
				}
				x.x = 0.0f ; 
				x.y =0.0f;
				vn1 = b.x ; 
				vn2 = b.y ; 
				if (vn1 >=0.0f && vn2 >=0.0f)
				{
					float2 d ;
					d = x - a;  
				    
					// Apply incremental impulse
					float2 P1 , P2 ; 
					P1.x = d.x * thisContact.normal.x ; P1.y = d.x * thisContact.normal.y ; 
					P2.x = d.y * thisContact.normal.x ; P2.y = d.y * thisContact.normal.y ; 

					thisVelA.vx = thisVelA.vx - thisContact.invMassA * (P1.x + P2.x);
					thisVelA.vy = thisVelA.vy - thisContact.invMassA * (P1.y + P2.y);
					float cr_rA1P1 = b2clCross_VV (thisPoints.rA1, P1 ) ; 
					float cr_rA2P2 = b2clCross_VV (thisPoints.rA2, P2 ) ;
					thisVelA.w = thisVelA.w - thisContact.invIA * (cr_rA1P1 + cr_rA2P2); 

					thisVelB.vx = thisVelB.vx + thisContact.invMassB * (P1.x + P2.x);
					thisVelB.vy = thisVelB.vy + thisContact.invMassB * (P1.y + P2.y);
					float cr_rB1P1 = b2clCross_VV (thisPoints.rB1, P1 ) ;
					float cr_rB2P2 = b2clCross_VV (thisPoints.rB2, P2 ) ;
					thisVelB.w = thisVelB.w + thisContact.invIB * (cr_rB1P1 + cr_rB2P2); 
					//printf ("thisVelA.vy: %f \n", thisVelA.vy);

					thisImpulse.normalImpulse1 = x.x ; 
					thisImpulse.normalImpulse2 = x.y ; 
					break;
				}
				break ; 
		   }
	}

    // Copy the results back to the buffers
    impulses[contactIndex] = thisImpulse;
    velocities[thisContact.indexA] = thisVelA;
    velocities[thisContact.indexB] = thisVelB;
}







__kernel void SolveVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global clb2Contact* contacts,
                                 __global clb2Impulse* impulses,
                                 __global clb2Points* points,
								 //__global float* testArray, 
                                 const unsigned int offset,
                                 const unsigned int colorLength
                                 
								 )

{
    unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) return;

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

//float kTangent = thisContact.invMassA * numA + thisContact.invMassB * numB + thisContact.invIA * numA * rtA * rtA + thisContact.invIB * numB * rtB * rtB;

    
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
	//testArray[7] = thisVelA.w ; testArray[8] = thisVelB.w ; testArray[9] = P.y ;testArray[10]= thisVelB.vy; 

	//testArray[0] = thisVelA.w ; testArray[1] = thisVelB.w ;

  // testArray[contactIndex* 20+0] = P.x ; testArray[contactIndex* 20+1] = P.y ;
 //  testArray[contactIndex* 20+2] = 0 -thisContact.invMassB * P.x; testArray[contactIndex* 20+3] = 0 - thisContact.invMassB * P.y; 
    
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
    }
    
    // Copy the results back to the buffers
    impulses[contactIndex] = thisImpulse;
    velocities[thisContact.indexA] = thisVelA;
    velocities[thisContact.indexB] = thisVelB;
}









__kernel void SolveSplitImpulseVelocityConstraint(
                                 __global clb2Velocity* velocities,
								 //__global clb2Velocity* oldVelocities,
								 __global clb2Velocity* splitVelocities,
                                 __global clb2Contact* contacts,
                                 __global clb2Impulse* impulses,
								// __global clb2Impulse* splitImpulses, 
                                 __global clb2Points* points,
								 //__global float* testBuffer, 
								 __global unsigned int* numContacts4EachBody,
								 __global unsigned int* indexContact2BodySplitVelocity,
								 unsigned int contactCount  
								 
								 //unsigned int lastIter
                                )
{
    unsigned int contactIndex = get_global_id(0);
	if (contactIndex >= contactCount) return ; 

    clb2Contact thisContact = contacts[contactIndex];
	clb2Impulse thisOldImpulse = impulses[contactIndex];  
    clb2Points thisPoints = points[contactIndex];

    clb2Velocity thisVelA = velocities[thisContact.indexA];
    clb2Velocity thisVelB = velocities[thisContact.indexB];

	//clb2Velocity thisOldVelA = oldVelocities[thisContact.indexA];
	//clb2Velocity thisOldVelB = oldVelocities[thisContact.indexB];

		/////////////////////////////////////
	//make invIB small
	//thisContact.invIA = thisContact.invIB = 1 ; 

	/////////////////////////////////////


	unsigned int indexContact2BodyA = indexContact2BodySplitVelocity[contactIndex*2+0];
	unsigned int indexContact2BodyB = indexContact2BodySplitVelocity[contactIndex*2+1]; 
	int numBodyA = numContacts4EachBody [thisContact.indexA];
	int numBodyB = numContacts4EachBody [thisContact.indexB];
	int maxContactNumPerBody = 20 ;
	bool isGround[2] ; isGround[0] = thisContact.invMassA == 0 ? true:false ; isGround[1] = thisContact.invMassB == 0 ? true: false ; 
	if (isGround[0])  { thisPoints.rA1.x = thisPoints.rA1.y = thisPoints.rA2.x = thisPoints.rA2.y = 0 ; } 
	if (isGround[1])  { thisPoints.rB1.x = thisPoints.rB1.y = thisPoints.rB2.x = thisPoints.rB2.y = 0 ; }

	clb2Impulse thisImpulse; 
	thisImpulse = impulses[contactIndex]; 

    // update old velocities using this new velocities. 
	//oldVelocities[thisContact.indexA] = thisVelA ; oldVelocities[thisContact.indexB] = thisVelB ;
	// update old impulse using this new impulse. 
	impulses[contactIndex] = thisImpulse ; 

    clb2Velocity deltaVA, deltaVB ; 
	deltaVA.vx = deltaVA.vy = deltaVA.w =0 ; deltaVB.vx = deltaVB.vy = deltaVB.w = 0 ; 

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
	float delta_x, delta_y, delta_w ; 


	//delta_x = thisContact.invMassA * P.x  ; delta_y = thisContact.invMassA * P.y ; delta_w = thisContact.invIA * (thisPoints.rA1.x * P.y - thisPoints.rA1.y * P.x) ; 

	delta_x = thisContact.invMassA*numBodyA * P.x  ; delta_y = thisContact.invMassA *numBodyA * P.y ; delta_w = thisContact.invIA*numBodyA * (thisPoints.rA1.x * P.y - thisPoints.rA1.y * P.x) ; 
	deltaVA.vx -= delta_x ; deltaVA.vy -= delta_y ; deltaVA.w -= delta_w ; 
	thisVelA.vx -= delta_x ; thisVelA.vy -= delta_y ; thisVelA.w -= delta_w ; 
	 
	//delta_x = thisContact.invMassB  * P.x ; delta_y = thisContact.invMassB  * P.y; delta_w = thisContact.invIB * (thisPoints.rB1.x * P.y - thisPoints.rB1.y * P.x) ; 
	delta_x = thisContact.invMassB *numBodyB * P.x ; delta_y = thisContact.invMassB *numBodyB * P.y; delta_w = thisContact.invIB*numBodyB * (thisPoints.rB1.x * P.y - thisPoints.rB1.y * P.x) ; 
    deltaVB.vx += delta_x ; deltaVB.vy += delta_y ; deltaVB.w += delta_w ; 
	thisVelB.vx += delta_x ; thisVelB.vy += delta_y ; thisVelB.w += delta_w ;  
    
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
        

	 // delta_x = thisContact.invMassA * P.x ; delta_y = thisContact.invMassA * P.y ; delta_w = thisContact.invIA * (thisPoints.rA2.x * P.y - thisPoints.rA2.y * P.x);   
	 delta_x = thisContact.invMassA *numBodyA * P.x ; delta_y = thisContact.invMassA *numBodyA * P.y ; delta_w = thisContact.invIA *numBodyA * (thisPoints.rA2.x * P.y - thisPoints.rA2.y * P.x);  
      deltaVA.vx -= delta_x ; deltaVA.vy -= delta_y ; deltaVA.w -= delta_w; 
	  thisVelA.vx -= delta_x ; thisVelA.vy -= delta_y ; thisVelA.w -= delta_w ; 

	 // delta_x = thisContact.invMassB * P.x ; delta_y = thisContact.invMassB * P.y ; delta_w = thisContact.invIB * (thisPoints.rB2.x * P.y - thisPoints.rB2.y * P.x) ; 
	 delta_x = thisContact.invMassB * numBodyB * P.x ; delta_y = thisContact.invMassB * numBodyB*  P.y ; delta_w = thisContact.invIB * numBodyB *  (thisPoints.rB2.x * P.y - thisPoints.rB2.y * P.x) ; 
	  deltaVB.vx += delta_x ; deltaVB.vy += delta_y; deltaVB.w += delta_w ; 
	  thisVelB.vx += delta_x ; thisVelB.vy += delta_y ; thisVelB.w += delta_w ; 
	  
	
    }
    

    // Calculate the normal impulse for the first contact point
    dv.x = thisVelB.vx-thisVelB.w * thisPoints.rB1.y-thisVelA.vx+thisVelA.w * thisPoints.rA1.y;
    dv.y = thisVelB.vy+thisVelB.w * thisPoints.rB1.x-thisVelA.vy-thisVelA.w * thisPoints.rA1.x;
	   
    //printf ("this Impulse lambda: %f \n", thisImpulse.normalImpulse1);
	//printf ("dv.y: %f \n", dv.y);

    vp = dv.x*thisContact.normal.x+dv.y*thisContact.normal.y;
	
    lambda = -thisPoints.normalMass1 * (vp-thisPoints.velocityBias1);
	//printf ("this Impulse lambda: %f \n", lambda);
   // printf ("normalMass1: %f \n", thisPoints.normalMass1);

	float lv = 1.0 ; 

    newImpulse=fmax(thisImpulse.normalImpulse1+lambda,0);
    lambda=newImpulse-thisImpulse.normalImpulse1;
    thisImpulse.normalImpulse1=newImpulse;

    
    P.x = lambda*thisContact.normal.x;
    P.y = lambda*thisContact.normal.y; 
  
	
	//delta_x = thisContact.invMassA * P.x; delta_y = thisContact.invMassA * P.y ; delta_w = thisContact.invIA * (thisPoints.rA1.x * P.y - thisPoints.rA1.y * P.x) ; 
	delta_x = thisContact.invMassA *numBodyA * P.x; delta_y = thisContact.invMassA * numBodyA * P.y ; delta_w = thisContact.invIA *numBodyA * (thisPoints.rA1.x * P.y - thisPoints.rA1.y * P.x) ; 
    deltaVA.vx -= delta_x ; deltaVA.vy -= delta_y ; deltaVA.w -= delta_w ; 
	thisVelA.vx -= delta_x ; thisVelA.vy -= delta_y ; thisVelA.w -= delta_w ; 

	//printf ("P_y: %f \n", P.y);

	//delta_x = thisContact.invMassB * P.x ; delta_y = thisContact.invMassB * P.y ; delta_w = thisContact.invIB * (thisPoints.rB1.x * P.y - thisPoints.rB1.y * P.x) ; 
	delta_x = thisContact.invMassB * numBodyB * P.x ; delta_y = thisContact.invMassB * numBodyB * P.y ; delta_w = thisContact.invIB * numBodyB* (thisPoints.rB1.x * P.y - thisPoints.rB1.y * P.x) ; 
    deltaVB.vx += delta_x ; deltaVB.vy += delta_y ; deltaVB.w += delta_w ; 
	thisVelB.vx += delta_x ; thisVelB.vy += delta_y ; thisVelB.w += delta_w ;
	
	 
	              	 
    // Calculate the normal impulse for the second contact point if there is the second contact point
	
    if(thisPoints.normalMass2>=0)
	{
	
        dv.x = thisVelB.vx-thisVelB.w * thisPoints.rB2.y-thisVelA.vx+thisVelA.w * thisPoints.rA2.y;
        dv.y = thisVelB.vy+thisVelB.w * thisPoints.rB2.x-thisVelA.vy-thisVelA.w * thisPoints.rA2.x;
        
        vp = dv.x*thisContact.normal.x+dv.y*thisContact.normal.y;
        lambda = -thisPoints.normalMass2 * (vp-thisPoints.velocityBias2);
		//printf ("this Impulse lambda: %f \n", lambda);
        

        newImpulse=fmax(thisImpulse.normalImpulse2+lambda,0);	
        lambda=newImpulse-thisImpulse.normalImpulse2;
        thisImpulse.normalImpulse2=newImpulse;
        
        P.x = lambda*thisContact.normal.x;
        P.y = lambda*thisContact.normal.y;


     // delta_x = thisContact.invMassA * P.x; delta_y = thisContact.invMassA * P.y ; delta_w = thisContact.invIA * (thisPoints.rA2.x * P.y - thisPoints.rA2.y * P.x) ; 
	 delta_x = thisContact.invMassA * numBodyA * P.x; delta_y = thisContact.invMassA * numBodyA*  P.y ; delta_w = thisContact.invIA * numBodyA * (thisPoints.rA2.x * P.y - thisPoints.rA2.y * P.x) ; 
      deltaVA.vx -= delta_x ; deltaVA.vy -= delta_y ; deltaVA.w -= delta_w ; 
	  thisVelA.vx -= delta_x ; thisVelA.vy -= delta_y ; thisVelA.w -= delta_w; 
	  
	  //delta_x = thisContact.invMassB * P.x ; delta_y = thisContact.invMassB * P.y; delta_w = thisContact.invIB * (thisPoints.rB2.x * P.y - thisPoints.rB2.y * P.x);
	  delta_x = thisContact.invMassB * numBodyB * P.x ; delta_y = thisContact.invMassB * numBodyB * P.y; delta_w = thisContact.invIB * numBodyB * (thisPoints.rB2.x * P.y - thisPoints.rB2.y * P.x);
      deltaVB.vx += delta_x ; deltaVB.vy += delta_y ; deltaVB.w += delta_w ; 
	  thisVelB.vx += delta_x ; thisVelB.vy += delta_y ; thisVelB.w += delta_w ; 	  
    }
    // Copy the results back to the buffers

	if (!isGround[0]) {
		//deltaVA.vx *= numBodyA ; deltaVA.vy *= numBodyA ; deltaVA.w *= numBodyA ; 
		splitVelocities [thisContact.indexA * maxContactNumPerBody + indexContact2BodyA] = deltaVA ; 
		//printf ("numBodyA: %f , \n", numBodyA) ; 
	}
	if (!isGround[1]) {
		//deltaVB.vx *= numBodyB ; deltaVB.vy *= numBodyB ; deltaVB.w *= numBodyB ; 
		splitVelocities [thisContact.indexB * maxContactNumPerBody + indexContact2BodyB] = deltaVB ; 
		//printf ("deltaB.vy: %f , \n", deltaVB.vy) ; 	
	}
  
    impulses[contactIndex] = thisImpulse;
	//printf ("thisImpulse: %f , \n", thisImpulse.normalImpulse1) ; 
}





__kernel void SolveMergeVelocityConstraint(
                                 __global clb2Velocity* velocities,
								 __global clb2Velocity* splitVelocities,
                                 __global clb2Contact* contacts,
                                 __global clb2Impulse* impulses,
                                 __global clb2Points* points,
								 __global unsigned int* numContacts4EachBody,
								 __global unsigned int* indexContact2BodySplitVelocity,
								 const uint bodyCount
                                )
{

    unsigned int bodyIndex = get_global_id(0);
	if (bodyIndex >= bodyCount) return ; 
	clb2Velocity thisBodyVelocity = velocities[bodyIndex];
	unsigned int numContacts4Body = numContacts4EachBody[bodyIndex]; 
	clb2Velocity  deltaMergedVelocity; deltaMergedVelocity.vx = deltaMergedVelocity.vy = deltaMergedVelocity.w = 0 ; 
	//__global clb2Velocity* pDeltaSplitVelocity  ; 
	clb2Velocity  thisSplitVelocity ; 

	int maxContactNumPerBody = 20 ; 
	for ( int i = 0 ; i < numContacts4Body; i ++ ) {
		thisSplitVelocity = splitVelocities[bodyIndex*maxContactNumPerBody+i] ; 
		deltaMergedVelocity.vx += thisSplitVelocity.vx ; deltaMergedVelocity.vy += thisSplitVelocity.vy ; deltaMergedVelocity.w += thisSplitVelocity.w ;
		//printf ("SplitedVelocitySpeed: %f \n", thisSplitVelocity.vy);  
	} 
	if (numContacts4Body > 0 ) {
		deltaMergedVelocity.vx /= numContacts4Body; deltaMergedVelocity.vy /= numContacts4Body ;  	deltaMergedVelocity.w /= numContacts4Body ;
	} 
	thisBodyVelocity.vx += deltaMergedVelocity.vx ; 
	thisBodyVelocity.vy += deltaMergedVelocity.vy ;
	thisBodyVelocity.w += deltaMergedVelocity.w ;
	velocities[bodyIndex] = thisBodyVelocity; 
	//printf ("MergeVelocitySpeed: %f \n", thisBodyVelocity.vy); 

}






__kernel void SynchronizeXf(
							__global b2clTransform* xfGlobal, // output
                            __global clb2Position* positions,
							const __global b2clBodyStatic* bodyStaticListBuffer,
							__global b2clBodyDynamic* bodyDynamicListBuffer,
                            const unsigned int num_body)
{
    unsigned int i = get_global_id(0);

    if(i >= num_body) return;

	b2clBodyStatic bs = bodyStaticListBuffer[i];
	__global b2clBodyDynamic* bd = bodyDynamicListBuffer + i;

	float2 c;
	c.x = positions[i].cx;
	c.y = positions[i].cy;
	float a = positions[i].a;

	// synchronize xf for each body
	float2 p, q;
	
	float sina, cosa;
	sina = sincos(a, &cosa);
	q.x = sina;
	q.y = cosa; 
	//q.x = /*native_*/sin(a);
	//q.y = cos(a);
	//q.y = cos_wrapper(a);
	p = c - b2clMul_Rotate(q, bs.m_localCenter);
	xfGlobal[i].p = p;
	xfGlobal[i].q = q;

	bd->m_sweep.c = c;
	bd->m_sweep.a = a;
}

__kernel void ReadLastImpulses(
                                 __global clb2Impulse* impulses, // output
								 __global uint* lastManifoldKeys, // input, sorted keys
								 __global uint* lastGlobalIndices, // input, sorted values
								 const unsigned int lastContactCount,
                                 const unsigned int contactCount,
						const __global int4* indices, //indices to bodies and fixtures
						const __global int* coloredContactIndexToContactIndexMap,
						const __global b2clManifold* manifolds,
						const __global b2clManifold* lastManifolds,
						const int warmStarting,
						const float dtRatio)
{
    unsigned int contactIndex = get_global_id(0);

    if(contactIndex >= contactCount) return;

	clb2Impulse currentImpulses;

	if (!warmStarting)
	{
		currentImpulses.normalImpulse1 = 0.0f;
		currentImpulses.tangentImpulse1 = 0.0f;
		currentImpulses.normalImpulse2 = 0.0f;
		currentImpulses.tangentImpulse2 = 0.0f;

		impulses[contactIndex] = currentImpulses;

		return;
	}

    int globalIndex = coloredContactIndexToContactIndexMap[contactIndex];
	int4 currentIndices = indices[globalIndex];


	int currentKey;
	if (currentIndices.x<currentIndices.y)
		currentKey = currentIndices.x<<16 | currentIndices.y; // suppose both indices are less than 65536 and thus fit in 16 bits.
	else
		currentKey = currentIndices.y<<16 | currentIndices.x; // suppose both indices are less than 65536 and thus fit in 16 bits.

	//if (currentIndices.z<currentIndices.w)
	//	currentKey = currentIndices.z<<16 | currentIndices.w; // suppose both indices are less than 65536 and thus fit in 16 bits.
	//else
	//	currentKey = currentIndices.w<<16 | currentIndices.z; // suppose both indices are less than 65536 and thus fit in 16 bits.

	// binary saerch currentKey in lastManifoldKeys
	int lb = 0; // low bound
	int ub = lastContactCount-1; // upper bound
	int mid;
	bool bFound = false;
	uint midKey;
	//if (lastManifoldKeys[lb]<currentKey || lastManifoldKeys[ub]>currentKey)
	//	// currentKey not in lastManifoldKeys
	//	bFound = false;
	//else // do binary search
	{
		for (; lb<ub; )
		{
			mid = (lb+ub)/2;
			midKey = lastManifoldKeys[mid];
			if (midKey==currentKey)
			{
				bFound = true;
				break;
			}
			else if (midKey<currentKey)
			{
				ub = mid-1;
			}
			else
			{
				lb = mid+1;
			}
		}
		if (ub<lb)
			bFound = false;
	}
	if (lb==ub)
	{
		mid = lb;
      //  if (currentKey == lastManifoldKeys[mid] ) bFound = true ; 
	   // else bFound = false;
		bFound = true;
	}

	if (bFound)
	{
		//printf ("bFound! \n" ); 
		b2clManifold currentManifold = manifolds[globalIndex];
		for (int k = 0; k < currentManifold.pointCount; ++k)
		{
			b2clManifoldPoint* mp2 = currentManifold.points + k;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			int key2 = mp2->id.key;

			int lastGlobalIndex = lastGlobalIndices[mid];
			b2clManifold lastManifold = lastManifolds[lastGlobalIndex];
			//printf("last global index: %d\n", lastGlobalIndex);
			for (int j = 0; j < lastManifold.pointCount; ++j)
			{
				b2clManifoldPoint* mp1 = lastManifold.points + j;

				//if (mp1->id.key == key2)
				//{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					break;
				//}
			}
		}

		currentImpulses.normalImpulse1 = dtRatio * currentManifold.points[0].normalImpulse;
		currentImpulses.tangentImpulse1 = dtRatio * currentManifold.points[0].tangentImpulse;
		//printf ("ReadImpulse : %f \n" , currentImpulses.normalImpulse1 ) ;
		if (currentManifold.pointCount>1)
		{
			currentImpulses.normalImpulse2 = dtRatio * currentManifold.points[1].normalImpulse;
			currentImpulses.tangentImpulse2 = dtRatio * currentManifold.points[1].tangentImpulse;
		}
	}
	else
	{
		currentImpulses.normalImpulse1 = 0.0f;
		currentImpulses.tangentImpulse1 = 0.0f;
		currentImpulses.normalImpulse2 = 0.0f;
		currentImpulses.tangentImpulse2 = 0.0f;
	}

	impulses[contactIndex] = currentImpulses;
}

__kernel void ReadLastImpulsesFirstFrame(
                                 __global clb2Impulse* impulses, // output
								 __global uint* lastManifoldKeys, // input, sorted keys
								 __global uint* lastGlobalIndices, // input, sorted values
                                 //__global clb2Position* positions, // for debug
								 const unsigned int lastContactCount,
                                 const unsigned int contactCount,
						const __global int4* indices, //indices to bodies and fixtures
						const __global b2clFixtureStatic* fixtureStaticListBuffer,
						const __global int* coloredContactIndexToContactIndexMap,
						const __global b2clManifold* manifolds,
						const __global b2clManifold* lastManifolds,
						const int warmStarting,
						const float dtRatio)
{
    unsigned int contactIndex = get_global_id(0);

    if(contactIndex >= contactCount) return;

	clb2Impulse currentImpulses;

	if (!warmStarting)
	{
		currentImpulses.normalImpulse1 = 0.0f;
		currentImpulses.tangentImpulse1 = 0.0f;
		currentImpulses.normalImpulse2 = 0.0f;
		currentImpulses.tangentImpulse2 = 0.0f;

		impulses[contactIndex] = currentImpulses;

		return;
	}

    int globalIndex = coloredContactIndexToContactIndexMap[contactIndex];
	int4 currentIndices = indices[globalIndex];
	//// for debug
	//clb2Position thisPos = positions[currentIndices.z];

	// binary saerch currentKey in lastManifoldKeys
	int lb = 0; // low bound
	int ub = lastContactCount-1; // upper bound
	int mid;
	bool bFound = false;
	uint midKey;
	int currentKey;
	//if (currentIndices.x<currentIndices.y)
	//	currentKey = currentIndices.x<<16 | currentIndices.y; // suppose both indices are less than 65536 and thus fit in 16 bits.
	//else
	//	currentKey = currentIndices.y<<16 | currentIndices.x; // suppose both indices are less than 65536 and thus fit in 16 bits.

	//printf("currentIndex: (%d, %d)\n", currentIndices.x, currentIndices.y);
	int2 lastIndices;
	lastIndices.x = fixtureStaticListBuffer[currentIndices.x].m_last_uid;
	lastIndices.y = fixtureStaticListBuffer[currentIndices.y].m_last_uid;
	//printf("lastIndex: (%d, %d)\n", lastIndices.x, lastIndices.y);

	if (lastIndices.x>=0 && lastIndices.y>=0)
	{
		if (lastIndices.x<lastIndices.y)
			currentKey = lastIndices.x<<16 | lastIndices.y; // suppose both indices are less than 65536 and thus fit in 16 bits.
		else
			currentKey = lastIndices.y<<16 | lastIndices.x; // suppose both indices are less than 65536 and thus fit in 16 bits.

		//if (lastManifoldKeys[lb]<currentKey || lastManifoldKeys[ub]>currentKey)
		//	// currentKey not in lastManifoldKeys
		//	bFound = false;
		//else // do binary search
		{
			for (; lb<ub; )
			{
				mid = (lb+ub)/2;
				midKey = lastManifoldKeys[mid];
				if (midKey==currentKey)
				{
					bFound = true;
					break;
				}
				else if (midKey<currentKey)
				{
					ub = mid-1;
				}
				else
				{
					lb = mid+1;
				}
			}
			if (ub<lb)
				bFound = false;
		}
		if (lb==ub)
		{
			mid = lb;
			bFound = true;
		}
	}

	if (bFound)
	{
		//if (/*get_global_id(0)<10 && */thisPos.cx<100 && lastIndices.x==398 && lastIndices.y==397)
		//	printf("Found!!!\n");
		b2clManifold currentManifold = manifolds[globalIndex];
		for (int k = 0; k < currentManifold.pointCount; ++k)
		{
			b2clManifoldPoint* mp2 = currentManifold.points + k;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			int key2 = mp2->id.key;

			int lastGlobalIndex = lastGlobalIndices[mid];
			b2clManifold lastManifold = lastManifolds[lastGlobalIndex];
			//printf("last global index: %d\n", lastGlobalIndex);
			for (int j = 0; j < lastManifold.pointCount; ++j)
			{
				b2clManifoldPoint* mp1 = lastManifold.points + j;

				if (mp1->id.key == key2)
				{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					//if (/*get_global_id(0)<10 && */thisPos.cx<100 && lastIndices.x==398 && lastIndices.y==397)
					//	printf("key2:%d, normalImpuls: %f, tangentImpulse: %f\n", key2, mp1->normalImpulse, mp1->tangentImpulse);
					break;
				}
			}
		}

		currentImpulses.normalImpulse1 = dtRatio * currentManifold.points[0].normalImpulse;
		currentImpulses.tangentImpulse1 = dtRatio * currentManifold.points[0].tangentImpulse;
		if (currentManifold.pointCount>1)
		{
			currentImpulses.normalImpulse2 = dtRatio * currentManifold.points[1].normalImpulse;
			currentImpulses.tangentImpulse2 = dtRatio * currentManifold.points[1].tangentImpulse;
		}
	}
	else
	{
		//printf("Not Found!!!\n");
		currentImpulses.normalImpulse1 = 0.0f;
		currentImpulses.tangentImpulse1 = 0.0f;
		currentImpulses.normalImpulse2 = 0.0f;
		currentImpulses.tangentImpulse2 = 0.0f;
	}

	impulses[contactIndex] = currentImpulses;

	//// for debug
	//if (/*get_global_id(0)<10 && */thisPos.cx<100 && lastIndices.x==398 && lastIndices.y==397)
	//	printf("Impulse %d: (%f, %f)\n", contactIndex, currentImpulses.normalImpulse1, currentImpulses.tangentImpulse1);
	//	//printf("thisPos.cx: %f\n", thisPos.cx);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeDistanceJointVelocityConstraint
//
// Initialize values of Distance joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeDistanceJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength 
						//__global testData* testBuffer 
						)
{
    unsigned int contactIndex = get_global_id(0) + offset;
    if(contactIndex>=offset+colorLength) return;

	//printf ("Initialize Distance JOint \n");

	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];

	
 
	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;


	

	// Can this part move to the copyJoint function? 
	float2 m_localAnchorA = (float2) (thisJoint.b.distanceJointData.localAnchorA[0] , thisJoint.b.distanceJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.distanceJointData.localAnchorB[0] , thisJoint.b.distanceJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	thisJoint.b.distanceJointData.localCenterA[0] = m_localCenterA.x;
	thisJoint.b.distanceJointData.localCenterA[1] = m_localCenterA.y;
	thisJoint.b.distanceJointData.localCenterB[0] = m_localCenterB.x;
	thisJoint.b.distanceJointData.localCenterB[1] = m_localCenterB.y;
	thisJoint.b.distanceJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.distanceJointData.invMassB = bodyStaticB.m_invMass; 
	thisJoint.b.distanceJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.distanceJointData.invIB = bodyStaticB.m_invI;
 
	float2 m_rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);
	float2 m_rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);


    float2 m_u =  cB + m_rB - cA - m_rA ; 


	
	float length = b2clDot ( m_u, m_u ) ; length = sqrt (length); 


	if (length > b2_linearSlop)
	{
		m_u *= 1.0f/length ; 
	}
	else{
		m_u.x = m_u.y = 0 ; 
	}
 
	float crAu = b2clCross_VV(m_rA, m_u); 
	float crBu = b2clCross_VV(m_rB, m_u);

	float invMass = thisJoint.b.distanceJointData.invMassA + thisJoint.b.distanceJointData.invIA*crAu*crAu + thisJoint.b.distanceJointData.invMassB + thisJoint.b.distanceJointData.invIB*crBu*crBu;
 

	thisJoint.b.distanceJointData.mass = invMass != 0.0f ? 1.0f /invMass :0.0f ; 
	if (thisJoint.b.distanceJointData.frequencyHz > 0.0f) {
		float C = length - thisJoint.b.distanceJointData.nlength ; 

		float omega = 2.0f*b2_pi * thisJoint.b.distanceJointData.frequencyHz;
		float d = 2.0f * thisJoint.b.distanceJointData.mass * thisJoint.b.distanceJointData.dampingRatio * omega;
		float k = thisJoint.b.distanceJointData.mass * omega * omega ; 
		float h = dt ; 
		thisJoint.b.distanceJointData.gamma = h * (d + h * k);
		thisJoint.b.distanceJointData.gamma = thisJoint.b.distanceJointData.gamma != 0.0f ? 1.0f / thisJoint.b.distanceJointData.gamma :0.0f ; 
		thisJoint.b.distanceJointData.bias = C * h * k * thisJoint.b.distanceJointData.gamma; 
		invMass += thisJoint.b.distanceJointData.gamma ; 
		thisJoint.b.distanceJointData.mass = invMass != 0.0f ?1.0f/invMass :0.0f ;  
	}
	else {
		thisJoint.b.distanceJointData.gamma = 0.0f;
		thisJoint.b.distanceJointData.bias = 0.0f ; 
	}
	if (warmStarting){
		// Scale the impulse to support a variable time step.
		thisJoint.a.y.scalarImpulse *= dtRatio;

		float2 P = thisJoint.a.y.scalarImpulse * m_u;
		vA -= thisJoint.b.distanceJointData.invMassA * P;
		wA -= thisJoint.b.distanceJointData.invIA * b2clCross_VV(m_rA, P);
		vB += thisJoint.b.distanceJointData.invMassB * P;
		wB += thisJoint.b.distanceJointData.invIB * b2clCross_VV(m_rB, P);

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB;
	}
	else {
		thisJoint.a.y.scalarImpulse = 0.0f ; 
	}
	thisJoint.b.distanceJointData.rA[0] = m_rA.x ; thisJoint.b.distanceJointData.rA[1] = m_rA.y ; 
	thisJoint.b.distanceJointData.rB[0] = m_rB.x ; thisJoint.b.distanceJointData.rB[1] = m_rB.y ; 
    thisJoint.b.distanceJointData.u[0] = m_u.x ;   thisJoint.b.distanceJointData.u[1] = m_u.y ;  
	jointListBuffer[contactIndex] = thisJoint ; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeRevoluteJointVelocityConstraint
//
// Initialize values of Revolute joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeRevoluteJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
					    int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength						
						//__global testData* testBuffer 
						)
{
    unsigned int contactIndex = get_global_id(0) + offset;
    if(contactIndex>=offset+colorLength) return;


	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];

	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	// Can this part move to the copyJoint function? 
	float2 m_localAnchorA = (float2) (thisJoint.b.revoluteJointData.localAnchorA[0] , thisJoint.b.revoluteJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.revoluteJointData.localAnchorB[0] , thisJoint.b.revoluteJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	thisJoint.b.revoluteJointData.localCenterA[0] = m_localCenterA.x;
	thisJoint.b.revoluteJointData.localCenterA[1] = m_localCenterA.y;
	thisJoint.b.revoluteJointData.localCenterB[0] = m_localCenterB.x;
	thisJoint.b.revoluteJointData.localCenterB[1] = m_localCenterB.y;
	thisJoint.b.revoluteJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.revoluteJointData.invMassB = bodyStaticB.m_invMass; 
	thisJoint.b.revoluteJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.revoluteJointData.invIB = bodyStaticB.m_invI;
 
	float2 m_rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);
	float2 m_rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);
	
	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	float mA = thisJoint.b.revoluteJointData.invMassA, mB = thisJoint.b.revoluteJointData.invMassB;
	float iA = thisJoint.b.revoluteJointData.invIA, iB = thisJoint.b.revoluteJointData.invIB;

	bool fixedRotation = (iA + iB == 0.0f);

	thisJoint.b.revoluteJointData.mass.ex[0] = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
	thisJoint.b.revoluteJointData.mass.ey[0] = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
	thisJoint.b.revoluteJointData.mass.ez[0] = -m_rA.y * iA - m_rB.y * iB;
	thisJoint.b.revoluteJointData.mass.ex[1] = thisJoint.b.revoluteJointData.mass.ey[0];
	thisJoint.b.revoluteJointData.mass.ey[1] = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
	thisJoint.b.revoluteJointData.mass.ez[1] = m_rA.x * iA + m_rB.x * iB;
	thisJoint.b.revoluteJointData.mass.ex[2] = thisJoint.b.revoluteJointData.mass.ez[0];
	thisJoint.b.revoluteJointData.mass.ey[2] = thisJoint.b.revoluteJointData.mass.ez[1];
	thisJoint.b.revoluteJointData.mass.ez[2] = iA + iB;

	thisJoint.b.revoluteJointData.motorMass = iA + iB;
	if (thisJoint.b.revoluteJointData.motorMass > 0.0f)
	{
		thisJoint.b.revoluteJointData.motorMass = 1.0f / thisJoint.b.revoluteJointData.motorMass;
	}

	if (thisJoint.b.revoluteJointData.enableMotor == 0 || fixedRotation)
	{
		thisJoint.motorImpulse = 0.0f;
	}

	if (thisJoint.b.revoluteJointData.enableLimit && fixedRotation == false)
	{
		float jointAngle = aB - aA - thisJoint.b.revoluteJointData.referenceAngle;
		if (b2clAbs(thisJoint.b.revoluteJointData.upperAngle - thisJoint.b.revoluteJointData.lowerAngle) < 2.0f * b2_angularSlop)
		{
			thisJoint.b.revoluteJointData.limitState = e_equalLimits;
		}
		else if (jointAngle <= thisJoint.b.revoluteJointData.lowerAngle)
		{
			if (thisJoint.b.revoluteJointData.limitState != e_atLowerLimit)
			{
				thisJoint.a.x.impulse[2] = 0.0f;
			}
			thisJoint.b.revoluteJointData.limitState = e_atLowerLimit;
		}
		else if (jointAngle >= thisJoint.b.revoluteJointData.upperAngle)
		{
			if (thisJoint.b.revoluteJointData.limitState != e_atUpperLimit)
			{
				thisJoint.a.x.impulse[2] = 0.0f;
			}
			thisJoint.b.revoluteJointData.limitState = e_atUpperLimit;
		}
		else
		{
			thisJoint.b.revoluteJointData.limitState = e_inactiveLimit;
			thisJoint.a.x.impulse[2] = 0.0f;
		}
	}
	else
	{
		thisJoint.b.revoluteJointData.limitState = e_inactiveLimit;
	}

	if (warmStarting)
	{
		// Scale impulses to support a variable time step.
		thisJoint.a.x.impulse[0] *= dtRatio;
		thisJoint.a.x.impulse[1] *= dtRatio;
		thisJoint.a.x.impulse[2] *= dtRatio;
		thisJoint.motorImpulse *= dtRatio;

		float2 P = (float2)(thisJoint.a.x.impulse[0], thisJoint.a.x.impulse[1]);

		vA -= mA * P;
		wA -= iA * (b2clCross_VV(m_rA, P) + thisJoint.motorImpulse + thisJoint.a.x.impulse[2]);

		vB += mB * P;
		wB += iB * (b2clCross_VV(m_rB, P) + thisJoint.motorImpulse + thisJoint.a.x.impulse[2]);

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB;
	}
	else 
	{
		thisJoint.a.x.impulse[0] = 0.0f;
		thisJoint.a.x.impulse[1] = 0.0f;
		thisJoint.a.x.impulse[2] = 0.0f;
		thisJoint.motorImpulse = 0.0f;
	}
	thisJoint.b.revoluteJointData.rA[0] = m_rA.x ; thisJoint.b.revoluteJointData.rA[1] = m_rA.y ; 
	thisJoint.b.revoluteJointData.rB[0] = m_rB.x ; thisJoint.b.revoluteJointData.rB[1] = m_rB.y ; 
	jointListBuffer[contactIndex] = thisJoint ; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializePrismaticJointVelocityConstraint
//
// Initialize values of Prismatic joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializePrismaticJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength
						//__global testData* testBuffer 
						)
{


    unsigned int contactIndex = get_global_id(0) + offset;
    if(contactIndex>=offset+colorLength) return;

	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];
	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];

	float2 m_localAnchorA = (float2) (thisJoint.b.prismaticJointData.localAnchorA[0] , thisJoint.b.prismaticJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.prismaticJointData.localAnchorB[0] , thisJoint.b.prismaticJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	float2 m_localXAxisA = (float2) (thisJoint.b.prismaticJointData.localXAxisA[0] , thisJoint.b.prismaticJointData.localXAxisA[1]);
	float2 m_localYAxisA = (float2) (thisJoint.b.prismaticJointData.localYAxisA[0] , thisJoint.b.prismaticJointData.localYAxisA[1]);
	thisJoint.b.prismaticJointData.localCenterA[0] = m_localCenterA.x;
	thisJoint.b.prismaticJointData.localCenterA[1] = m_localCenterA.y;
	thisJoint.b.prismaticJointData.localCenterB[0] = m_localCenterB.x;
	thisJoint.b.prismaticJointData.localCenterB[1] = m_localCenterB.y;
	thisJoint.b.prismaticJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.prismaticJointData.invMassB = bodyStaticB.m_invMass; 
	thisJoint.b.prismaticJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.prismaticJointData.invIB = bodyStaticB.m_invI;

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	float2 rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);
	float2 rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);
	float2 d =  cB - cA + rB - rA ; 

	float mA = thisJoint.b.prismaticJointData.invMassA, mB = thisJoint.b.prismaticJointData.invMassB;
	float iA = thisJoint.b.prismaticJointData.invIA, iB = thisJoint.b.prismaticJointData.invIB;

	// Compute motor Jacobian and effective mass.
	float2 m_axis = b2clMul_Rotate(xfA.q, m_localXAxisA);
	thisJoint.b.prismaticJointData.a1 = b2clCross_VV(d + rA, m_axis);
	thisJoint.b.prismaticJointData.a2 = b2clCross_VV(rB, m_axis);

	thisJoint.b.prismaticJointData.motorMass = mA + mB + iA * thisJoint.b.prismaticJointData.a1 * thisJoint.b.prismaticJointData.a1 + iB * thisJoint.b.prismaticJointData.a2 * thisJoint.b.prismaticJointData.a2;
	if (thisJoint.b.prismaticJointData.motorMass > 0.0f)
	{
		thisJoint.b.prismaticJointData.motorMass = 1.0f / thisJoint.b.prismaticJointData.motorMass;
	}

	// Prismatic constraint.
	float2 m_perp = b2clMul_Rotate(xfA.q, m_localYAxisA);

	thisJoint.b.prismaticJointData.s1 = b2clCross_VV(d + rA, m_perp);
	thisJoint.b.prismaticJointData.s2 = b2clCross_VV(rB, m_perp);

	float k11 = mA + mB + iA * thisJoint.b.prismaticJointData.s1 * thisJoint.b.prismaticJointData.s1 + iB * thisJoint.b.prismaticJointData.s2 * thisJoint.b.prismaticJointData.s2;
	float k12 = iA * thisJoint.b.prismaticJointData.s1 + iB * thisJoint.b.prismaticJointData.s2;
	float k13 = iA * thisJoint.b.prismaticJointData.s1 * thisJoint.b.prismaticJointData.a1 + iB * thisJoint.b.prismaticJointData.s2 * thisJoint.b.prismaticJointData.a2;
	float k22 = iA + iB;
	if (k22 == 0.0f)
	{
		// For bodies with fixed rotation.
		k22 = 1.0f;
	}
	float k23 = iA * thisJoint.b.prismaticJointData.a1 + iB * thisJoint.b.prismaticJointData.a2;
	float k33 = mA + mB + iA * thisJoint.b.prismaticJointData.a1 * thisJoint.b.prismaticJointData.a1 + iB * thisJoint.b.prismaticJointData.a2 * thisJoint.b.prismaticJointData.a2;

	thisJoint.b.prismaticJointData.K.ex[0] = k11;
	thisJoint.b.prismaticJointData.K.ex[1] = k12;
	thisJoint.b.prismaticJointData.K.ex[2] = k13;
	thisJoint.b.prismaticJointData.K.ey[0] = k12;
	thisJoint.b.prismaticJointData.K.ey[1] = k22;
	thisJoint.b.prismaticJointData.K.ey[2] = k23;
	thisJoint.b.prismaticJointData.K.ez[0] = k13;
	thisJoint.b.prismaticJointData.K.ez[1] = k23;
	thisJoint.b.prismaticJointData.K.ez[2] = k33;

	// Compute motor and limit terms.
	if (thisJoint.b.prismaticJointData.enableLimit)
	{
		float jointTranslation = b2clDot(m_axis, d);
		if (b2clAbs(thisJoint.b.prismaticJointData.upperTranslation - thisJoint.b.prismaticJointData.lowerTranslation) < 2.0f * b2_linearSlop)
		{
			thisJoint.b.prismaticJointData.limitState = e_equalLimits;
		}
		else if (jointTranslation <= thisJoint.b.prismaticJointData.lowerTranslation)
		{
			if (thisJoint.b.prismaticJointData.limitState != e_atLowerLimit)
			{
				thisJoint.b.prismaticJointData.limitState = e_atLowerLimit;
				thisJoint.a.x.impulse[2] = 0.0f;
			}
		}
		else if (jointTranslation >= thisJoint.b.prismaticJointData.upperTranslation)
		{
			if (thisJoint.b.prismaticJointData.limitState != e_atUpperLimit)
			{
				thisJoint.b.prismaticJointData.limitState = e_atUpperLimit;
				thisJoint.a.x.impulse[2] = 0.0f;
			}
		}
		else
		{
			thisJoint.b.prismaticJointData.limitState = e_inactiveLimit;
			thisJoint.a.x.impulse[2] = 0.0f;
		}
	}
	else
	{
		thisJoint.b.prismaticJointData.limitState = e_inactiveLimit;
		thisJoint.a.x.impulse[2] = 0.0f;
	}

	if (thisJoint.b.prismaticJointData.enableMotor == false)
	{
		thisJoint.motorImpulse = 0.0f;
	}

	if (warmStarting)
	{
		// Account for variable time step.
		thisJoint.a.x.impulse[0] *= dtRatio;
		thisJoint.a.x.impulse[1] *= dtRatio;
		thisJoint.a.x.impulse[2] *= dtRatio;
		thisJoint.motorImpulse *= dtRatio;

		float2 P = thisJoint.a.x.impulse[0] * m_perp + (thisJoint.motorImpulse + thisJoint.a.x.impulse[2]) * m_axis;
		float LA = thisJoint.a.x.impulse[0] * thisJoint.b.prismaticJointData.s1 + thisJoint.a.x.impulse[1] + (thisJoint.motorImpulse + thisJoint.a.x.impulse[2]) * thisJoint.b.prismaticJointData.a1;
		float LB = thisJoint.a.x.impulse[0] * thisJoint.b.prismaticJointData.s2 + thisJoint.a.x.impulse[1] + (thisJoint.motorImpulse + thisJoint.a.x.impulse[2]) * thisJoint.b.prismaticJointData.a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB;
	}
	else
	{
		thisJoint.a.x.impulse[0] = 0.0f;
		thisJoint.a.x.impulse[1] = 0.0f;
		thisJoint.a.x.impulse[2] = 0.0f;
		thisJoint.motorImpulse = 0.0f;
	}

	thisJoint.b.prismaticJointData.axis[0] = m_axis.x;
	thisJoint.b.prismaticJointData.axis[1] = m_axis.y;
	thisJoint.b.prismaticJointData.perp[0] = m_perp.x;
	thisJoint.b.prismaticJointData.perp[1] = m_perp.y;
	jointListBuffer[contactIndex] = thisJoint;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeGearJointVelocityConstraint
//
// Initialize values of Gear joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeGearJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						//__global testData* testBuffer 
						const unsigned int offset,
						const unsigned int colorLength
						)
{
    unsigned int contactIndex = get_global_id(0) + offset;
    if(contactIndex>=offset+colorLength) return;

	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];
	b2clBodyStatic bodyStaticC = bodyStaticListBuffer[thisJoint.indexC];
	b2clBodyStatic bodyStaticD = bodyStaticListBuffer[thisJoint.indexD];
	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];
	b2clTransform xfC = xfGlobal[thisJoint.indexC];
	b2clTransform xfD = xfGlobal[thisJoint.indexD];

	float2 m_localAnchorA = (float2) (thisJoint.b.gearJointData.localAnchorA[0] , thisJoint.b.gearJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.gearJointData.localAnchorB[0] , thisJoint.b.gearJointData.localAnchorB[1]);
	float2 m_localAnchorC = (float2) (thisJoint.b.gearJointData.localAnchorC[0] , thisJoint.b.gearJointData.localAnchorC[1]);
	float2 m_localAnchorD = (float2) (thisJoint.b.gearJointData.localAnchorD[0] , thisJoint.b.gearJointData.localAnchorD[1]);

	float2 m_lcA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_lcB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	float2 m_lcC = (float2) (bodyStaticC.m_localCenter.x, bodyStaticC.m_localCenter.y);
	float2 m_lcD = (float2) (bodyStaticD.m_localCenter.x, bodyStaticD.m_localCenter.y);

	float2 localAxisC = (float2) (thisJoint.b.gearJointData.localAxisC[0] , thisJoint.b.gearJointData.localAxisC[1]);
	float2 localAxisD = (float2) (thisJoint.b.gearJointData.localAxisD[0] , thisJoint.b.gearJointData.localAxisD[1]);

	thisJoint.b.gearJointData.lcA[0] = m_lcA.x;
	thisJoint.b.gearJointData.lcA[1] = m_lcA.y;
	thisJoint.b.gearJointData.lcB[0] = m_lcB.x;
	thisJoint.b.gearJointData.lcB[1] = m_lcB.y;
	thisJoint.b.gearJointData.lcC[0] = m_lcC.x;
	thisJoint.b.gearJointData.lcC[1] = m_lcC.y;
	thisJoint.b.gearJointData.lcD[0] = m_lcD.x;
	thisJoint.b.gearJointData.lcD[1] = m_lcD.y;
	thisJoint.b.gearJointData.mA = bodyStaticA.m_invMass;
	thisJoint.b.gearJointData.mB = bodyStaticB.m_invMass; 
	thisJoint.b.gearJointData.mC = bodyStaticC.m_invMass;
	thisJoint.b.gearJointData.mD = bodyStaticD.m_invMass; 
	thisJoint.b.gearJointData.iA = bodyStaticA.m_invI;
	thisJoint.b.gearJointData.iB = bodyStaticB.m_invI;
	thisJoint.b.gearJointData.iC = bodyStaticC.m_invI;
	thisJoint.b.gearJointData.iD = bodyStaticD.m_invI;

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	clb2Position positionC = positions[thisJoint.indexC]; clb2Velocity velocityC = velocities[thisJoint.indexC];  
	clb2Position positionD = positions[thisJoint.indexD]; clb2Velocity velocityD = velocities[thisJoint.indexD]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;
	float2 cC =  (float2)(positionC.cx, positionC.cy);   float aC = positionC.a;
	float2 vC =  (float2)(velocityC.vx, velocityC.vy);   float wC = velocityC.w;
	float2 cD =  (float2)(positionD.cx, positionD.cy);  float aD = positionD.a;
    float2 vD =  (float2)(velocityD.vx, velocityD.vy);  float wD = velocityD.w;

	thisJoint.b.gearJointData.mass = 0.0f;

	float2 m_JvAC, m_JvBD;
	if (thisJoint.b.gearJointData.typeA == e_revoluteJoint)
	{
		m_JvAC = (float2)(0.0f, 0.0f);
		thisJoint.b.gearJointData.JwA = 1.0f;
		thisJoint.b.gearJointData.JwC = 1.0f;
		thisJoint.b.gearJointData.mass += thisJoint.b.gearJointData.iA + thisJoint.b.gearJointData.iC;
	}
	else
	{
		float2 u = b2clMul_Rotate(xfC.q, localAxisC);
		float2 rC = b2clMul_Rotate(xfC.q, m_localAnchorC - m_lcC);
		float2 rA = b2clMul_Rotate(xfA.q, m_localAnchorA - m_lcA);
		m_JvAC = u;
		thisJoint.b.gearJointData.JwC = b2clCross_VV(rC, u);
		thisJoint.b.gearJointData.JwA = b2clCross_VV(rA, u);
		thisJoint.b.gearJointData.mass += thisJoint.b.gearJointData.mC + thisJoint.b.gearJointData.mA + thisJoint.b.gearJointData.iC * thisJoint.b.gearJointData.JwC * thisJoint.b.gearJointData.JwC + thisJoint.b.gearJointData.iA * thisJoint.b.gearJointData.JwA * thisJoint.b.gearJointData.JwA;
	}

	if (thisJoint.b.gearJointData.typeB == e_revoluteJoint)
	{
		m_JvBD = (float2)(0.0f, 0.0f);
		thisJoint.b.gearJointData.JwB = thisJoint.b.gearJointData.ratio;
		thisJoint.b.gearJointData.JwD = thisJoint.b.gearJointData.ratio;
		thisJoint.b.gearJointData.mass += thisJoint.b.gearJointData.ratio * thisJoint.b.gearJointData.ratio * (thisJoint.b.gearJointData.iB + thisJoint.b.gearJointData.iD);
	}
	else
	{
		float2 u = b2clMul_Rotate(xfD.q, localAxisD);
		float2 rD = b2clMul_Rotate(xfD.q, m_localAnchorD - m_lcD);
		float2 rB = b2clMul_Rotate(xfB.q, m_localAnchorB - m_lcB);
		m_JvBD = thisJoint.b.gearJointData.ratio * u;
		thisJoint.b.gearJointData.JwD = thisJoint.b.gearJointData.ratio * b2clCross_VV(rD, u);
		thisJoint.b.gearJointData.JwB = thisJoint.b.gearJointData.ratio * b2clCross_VV(rB, u);
		thisJoint.b.gearJointData.mass += thisJoint.b.gearJointData.ratio * thisJoint.b.gearJointData.ratio * (thisJoint.b.gearJointData.mD + thisJoint.b.gearJointData.mB) + thisJoint.b.gearJointData.iD * thisJoint.b.gearJointData.JwD * thisJoint.b.gearJointData.JwD + thisJoint.b.gearJointData.iB * thisJoint.b.gearJointData.JwB * thisJoint.b.gearJointData.JwB;
	}

	// Compute effective mass.
	thisJoint.b.gearJointData.mass = thisJoint.b.gearJointData.mass > 0.0f ? 1.0f / thisJoint.b.gearJointData.mass : 0.0f;

	if (warmStarting)
	{
		vA += (thisJoint.b.gearJointData.mA * thisJoint.a.y.scalarImpulse) * m_JvAC;
		wA += thisJoint.b.gearJointData.iA * thisJoint.a.y.scalarImpulse * thisJoint.b.gearJointData.JwA;
		vB += (thisJoint.b.gearJointData.mB * thisJoint.a.y.scalarImpulse) * m_JvBD;
		wB += thisJoint.b.gearJointData.iB * thisJoint.a.y.scalarImpulse * thisJoint.b.gearJointData.JwB;
		vC -= (thisJoint.b.gearJointData.mC * thisJoint.a.y.scalarImpulse) * m_JvAC;
		wC -= thisJoint.b.gearJointData.iC * thisJoint.a.y.scalarImpulse * thisJoint.b.gearJointData.JwC;
		vD -= (thisJoint.b.gearJointData.mD * thisJoint.a.y.scalarImpulse) * m_JvBD;
		wD -= thisJoint.b.gearJointData.iD * thisJoint.a.y.scalarImpulse * thisJoint.b.gearJointData.JwD;

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
		velocityC.vx = vC.x ; velocityC.vy = vC.y ; velocityC.w = wC ; velocities[thisJoint.indexC] = velocityC; 
		velocityD.vx = vD.x ; velocityD.vy = vD.y ; velocityD.w = wD ; velocities[thisJoint.indexD] = velocityD; 
	}
	else
	{
		thisJoint.a.y.scalarImpulse = 0.0f;
	}

	thisJoint.b.gearJointData.JvAC[0] = m_JvAC.x;
	thisJoint.b.gearJointData.JvAC[1] = m_JvAC.y;
	thisJoint.b.gearJointData.JvBD[0] = m_JvBD.x;
	thisJoint.b.gearJointData.JvBD[1] = m_JvBD.y;
	jointListBuffer[contactIndex] = thisJoint;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializePulleyJointVelocityConstraint
//
// Initialize values of Pulley joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializePulleyJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength
						//__global testData* testBuffer 
						)
{
	 unsigned int contactIndex = get_global_id(0) + offset;
    if(contactIndex>=offset+colorLength) return;

	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];
	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];
	float2 m_localAnchorA = (float2) (thisJoint.b.pulleyJointData.localAnchorA[0] , thisJoint.b.pulleyJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.pulleyJointData.localAnchorB[0] , thisJoint.b.pulleyJointData.localAnchorB[1]);
	float2 m_groundAnchorA = (float2) (thisJoint.b.pulleyJointData.groundAnchorA[0], thisJoint.b.pulleyJointData.groundAnchorA[1]);
	float2 m_groundAnchorB = (float2) (thisJoint.b.pulleyJointData.groundAnchorB[0], thisJoint.b.pulleyJointData.groundAnchorB[1]); 
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	thisJoint.b.pulleyJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.pulleyJointData.invMassB = bodyStaticB.m_invMass;  
	thisJoint.b.pulleyJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.pulleyJointData.invIB = bodyStaticB.m_invI;
	thisJoint.b.pulleyJointData.localCenterA[0] = m_localCenterA.x ; thisJoint.b.pulleyJointData.localCenterA[1] = m_localCenterA.y ;
	thisJoint.b.pulleyJointData.localCenterB[0] = m_localCenterB.x ; thisJoint.b.pulleyJointData.localCenterB[1] = m_localCenterB.y ;

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	float2 rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);float2 rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);
	float2 uA = cA + rA - m_groundAnchorA ; float2 uB = cB + rB - m_groundAnchorB ; 

	float lengthA = uA.x * uA.x + uA.y * uA.y; lengthA = sqrt (lengthA); 
	float lengthB = uB.x * uB.x + uB.y * uB.y; lengthB = sqrt (lengthB);
	if (lengthA > 10.0f * b2_linearSlop) {
		uA *= 1.0f/lengthA ; 
	}
	else {
		uA.x = uA.y = 0 ; 
	}
	if (lengthB > 10.0f * b2_linearSlop) {
		uB *= 1.0f/lengthB ; 
	}
	else{
		uB.x = uB.y = 0 ; 
	}

	float ruA = b2clCross_VV (rA, uA);
	float ruB = b2clCross_VV (rB, uB);
	float mA = thisJoint.b.pulleyJointData.invMassA + thisJoint.b.pulleyJointData.invIA * ruA * ruA ; 
	float mB = thisJoint.b.pulleyJointData.invMassB + thisJoint.b.pulleyJointData.invIB * ruB * ruB ; 
	thisJoint.b.pulleyJointData.mass = mA + thisJoint.b.pulleyJointData.ratio* thisJoint.b.pulleyJointData.ratio*mB ; 
	if (thisJoint.b.pulleyJointData.mass > 0.0f) {
		thisJoint.b.pulleyJointData.mass = 1.0f/thisJoint.b.pulleyJointData.mass; 
	}
	if (warmStarting){
		// Scale impulses to support variable time steps.
		thisJoint.a.y.scalarImpulse *= dtRatio;

		// Warm starting.
		float2 PA = -(thisJoint.a.y.scalarImpulse) * uA;
		float2 PB = (-thisJoint.b.pulleyJointData.ratio * thisJoint.a.y.scalarImpulse) * uB;

		vA += thisJoint.b.pulleyJointData.invMassA * PA;
		wA += thisJoint.b.pulleyJointData.invIA * b2clCross_VV(rA, PA);
		vB += thisJoint.b.pulleyJointData.invMassB * PB;
		wB += thisJoint.b.pulleyJointData.invIB * b2clCross_VV(rB, PB);

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	}
	else {
		thisJoint.a.y.scalarImpulse = 0.0f ; 
	}
	thisJoint.b.pulleyJointData.rA[0] = rA.x ; thisJoint.b.pulleyJointData.rA[1] = rA.y ; 	thisJoint.b.pulleyJointData.rB[0] = rB.x ; thisJoint.b.pulleyJointData.rB[1] = rB.y ; 
	thisJoint.b.pulleyJointData.uA[0] = uA.x ; thisJoint.b.pulleyJointData.uA[1] = uA.y ; thisJoint.b.pulleyJointData.uB[0] = uB.x ; thisJoint.b.pulleyJointData.uB[1] = uB.y ; 
	jointListBuffer[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeRopeJointVelocityConstraint
//
// Initialize values of Rope joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeRopeJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength
						//__global testData* testBuffer 
						)
{
    unsigned int contactIndex = get_global_id(0) + offset;
    if(contactIndex>=offset+colorLength) return;

	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];
	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];
	float2 m_localAnchorA = (float2) (thisJoint.b.ropeJointData.localAnchorA[0] , thisJoint.b.ropeJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.ropeJointData.localAnchorB[0] , thisJoint.b.ropeJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	thisJoint.b.ropeJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.ropeJointData.invMassB = bodyStaticB.m_invMass;  
	thisJoint.b.ropeJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.ropeJointData.invIB = bodyStaticB.m_invI;
	thisJoint.b.ropeJointData.localCenterA[0] = m_localCenterA.x ; thisJoint.b.ropeJointData.localCenterA[1] = m_localCenterA.y ;
	thisJoint.b.ropeJointData.localCenterB[0] = m_localCenterB.x ; thisJoint.b.ropeJointData.localCenterB[1] = m_localCenterB.y ;
	
	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	float2 rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);
	float2 rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);
	thisJoint.b.ropeJointData.rA[0] = rA.x ; thisJoint.b.ropeJointData.rA[1] = rA.y ; 	thisJoint.b.ropeJointData.rB[0] = rB.x ; thisJoint.b.ropeJointData.rB[1] = rB.y ; 
	float2 u = cB + rB - cA -  rA ; 
	 
	float length = sqrt (b2clDot (u,u));
	thisJoint.b.ropeJointData.nlength = length ; 
	float C = length - thisJoint.b.ropeJointData.maxLength ;
    
	if (C > 0.0f)
	{
		thisJoint.b.ropeJointData.limitState = e_atUpperLimit;
	}
	else
	{
		thisJoint.b.ropeJointData.limitState = e_inactiveLimit;
	}
	if ( thisJoint.b.ropeJointData.nlength > b2_linearSlop)
	{
		u *= 1.0f / thisJoint.b.ropeJointData.nlength;
	}
	else
	{
		u.x = u.y = 0;
		thisJoint.b.ropeJointData.u[0] = thisJoint.b.ropeJointData.u[1] = 0 ; 
		thisJoint.b.ropeJointData.mass = 0.0f;
		thisJoint.a.y.scalarImpulse = 0.0f;
		jointListBuffer[contactIndex] = thisJoint; 
		return;
	}
	thisJoint.b.ropeJointData.u[0] = u.x ; thisJoint.b.ropeJointData.u[1] = u.y ; 	
	float crA = b2clCross_VV(rA, u);
	float crB = b2clCross_VV(rB, u);
	float invMass = thisJoint.b.ropeJointData.invMassA + thisJoint.b.ropeJointData.invIA * crA * crA + thisJoint.b.ropeJointData.invMassB + thisJoint.b.ropeJointData.invIB * crB * crB;

	thisJoint.b.ropeJointData.mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

	if (warmStarting)
	{
		// Scale the impulse to support a variable time step.
		thisJoint.a.y.scalarImpulse *= dtRatio;
		float2 P = u * thisJoint.a.y.scalarImpulse; 
		vA -=  P * thisJoint.b.ropeJointData.invMassA;
		wA -= b2clCross_VV(rA, P) * thisJoint.b.ropeJointData.invIA;
		vB += P * thisJoint.b.ropeJointData.invMassB;
		wB += b2clCross_VV(rB, P) * thisJoint.b.ropeJointData.invIB;

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB;
	}
	else
	{
		thisJoint.a.y.scalarImpulse = 0.0f;
	}
	jointListBuffer[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeWheelJointVelocityConstraint
//
// Initialize values of Wheel joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeWheelJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength
						//__global testData* testBuffer 
						)
{
	 unsigned int contactIndex = get_global_id(0) + offset;
    if(contactIndex>=offset+colorLength) return;

	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];

	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	float2 m_localAnchorA = (float2) (thisJoint.b.wheelJointData.localAnchorA[0] , thisJoint.b.wheelJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.wheelJointData.localAnchorB[0] , thisJoint.b.wheelJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	thisJoint.b.wheelJointData.localCenterA[0] = m_localCenterA.x;
	thisJoint.b.wheelJointData.localCenterA[1] = m_localCenterA.y;
	thisJoint.b.wheelJointData.localCenterB[0] = m_localCenterB.x;
	thisJoint.b.wheelJointData.localCenterB[1] = m_localCenterB.y;
	thisJoint.b.wheelJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.wheelJointData.invMassB = bodyStaticB.m_invMass; 
	thisJoint.b.wheelJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.wheelJointData.invIB = bodyStaticB.m_invI;
 
	float mA = thisJoint.b.wheelJointData.invMassA, mB = thisJoint.b.wheelJointData.invMassB;
	float iA = thisJoint.b.wheelJointData.invIA, iB = thisJoint.b.wheelJointData.invIB;

	// Compute the effective masses.
	float2 rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);
	float2 rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);
	float2 d = cB + rB - cA - rA;

	float2 m_ax, m_ay;
	float2 m_localXAxisA = (float2)(thisJoint.b.wheelJointData.localXAxisA[0], thisJoint.b.wheelJointData.localXAxisA[1]);
	float2 m_localYAxisA = (float2)(thisJoint.b.wheelJointData.localYAxisA[0], thisJoint.b.wheelJointData.localYAxisA[1]);
	// Point to line constraint
	{
		m_ay = b2clMul_Rotate(xfA.q, m_localYAxisA);
		thisJoint.b.wheelJointData.sAy = b2clCross_VV(d + rA, m_ay);
		thisJoint.b.wheelJointData.sBy = b2clCross_VV(rB, m_ay);

		thisJoint.b.wheelJointData.mass = mA + mB + iA * thisJoint.b.wheelJointData.sAy * thisJoint.b.wheelJointData.sAy + iB * thisJoint.b.wheelJointData.sBy * thisJoint.b.wheelJointData.sBy;

		if (thisJoint.b.wheelJointData.mass > 0.0f)
		{
			thisJoint.b.wheelJointData.mass = 1.0f / thisJoint.b.wheelJointData.mass;
		}
	}

	// Spring constraint
	thisJoint.b.wheelJointData.springMass = 0.0f;
	thisJoint.b.wheelJointData.bias = 0.0f;
	thisJoint.b.wheelJointData.gamma = 0.0f;
	if (thisJoint.b.wheelJointData.frequencyHz > 0.0f)
	{
		m_ax = b2clMul_Rotate(xfA.q, m_localXAxisA);
		thisJoint.b.wheelJointData.sAx = b2clCross_VV(d + rA, m_ax);
		thisJoint.b.wheelJointData.sBx = b2clCross_VV(rB, m_ax);

		float invMass = mA + mB + iA * thisJoint.b.wheelJointData.sAx * thisJoint.b.wheelJointData.sAx + iB * thisJoint.b.wheelJointData.sBx * thisJoint.b.wheelJointData.sBx;

		if (invMass > 0.0f)
		{
			thisJoint.b.wheelJointData.springMass = 1.0f / invMass;

			float C = b2clDot(d, m_ax);

			// Frequency
			float omega = 2.0f * b2_pi * thisJoint.b.wheelJointData.frequencyHz;

			// Damping coefficient
			float d = 2.0f * thisJoint.b.wheelJointData.springMass * thisJoint.b.wheelJointData.dampingRatio * omega;

			// Spring stiffness
			float k = thisJoint.b.wheelJointData.springMass * omega * omega;

			// magic formulas
			float h = dt;
			thisJoint.b.wheelJointData.gamma = h * (d + h * k);
			if (thisJoint.b.wheelJointData.gamma > 0.0f)
			{
				thisJoint.b.wheelJointData.gamma = 1.0f / thisJoint.b.wheelJointData.gamma;
			}

			thisJoint.b.wheelJointData.bias = C * h * k * thisJoint.b.wheelJointData.gamma;

			thisJoint.b.wheelJointData.springMass = invMass + thisJoint.b.wheelJointData.gamma;
			if (thisJoint.b.wheelJointData.springMass > 0.0f)
			{
				thisJoint.b.wheelJointData.springMass = 1.0f / thisJoint.b.wheelJointData.springMass;
			}
		}
	}
	else
	{
		thisJoint.a.y.springImpulse = 0.0f;
	}

	// Rotational motor
	if (thisJoint.b.wheelJointData.enableMotor)
	{
		thisJoint.b.wheelJointData.motorMass = iA + iB;
		if (thisJoint.b.wheelJointData.motorMass > 0.0f)
		{
			thisJoint.b.wheelJointData.motorMass = 1.0f / thisJoint.b.wheelJointData.motorMass;
		}
	}
	else
	{
		thisJoint.b.wheelJointData.motorMass = 0.0f;
		thisJoint.motorImpulse = 0.0f;
	}

	if (warmStarting)
	{
		// Account for variable time step.
		thisJoint.a.y.scalarImpulse *= dtRatio;
		thisJoint.a.y.springImpulse *= dtRatio;
		thisJoint.motorImpulse *= dtRatio;

		float2 P = thisJoint.a.y.scalarImpulse * m_ay + thisJoint.a.y.springImpulse * m_ax;
		float LA = thisJoint.a.y.scalarImpulse * thisJoint.b.wheelJointData.sAy + thisJoint.a.y.springImpulse * thisJoint.b.wheelJointData.sAx + thisJoint.motorImpulse;
		float LB = thisJoint.a.y.scalarImpulse * thisJoint.b.wheelJointData.sBy + thisJoint.a.y.springImpulse * thisJoint.b.wheelJointData.sBx + thisJoint.motorImpulse;

		vA -= thisJoint.b.wheelJointData.invMassA * P;
		wA -= thisJoint.b.wheelJointData.invIA * LA;

		vB += thisJoint.b.wheelJointData.invMassB * P;
		wB += thisJoint.b.wheelJointData.invIB * LB;

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB;
	}
	else
	{
		thisJoint.a.y.scalarImpulse = 0.0f;
		thisJoint.a.y.springImpulse = 0.0f;
		thisJoint.motorImpulse = 0.0f;
	}

	thisJoint.b.wheelJointData.ax[0] = m_ax.x;
	thisJoint.b.wheelJointData.ax[1] = m_ax.y;
	thisJoint.b.wheelJointData.ay[0] = m_ay.x;
	thisJoint.b.wheelJointData.ay[1] = m_ay.y;

	jointListBuffer[contactIndex] = thisJoint;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeWeldJointVelocityConstraint
//
// Initialize values of Weld joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeWeldJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
                        const unsigned int colorLength
						//__global testData* testBuffer 
						)
{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex >= offset + colorLength) 
		return;
	
	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];

	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	float2 m_localAnchorA = (float2) (thisJoint.b.weldJointData.localAnchorA[0] , thisJoint.b.weldJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.weldJointData.localAnchorB[0] , thisJoint.b.weldJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	thisJoint.b.weldJointData.localCenterA[0] = m_localCenterA.x;
	thisJoint.b.weldJointData.localCenterA[1] = m_localCenterA.y;
	thisJoint.b.weldJointData.localCenterB[0] = m_localCenterB.x;
	thisJoint.b.weldJointData.localCenterB[1] = m_localCenterB.y;
	thisJoint.b.weldJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.weldJointData.invMassB = bodyStaticB.m_invMass; 
	thisJoint.b.weldJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.weldJointData.invIB = bodyStaticB.m_invI;

	float2 m_rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);
	float2 m_rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	float mA = thisJoint.b.weldJointData.invMassA, mB = thisJoint.b.weldJointData.invMassB;
	float iA = thisJoint.b.weldJointData.invIA, iB = thisJoint.b.weldJointData.invIB;

	b2clMat33 K;
	K.ex[0] = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
	K.ey[0] = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
	K.ez[0] = -m_rA.y * iA - m_rB.y * iB;
	K.ex[1] = K.ey[0];
	K.ey[1] = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
	K.ez[1] = m_rA.x * iA + m_rB.x * iB;
	K.ex[2] = K.ez[0];
	K.ey[2] = K.ez[1];
	K.ez[2] = iA + iB;

	if (thisJoint.b.weldJointData.frequencyHz > 0.0f)
	{
		b2clGetInverse22(K, &thisJoint.b.weldJointData.mass);

		float invM = iA + iB;
		float m = invM > 0.0f ? 1.0f / invM : 0.0f;

		float C = aB - aA - thisJoint.b.weldJointData.referenceAngle;

		// Frequency
		float omega = 2.0f * b2_pi * thisJoint.b.weldJointData.frequencyHz;

		// Damping coefficient
		float d = 2.0f * m * thisJoint.b.weldJointData.dampingRatio * omega;

		// Spring stiffness
		float k = m * omega * omega;

		// magic formulas
		float h = dt;
		thisJoint.b.weldJointData.gamma = h * (d + h * k);
		thisJoint.b.weldJointData.gamma = thisJoint.b.weldJointData.gamma != 0.0f ? 1.0f / thisJoint.b.weldJointData.gamma : 0.0f;
		thisJoint.b.weldJointData.bias = C * h * k * thisJoint.b.weldJointData.gamma;

		invM += thisJoint.b.weldJointData.gamma;
		thisJoint.b.weldJointData.mass.ez[2] = invM != 0.0f ? 1.0f / invM : 0.0f;
	}
	else
	{
		b2clGetSymInverse33(K, &thisJoint.b.weldJointData.mass);
		thisJoint.b.weldJointData.gamma = 0.0f;
		thisJoint.b.weldJointData.bias = 0.0f;
	}

	if (warmStarting)
	{
		// Scale impulses to support a variable time step.
		thisJoint.a.x.impulse[0] *= dtRatio;
		thisJoint.a.x.impulse[1] *= dtRatio;
		thisJoint.a.x.impulse[2] *= dtRatio;

		float2 P = (float2)(thisJoint.a.x.impulse[0], thisJoint.a.x.impulse[1]);

		vA -= mA * P;
		wA -= iA * (b2clCross_VV(m_rA, P) + thisJoint.a.x.impulse[2]);

		vB += mB * P;
		wB += iB * (b2clCross_VV(m_rB, P) + thisJoint.a.x.impulse[2]);

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	}
	else
	{
		thisJoint.a.x.impulse[0] = 0.0f;
		thisJoint.a.x.impulse[1] = 0.0f;
		thisJoint.a.x.impulse[2] = 0.0f;
	}

	thisJoint.b.weldJointData.rA[0] = m_rA.x;
	thisJoint.b.weldJointData.rA[1] = m_rA.y;
	thisJoint.b.weldJointData.rB[0] = m_rB.x;
	thisJoint.b.weldJointData.rB[1] = m_rB.y;

	jointListBuffer[contactIndex] = thisJoint;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeMouseJointVelocityConstraint
//
// Initialize values of Mouse joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeMouseJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						int warmStarting,
						float dt,
						float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength
						//__global testData* testBuffer 
						)
{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex >= offset + colorLength) 
		return;
		
	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];

	float2 m_localAnchorB = (float2) (thisJoint.b.mouseJointData.localAnchorB[0] , thisJoint.b.mouseJointData.localAnchorB[1]);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);

	float2 m_targetA = (float2) (thisJoint.b.mouseJointData.targetA[0], thisJoint.b.mouseJointData.targetA[1]);

	thisJoint.b.mouseJointData.localCenterB[0] = m_localCenterB.x;
	thisJoint.b.mouseJointData.localCenterB[1] = m_localCenterB.y;
	thisJoint.b.mouseJointData.invMassB = bodyStaticB.m_invMass; 
	thisJoint.b.mouseJointData.invIB = bodyStaticB.m_invI;

	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	float mB = thisJoint.b.mouseJointData.invMassB;
	float iB = thisJoint.b.mouseJointData.invIB;

	float mass = 1.0f / mB;

	// Frequency
	float omega = 2.0f * b2_pi * thisJoint.b.mouseJointData.frequencyHz;

	// Damping coefficient
	float d = 2.0f * mass * thisJoint.b.mouseJointData.dampingRatio * omega;

	// Spring stiffness
	float k = mass * (omega * omega);

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	float h = dt;
	thisJoint.b.mouseJointData.gamma = h * (d + h * k);
	if (thisJoint.b.mouseJointData.gamma != 0.0f)
	{
		thisJoint.b.mouseJointData.gamma = 1.0f / thisJoint.b.mouseJointData.gamma;
	}
	thisJoint.b.mouseJointData.beta = h * k * thisJoint.b.mouseJointData.gamma;

	// Compute the effective mass matrix.
	float2 m_rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	b2clMat22 K;
	K.ex[0] = thisJoint.b.mouseJointData.invMassB + thisJoint.b.mouseJointData.invIB * m_rB.y * m_rB.y + thisJoint.b.mouseJointData.gamma;
	K.ex[1] = -thisJoint.b.mouseJointData.invIB * m_rB.x * m_rB.y;
	K.ey[0] = K.ex[1];
	K.ey[1] = thisJoint.b.mouseJointData.invMassB + thisJoint.b.mouseJointData.invIB * m_rB.x * m_rB.x + thisJoint.b.mouseJointData.gamma;

	b2clMat22GetInverse(K, &thisJoint.b.mouseJointData.mass);

	float2 m_C = cB + m_rB - m_targetA;
	m_C *= thisJoint.b.mouseJointData.beta;

	// Cheat with some damping
	wB *= 0.98f;

	if (warmStarting)
	{
		float2 m_impulse = (float2)(thisJoint.a.x.impulse[0], thisJoint.a.x.impulse[1]);
		m_impulse *= dtRatio;
		vB += thisJoint.b.mouseJointData.invMassB * m_impulse;
		wB += thisJoint.b.mouseJointData.invIB * b2clCross_VV(m_rB, m_impulse);

		thisJoint.a.x.impulse[0] = m_impulse.x;
		thisJoint.a.x.impulse[1] = m_impulse.y;

		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	}
	else
	{
		thisJoint.a.x.impulse[0] = 0.0f;
		thisJoint.a.x.impulse[1] = 0.0f;
	}

	thisJoint.b.mouseJointData.rB[0] = m_rB.x;
	thisJoint.b.mouseJointData.rB[1] = m_rB.y;
	thisJoint.b.mouseJointData.C[0] = m_C.x;
	thisJoint.b.mouseJointData.C[1] = m_C.y;
	jointListBuffer[contactIndex] = thisJoint;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// InitializeFrictionJointVelocityConstraint
//
// Initialize values of Friction joints for the solver computation.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void InitializeFrictionJointVelocityConstraint(
						__global clb2Velocity* velocities,
						const __global clb2Position* positions,
						const __global b2clBodyStatic* bodyStaticListBuffer,
						const __global b2clTransform* xfGlobal,
						__global b2clJoint* jointListBuffer,
						//int numContact, 
						const int warmStarting,
						const float dt,
						const float dtRatio,
						const unsigned int offset,
						const unsigned int colorLength
						//__global testData* testBuffer 
						)
{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex >= offset + colorLength) 
		return;
	
	b2clJoint thisJoint = jointListBuffer[contactIndex]; 
	b2clBodyStatic bodyStaticA = bodyStaticListBuffer[thisJoint.indexA];
	b2clBodyStatic bodyStaticB = bodyStaticListBuffer[thisJoint.indexB];

	b2clTransform xfA = xfGlobal[thisJoint.indexA];
	b2clTransform xfB = xfGlobal[thisJoint.indexB];

	clb2Position positionA = positions[thisJoint.indexA]; clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Position positionB = positions[thisJoint.indexB]; clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 cA =  (float2)(positionA.cx, positionA.cy);   float aA = positionA.a;
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
	float2 cB =  (float2)(positionB.cx, positionB.cy);  float aB = positionB.a;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy);  float wB = velocityB.w;

	float2 m_localAnchorA = (float2) (thisJoint.b.frictionJointData.localAnchorA[0] , thisJoint.b.frictionJointData.localAnchorA[1]);
	float2 m_localAnchorB = (float2) (thisJoint.b.frictionJointData.localAnchorB[0] , thisJoint.b.frictionJointData.localAnchorB[1]);
	float2 m_localCenterA = (float2) (bodyStaticA.m_localCenter.x, bodyStaticA.m_localCenter.y);
	float2 m_localCenterB = (float2) (bodyStaticB.m_localCenter.x, bodyStaticB.m_localCenter.y);
	thisJoint.b.frictionJointData.localCenterA[0] = m_localCenterA.x;
	thisJoint.b.frictionJointData.localCenterA[1] = m_localCenterA.y;
	thisJoint.b.frictionJointData.localCenterB[0] = m_localCenterB.x;
	thisJoint.b.frictionJointData.localCenterB[1] = m_localCenterB.y;
	thisJoint.b.frictionJointData.invMassA = bodyStaticA.m_invMass;
	thisJoint.b.frictionJointData.invMassB = bodyStaticB.m_invMass; 
	thisJoint.b.frictionJointData.invIA = bodyStaticA.m_invI;
	thisJoint.b.frictionJointData.invIB = bodyStaticB.m_invI;

	float2 m_rA = b2clMul_Rotate( xfA.q, m_localAnchorA - m_localCenterA);
	float2 m_rB = b2clMul_Rotate( xfB.q, m_localAnchorB - m_localCenterB);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	float mA = thisJoint.b.frictionJointData.invMassA, mB = thisJoint.b.frictionJointData.invMassB;
	float iA = thisJoint.b.frictionJointData.invIA, iB = thisJoint.b.frictionJointData.invIB;

	b2clMat22 K;
	K.ex[0] = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
	K.ex[1] = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
	K.ey[0] = K.ex[1];
	K.ey[1] = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

	b2clMat22GetInverse(K, &thisJoint.b.frictionJointData.linearMass);

	thisJoint.b.frictionJointData.angularMass = iA + iB;
	if (thisJoint.b.frictionJointData.angularMass > 0.0f)
	{
		thisJoint.b.frictionJointData.angularMass = 1.0f / thisJoint.b.frictionJointData.angularMass;
	}

	if (warmStarting)
	{
		// Scale impulses to support a variable time step.
		thisJoint.a.z.linearImpulse[0] *= dtRatio;
		thisJoint.a.z.linearImpulse[1] *= dtRatio;
		thisJoint.a.z.angularImpulse *= dtRatio;

		float2 P = (float2)(thisJoint.a.z.linearImpulse[0], thisJoint.a.z.linearImpulse[1]);
		vA -= mA * P;
		wA -= iA * (b2clCross_VV(m_rA, P) + thisJoint.a.z.angularImpulse);
		vB += mB * P;
		wB += iB * (b2clCross_VV(m_rB, P) + thisJoint.a.z.angularImpulse);

		velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
		velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	}
	else
	{
		thisJoint.a.z.linearImpulse[0] = 0.0f;
		thisJoint.a.z.linearImpulse[1] = 0.0f;
		thisJoint.a.z.angularImpulse = 0.0f;
	}

	thisJoint.b.frictionJointData.rA[0] = m_rA.x;
	thisJoint.b.frictionJointData.rA[1] = m_rA.y;
	thisJoint.b.frictionJointData.rB[0] = m_rB.x;
	thisJoint.b.frictionJointData.rB[1] = m_rB.y;

	jointListBuffer[contactIndex] = thisJoint;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveDistanceJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Distance joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveDistanceJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 float dt, 
                                 const unsigned int offset,
                                 const unsigned int colorLength
								// __global testData* testBuffer                                 
								 )

{

 //printf ("impulse \n");
    unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) return;

	b2clJoint thisJoint = jointList[contactIndex]; 

	clb2Velocity velocityA = velocities[thisJoint.indexA];  clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w;

	float2 m_rA = (float2) (thisJoint.b.distanceJointData.rA[0] , thisJoint.b.distanceJointData.rA[1]);
	float2 m_rB = (float2) (thisJoint.b.distanceJointData.rB[0] , thisJoint.b.distanceJointData.rB[1]); 
	float2 m_u  = (float2) (thisJoint.b.distanceJointData.u[0] , thisJoint.b.distanceJointData.u[1] );

	float2 vpA = vA +  b2clCross_SV(wA, m_rA); 
	float2 vpB = vB +  b2clCross_SV(wB, m_rB);
	float Cdot =  b2clDot ( m_u, vpB - vpA );
	
	float impulse = -thisJoint.b.distanceJointData.mass * (Cdot + thisJoint.b.distanceJointData.bias + thisJoint.b.distanceJointData.gamma * thisJoint.a.y.scalarImpulse) ;
	thisJoint.a.y.scalarImpulse += impulse ; 
	 

	float2 P = (float2) (impulse * m_u.x, impulse * m_u.y);

	vA -= (float2)(thisJoint.b.distanceJointData.invMassA* P.x , thisJoint.b.distanceJointData.invMassA* P.y);
	wA -=  thisJoint.b.distanceJointData.invIA * b2clCross_VV(m_rA, P) ; 
	vB += (float2)(thisJoint.b.distanceJointData.invMassB* P.x , thisJoint.b.distanceJointData.invMassB * P.y);
	wB += thisJoint.b.distanceJointData.invIB * b2clCross_VV(m_rB, P); 
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	jointList[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveRevoluteJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Revolute joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveRevoluteJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 const float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
			
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;
	

    if(contactIndex>=offset+colorLength) 
		return;
	b2clJoint thisJoint = jointList[contactIndex]; 

	clb2Velocity velocityA = velocities[thisJoint.indexA];  clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA =  (float2)(velocityA.vx, velocityA.vy);   float wA = velocityA.w;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w;
	float2 m_rA = (float2) (thisJoint.b.revoluteJointData.rA[0] , thisJoint.b.revoluteJointData.rA[1]);
	float2 m_rB = (float2) (thisJoint.b.revoluteJointData.rB[0] , thisJoint.b.revoluteJointData.rB[1]); 

	float mA = thisJoint.b.revoluteJointData.invMassA, mB = thisJoint.b.revoluteJointData.invMassB;
	float iA = thisJoint.b.revoluteJointData.invIA, iB = thisJoint.b.revoluteJointData.invIB;

	bool fixedRotation = (iA + iB == 0.0f);

	// Solve motor constraint.
	if (thisJoint.b.revoluteJointData.enableMotor && thisJoint.b.revoluteJointData.limitState != e_equalLimits && fixedRotation == false)
	{
		float Cdot = wB - wA - thisJoint.b.revoluteJointData.motorSpeed;
		float impulse = -thisJoint.b.revoluteJointData.motorMass * Cdot;
		float oldImpulse = thisJoint.motorImpulse;
		float maxImpulse = dt * thisJoint.b.revoluteJointData.maxMotorTorque;
		thisJoint.motorImpulse = b2clClamp(thisJoint.motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = thisJoint.motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve limit constraint.
	if (thisJoint.b.revoluteJointData.enableLimit && thisJoint.b.revoluteJointData.limitState != e_inactiveLimit && fixedRotation == false)
	{
		float2 Cdot1 = vB + b2clCross_SV(wB, m_rB) - vA - b2clCross_SV(wA, m_rA);
		float Cdot2 = wB - wA;
		float3 Cdot = (float3)(Cdot1.x, Cdot1.y, Cdot2);

		float3 impulse = -b2clMat33Solve(thisJoint.b.revoluteJointData.mass, Cdot);

		if (thisJoint.b.revoluteJointData.limitState == e_equalLimits)
		{
			thisJoint.a.x.impulse[0] += impulse.x;
			thisJoint.a.x.impulse[1] += impulse.y;
			thisJoint.a.x.impulse[2] += impulse.z;
		}
		else if (thisJoint.b.revoluteJointData.limitState == e_atLowerLimit)
		{
			float newImpulse = thisJoint.a.x.impulse[2] + impulse.z;
			if (newImpulse < 0.0f)
			{
				float2 rhs = -Cdot1 + thisJoint.a.x.impulse[2] * (float2)(thisJoint.b.revoluteJointData.mass.ez[0], thisJoint.b.revoluteJointData.mass.ez[1]);
				float2 reduced = b2clMat33Solve22(thisJoint.b.revoluteJointData.mass, rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -thisJoint.a.x.impulse[2];
				thisJoint.a.x.impulse[0] += reduced.x;
				thisJoint.a.x.impulse[1] += reduced.y;
				thisJoint.a.x.impulse[2] = 0.0f;
			}
			else
			{
				thisJoint.a.x.impulse[0] += impulse.x;
				thisJoint.a.x.impulse[1] += impulse.y;
				thisJoint.a.x.impulse[2] += impulse.z;
			}
		}
		else if (thisJoint.b.revoluteJointData.limitState == e_atUpperLimit)
		{
			float newImpulse = thisJoint.a.x.impulse[2] + impulse.z;
			if (newImpulse > 0.0f)
			{
				float2 rhs = -Cdot1 + thisJoint.a.x.impulse[2] * (float2)(thisJoint.b.revoluteJointData.mass.ez[0], thisJoint.b.revoluteJointData.mass.ez[1]);
				float2 reduced = b2clMat33Solve22(thisJoint.b.revoluteJointData.mass, rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -thisJoint.a.x.impulse[2];
				thisJoint.a.x.impulse[0] += reduced.x;
				thisJoint.a.x.impulse[1] += reduced.y;
				thisJoint.a.x.impulse[2] = 0.0f;
			}
			else
			{
				thisJoint.a.x.impulse[0] += impulse.x;
				thisJoint.a.x.impulse[1] += impulse.y;
				thisJoint.a.x.impulse[2] += impulse.z;
			}
		}

		float2 P = (float2)(impulse.x, impulse.y);

		vA -= mA * P;
		wA -= iA * (b2clCross_VV(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += iB * (b2clCross_VV(m_rB, P) + impulse.z);
	}
	else
	{
		// Solve point-to-point constraint
		float2 Cdot = vB + b2clCross_SV(wB, m_rB) - vA - b2clCross_SV(wA, m_rA);
		float2 impulse = b2clMat33Solve22(thisJoint.b.revoluteJointData.mass, -Cdot);

		thisJoint.a.x.impulse[0] += impulse.x;
		thisJoint.a.x.impulse[1] += impulse.y;

		vA -= mA * impulse;
		wA -= iA * b2clCross_VV(m_rA, impulse);

		vB += mB * impulse;
		wB += iB * b2clCross_VV(m_rB, impulse);
	}
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 

	jointList[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolvePrismaticJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Prismatic joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolvePrismaticJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								  const float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
								
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;

	b2clJoint thisJoint = jointList[contactIndex]; 

	clb2Velocity velocityA = velocities[thisJoint.indexA];  clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA =  (float2)(velocityA.vx, velocityA.vy); float wA = velocityA.w;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w; 

	float mA = thisJoint.b.prismaticJointData.invMassA, mB = thisJoint.b.prismaticJointData.invMassB;
	float iA = thisJoint.b.prismaticJointData.invIA, iB = thisJoint.b.prismaticJointData.invIB;

	const float2 m_axis = (float2) (thisJoint.b.prismaticJointData.axis[0] , thisJoint.b.prismaticJointData.axis[1]);
	const float2 m_perp = (float2) (thisJoint.b.prismaticJointData.perp[0] , thisJoint.b.prismaticJointData.perp[1]);

	// Solve linear motor constraint.
	if (thisJoint.b.prismaticJointData.enableMotor && thisJoint.b.prismaticJointData.limitState != e_equalLimits)
	{
		float Cdot = b2clDot(m_axis, vB - vA) + thisJoint.b.prismaticJointData.a2 * wB - thisJoint.b.prismaticJointData.a1 * wA;
		float impulse = thisJoint.b.prismaticJointData.motorMass * (thisJoint.b.prismaticJointData.motorSpeed - Cdot);
		float oldImpulse = thisJoint.motorImpulse;
		float maxImpulse = dt * thisJoint.b.prismaticJointData.maxMotorForce;
		thisJoint.motorImpulse = b2clClamp(thisJoint.motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = thisJoint.motorImpulse - oldImpulse;

		float2 P = impulse * m_axis;
		float LA = impulse * thisJoint.b.prismaticJointData.a1;
		float LB = impulse * thisJoint.b.prismaticJointData.a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	float2 Cdot1;
	Cdot1.x = b2clDot(m_perp, vB - vA) + thisJoint.b.prismaticJointData.s2 * wB - thisJoint.b.prismaticJointData.s1 * wA;
	Cdot1.y = wB - wA;

	if (thisJoint.b.prismaticJointData.enableLimit && thisJoint.b.prismaticJointData.limitState != e_inactiveLimit)
	{
		// Solve prismatic and limit constraint in block form.
		float Cdot2;
		Cdot2 = b2clDot(m_axis, vB - vA) + thisJoint.b.prismaticJointData.a2 * wB - thisJoint.b.prismaticJointData.a1 * wA;
		float3 Cdot = (float3)(Cdot1.x, Cdot1.y, Cdot2);

		float3 f1 = (float3)(thisJoint.a.x.impulse[0], thisJoint.a.x.impulse[1], thisJoint.a.x.impulse[2]);
		float3 df =  b2clMat33Solve(thisJoint.b.prismaticJointData.K, -Cdot);
		thisJoint.a.x.impulse[0] += df.x;
		thisJoint.a.x.impulse[1] += df.y;
		thisJoint.a.x.impulse[2] += df.z;

		if (thisJoint.b.prismaticJointData.limitState == e_atLowerLimit)
		{
			thisJoint.a.x.impulse[2] = max(thisJoint.a.x.impulse[2], 0.0f);
		}
		else if (thisJoint.b.prismaticJointData.limitState == e_atUpperLimit)
		{
			thisJoint.a.x.impulse[2] = min(thisJoint.a.x.impulse[2], 0.0f);
		}

		// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		float2 b = -Cdot1 - (thisJoint.a.x.impulse[2] - f1.z) * (float2)(thisJoint.b.prismaticJointData.K.ez[0], thisJoint.b.prismaticJointData.K.ez[1]);
		float2 f2r = b2clMat33Solve22(thisJoint.b.prismaticJointData.K, b) + (float2)(f1.x, f1.y);
		thisJoint.a.x.impulse[0] = f2r.x;
		thisJoint.a.x.impulse[1] = f2r.y;

		df.x = thisJoint.a.x.impulse[0] - f1.x;
		df.y = thisJoint.a.x.impulse[1] - f1.y;
		df.z = thisJoint.a.x.impulse[2] - f1.z;

		float2 P = df.x * m_perp + df.z * m_axis;
		float LA = df.x * thisJoint.b.prismaticJointData.s1 + df.y + df.z * thisJoint.b.prismaticJointData.a1;
		float LB = df.x * thisJoint.b.prismaticJointData.s2 + df.y + df.z * thisJoint.b.prismaticJointData.a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}
	else
	{
		// Limit is inactive, just solve the prismatic constraint in block form.
		float2 df = b2clMat33Solve22(thisJoint.b.prismaticJointData.K, -Cdot1);
		thisJoint.a.x.impulse[0] += df.x;
		thisJoint.a.x.impulse[1] += df.y;

		float2 P = df.x * m_perp;
		float LA = df.x * thisJoint.b.prismaticJointData.s1 + df.y;
		float LB = df.x * thisJoint.b.prismaticJointData.s2 + df.y;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;

		float2 Cdot10 = Cdot1;

		Cdot1.x = b2clDot(m_perp, vB - vA) + thisJoint.b.prismaticJointData.s2 * wB - thisJoint.b.prismaticJointData.s1 * wA;
		Cdot1.y = wB - wA;

		if (b2clAbs(Cdot1.x) > 0.01f || b2clAbs(Cdot1.y) > 0.01f)
		{
			float2 test = b2clMat33Mul22(thisJoint.b.prismaticJointData.K, df);
			Cdot1.x += 0.0f;
		}
	}
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 

	jointList[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveGearJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Gear joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveGearJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 const float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
								 
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;

	b2clJoint thisJoint = jointList[contactIndex]; 

	clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	clb2Velocity velocityC = velocities[thisJoint.indexC];  
	clb2Velocity velocityD = velocities[thisJoint.indexD];
	float2 vA =  (float2)(velocityA.vx, velocityA.vy); float wA = velocityA.w;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w; 
	float2 vC =  (float2)(velocityC.vx, velocityC.vy); float wC = velocityC.w;
    float2 vD =  (float2)(velocityD.vx, velocityD.vy); float wD = velocityD.w; 

	float2 m_JvAC = (float2)(thisJoint.b.gearJointData.JvAC[0], thisJoint.b.gearJointData.JvAC[1]);
	float2 m_JvBD = (float2)(thisJoint.b.gearJointData.JvBD[0], thisJoint.b.gearJointData.JvBD[1]);

	float Cdot = b2clDot(m_JvAC, vA - vC) + b2clDot(m_JvBD, vB - vD);
	Cdot += (thisJoint.b.gearJointData.JwA * wA - thisJoint.b.gearJointData.JwC * wC) + (thisJoint.b.gearJointData.JwB * wB - thisJoint.b.gearJointData.JwD * wD);

	float impulse = -thisJoint.b.gearJointData.mass * Cdot;
	thisJoint.a.y.scalarImpulse += impulse;

	vA += (thisJoint.b.gearJointData.mA * impulse) * m_JvAC;
	wA += thisJoint.b.gearJointData.iA * impulse * thisJoint.b.gearJointData.JwA;
	vB += (thisJoint.b.gearJointData.mB * impulse) * m_JvBD;
	wB += thisJoint.b.gearJointData.iB * impulse * thisJoint.b.gearJointData.JwB;
	vC -= (thisJoint.b.gearJointData.mC * impulse) * m_JvAC;
	wC -= thisJoint.b.gearJointData.iC * impulse * thisJoint.b.gearJointData.JwC;
	vD -= (thisJoint.b.gearJointData.mD * impulse) * m_JvBD;
	wD -= thisJoint.b.gearJointData.iD * impulse * thisJoint.b.gearJointData.JwD;
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	velocityC.vx = vC.x ; velocityC.vy = vC.y ; velocityC.w = wC ; velocities[thisJoint.indexC] = velocityC; 
	velocityD.vx = vD.x ; velocityD.vy = vD.y ; velocityD.w = wD ; velocities[thisJoint.indexD] = velocityD; 

	jointList[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolvePulleyJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Pulley joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolvePulleyJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 float dt,	
                                 unsigned int offset,
                                 unsigned int colorLength
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;

	b2clJoint thisJoint = jointList[contactIndex]; 
	clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA =  (float2)(velocityA.vx, velocityA.vy); float wA = velocityA.w;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w; 
	float2 rA = (float2)(thisJoint.b.pulleyJointData.rA[0] , thisJoint.b.pulleyJointData.rA[1]); float2 rB = (float2) (thisJoint.b.pulleyJointData.rB[0], thisJoint.b.pulleyJointData.rB[1]);
	float2 uA = (float2)(thisJoint.b.pulleyJointData.uA[0] , thisJoint.b.pulleyJointData.uA[1]); float2 uB = (float2) (thisJoint.b.pulleyJointData.uB[0], thisJoint.b.pulleyJointData.uB[1]);

	float2 vpA = vA + b2clCross_SV (wA,rA);
	float2 vpB = vB + b2clCross_SV (wB,rB);
	
	float Cdot = 0 - b2clDot (uA, vpA) - thisJoint.b.pulleyJointData.ratio * b2clDot (uB, vpB); 
	float impulse = -thisJoint.b.pulleyJointData.mass * Cdot ; 
	thisJoint.a.y.scalarImpulse += impulse ; 
	

	float2 PA = -impulse * uA ; 
	float2 PB = -thisJoint.b.pulleyJointData.ratio*impulse * uB ; 
	vA += (float2) (thisJoint.b.pulleyJointData.invMassA * PA.x, thisJoint.b.pulleyJointData.invMassA * PA.y ); 
	wA += thisJoint.b.pulleyJointData.invIA * b2clCross_VV( rA, PA ) ; 
	vB += (float2) (thisJoint.b.pulleyJointData.invMassB * PB.x , thisJoint.b.pulleyJointData.invMassB * PB.y); 
	wB += thisJoint.b.pulleyJointData.invIB * b2clCross_VV (rB, PB);
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	jointList[contactIndex] = thisJoint ; 

}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveRopeJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Rope joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveRopeJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 const float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;

	b2clJoint thisJoint = jointList[contactIndex]; 
	clb2Velocity velocityA = velocities[thisJoint.indexA];  
	clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA =  (float2)(velocityA.vx, velocityA.vy); float wA = velocityA.w;
    float2 vB =  (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w; 
	float2 rA = (float2)(thisJoint.b.ropeJointData.rA[0] , thisJoint.b.ropeJointData.rA[1]); float2 rB = (float2) (thisJoint.b.ropeJointData.rB[0], thisJoint.b.ropeJointData.rB[1]);
	float2 u = (float2)(thisJoint.b.ropeJointData.u[0] , thisJoint.b.ropeJointData.u[1]);  

	
	float2 vpA = vA + b2clCross_SV (wA,rA);
	float2 vpB = vB + b2clCross_SV (wB,rB);
	float C = thisJoint.b.ropeJointData.nlength - thisJoint.b.ropeJointData.maxLength ; 
		
	float Cdot =  b2clDot (u, vpB- vpA) ; 
	if (C < 0.0f) {
		float inv_dt =   dt!=0 ?1/dt:0; 
		Cdot += C * inv_dt ; 
	}
	float impulse = -thisJoint.b.ropeJointData.mass * Cdot ; 
	float oldImpulse = thisJoint.a.y.scalarImpulse ; 
	float newImpulse = thisJoint.a.y.scalarImpulse + impulse;
	thisJoint.a.y.scalarImpulse =   newImpulse < 0 ? newImpulse : 0 ;
	impulse = thisJoint.a.y.scalarImpulse - oldImpulse ; 


	
	
	float2 P = impulse * u ; 
	
	vA -= (float2) (thisJoint.b.ropeJointData.invMassA * P.x, thisJoint.b.ropeJointData.invMassA * P.y ); 
	wA -= thisJoint.b.ropeJointData.invIA * b2clCross_VV( rA, P ) ; 
	vB += (float2) (thisJoint.b.ropeJointData.invMassB * P.x , thisJoint.b.ropeJointData.invMassB * P.y); 
	wB += thisJoint.b.ropeJointData.invIB * b2clCross_VV (rB, P);
	
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	jointList[contactIndex] = thisJoint ; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveWheelJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Wheel joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveWheelJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;

	b2clJoint thisJoint = jointList[contactIndex]; 

	clb2Velocity velocityA = velocities[thisJoint.indexA];  clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA = (float2)(velocityA.vx, velocityA.vy); float wA = velocityA.w;
    float2 vB = (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w;

	float mA = thisJoint.b.wheelJointData.invMassA, mB = thisJoint.b.wheelJointData.invMassB;
	float iA = thisJoint.b.wheelJointData.invIA, iB = thisJoint.b.wheelJointData.invIB;

	const float2 m_ax = (float2)(thisJoint.b.wheelJointData.ax[0], thisJoint.b.wheelJointData.ax[1]);
	const float2 m_ay = (float2)(thisJoint.b.wheelJointData.ay[0], thisJoint.b.wheelJointData.ay[1]);
	
	// Solve spring constraint
	{
		float Cdot = b2clDot(m_ax, vB - vA) + thisJoint.b.wheelJointData.sBx * wB - thisJoint.b.wheelJointData.sAx * wA;
		float impulse = -thisJoint.b.wheelJointData.springMass * (Cdot + thisJoint.b.wheelJointData.bias + thisJoint.b.wheelJointData.gamma * thisJoint.a.y.springImpulse);
		thisJoint.a.y.springImpulse += impulse;

		float2 P = impulse * m_ax;
		float LA = impulse * thisJoint.b.wheelJointData.sAx;
		float LB = impulse * thisJoint.b.wheelJointData.sBx;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}
	
	// Solve rotational motor constraint
	{
		float Cdot = wB - wA - thisJoint.b.wheelJointData.motorSpeed;
		float impulse = -thisJoint.b.wheelJointData.motorMass * Cdot;

		float oldImpulse = thisJoint.motorImpulse;
		float maxImpulse = dt * thisJoint.b.wheelJointData.maxMotorTorque;
		thisJoint.motorImpulse = b2clClamp(thisJoint.motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = thisJoint.motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}
	
	// Solve point to line constraint
	{
		float Cdot = b2clDot(m_ay, vB - vA) + thisJoint.b.wheelJointData.sBy * wB - thisJoint.b.wheelJointData.sAy * wA;
		float impulse = -thisJoint.b.wheelJointData.mass * Cdot;
		thisJoint.a.y.scalarImpulse += impulse;

		float2 P = impulse * m_ay;
		float LA = impulse * thisJoint.b.wheelJointData.sAy;
		float LB = impulse * thisJoint.b.wheelJointData.sBy;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	
	jointList[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveWeldJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Weld joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveWeldJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								  const float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;

	b2clJoint thisJoint = jointList[contactIndex]; 
	
	clb2Velocity velocityA = velocities[thisJoint.indexA];  clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA = (float2)(velocityA.vx, velocityA.vy); float wA = velocityA.w;
    float2 vB = (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w;

	float mA = thisJoint.b.weldJointData.invMassA, mB = thisJoint.b.weldJointData.invMassB;
	float iA = thisJoint.b.weldJointData.invIA, iB = thisJoint.b.weldJointData.invIB;

	float2 m_rA = (float2)(thisJoint.b.weldJointData.rA[0], thisJoint.b.weldJointData.rA[1]);
	float2 m_rB = (float2)(thisJoint.b.weldJointData.rB[0], thisJoint.b.weldJointData.rB[1]);

	if (thisJoint.b.weldJointData.frequencyHz > 0.0f)
	{
		float Cdot2 = wB - wA;

		float impulse2 = -thisJoint.b.weldJointData.mass.ez[2] * (Cdot2 + thisJoint.b.weldJointData.bias + thisJoint.b.weldJointData.gamma * thisJoint.a.x.impulse[2]);
		thisJoint.a.x.impulse[2] += impulse2;

		wA -= iA * impulse2;
		wB += iB * impulse2;

		float2 Cdot1 = vB + b2clCross_SV(wB, m_rB) - vA - b2clCross_SV(wA, m_rA);

		float2 impulse1 = -b2clMat33MulV2(thisJoint.b.weldJointData.mass, Cdot1);
		thisJoint.a.x.impulse[0] += impulse1.x;
		thisJoint.a.x.impulse[1] += impulse1.y;

		float2 P = impulse1;

		vA -= mA * P;
		wA -= iA * b2clCross_VV(m_rA, P);

		vB += mB * P;
		wB += iB * b2clCross_VV(m_rB, P);
	}
	else
	{
		float2 Cdot1 = vB + b2clCross_SV(wB, m_rB) - vA - b2clCross_SV(wA, m_rA);
		float Cdot2 = wB - wA;
		float3 Cdot = (float3)(Cdot1.x, Cdot1.y, Cdot2);

		float3 impulse = -b2clMat33MulV3(thisJoint.b.weldJointData.mass, Cdot);
		thisJoint.a.x.impulse[0] += impulse.x;
		thisJoint.a.x.impulse[1] += impulse.y;
		thisJoint.a.x.impulse[2] += impulse.z;

		float2 P = (float2)(impulse.x, impulse.y);

		vA -= mA * P;
		wA -= iA * (b2clCross_VV(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += iB * (b2clCross_VV(m_rB, P) + impulse.z);
	}
	
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	
	jointList[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveMouseJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Mouse joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveMouseJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;
		
	b2clJoint thisJoint = jointList[contactIndex]; 

	clb2Velocity velocityB = velocities[thisJoint.indexB]; 
    float2 vB =  (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w; 

	float2 m_impulse = (float2)(thisJoint.a.x.impulse[0], thisJoint.a.x.impulse[1]);
	const float2 m_rB = (float2)(thisJoint.b.mouseJointData.rB[0], thisJoint.b.mouseJointData.rB[1]);
	const float2 m_C = (float2)(thisJoint.b.mouseJointData.C[0], thisJoint.b.mouseJointData.C[1]);

	// Cdot = v + cross(w, r)
	float2 Cdot = vB + b2clCross_SV(wB, m_rB);
	float2 impulse = b2clMat22Mul(thisJoint.b.mouseJointData.mass, -(Cdot + m_C + thisJoint.b.mouseJointData.gamma * m_impulse));

	float2 oldImpulse = m_impulse;
	m_impulse += impulse;
	float maxImpulse = dt * thisJoint.b.mouseJointData.maxForce;
	if (b2clDot(m_impulse, m_impulse) > maxImpulse * maxImpulse)
	{
		m_impulse *= maxImpulse / sqrt(b2clDot(m_impulse, m_impulse));
	}
	impulse = m_impulse - oldImpulse;

	vB += thisJoint.b.mouseJointData.invMassB * impulse;
	wB += thisJoint.b.mouseJointData.invIB * b2clCross_VV(m_rB, impulse);
	
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 

	thisJoint.a.x.impulse[0] = m_impulse.x;
	thisJoint.a.x.impulse[1] = m_impulse.y;
	jointList[contactIndex] = thisJoint; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SolveFrictionJointVelocityConstraint
//
// Compute velocities of bodies which are constrained by a Friction joint.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void SolveFrictionJointVelocityConstraint(
                                 __global clb2Velocity* velocities,
                                 __global b2clJoint* jointList,
								 float dt,
                                 const unsigned int offset,
                                 const unsigned int colorLength
								// __global testData* testBuffer                                 
								 )

{
	unsigned int contactIndex = get_global_id(0) + offset;

    if(contactIndex>=offset+colorLength) 
		return;

	b2clJoint thisJoint = jointList[contactIndex]; 
	
	clb2Velocity velocityA = velocities[thisJoint.indexA];  clb2Velocity velocityB = velocities[thisJoint.indexB]; 
	float2 vA = (float2)(velocityA.vx, velocityA.vy); float wA = velocityA.w;
    float2 vB = (float2)(velocityB.vx, velocityB.vy); float wB = velocityB.w;

	float mA = thisJoint.b.frictionJointData.invMassA, mB = thisJoint.b.frictionJointData.invMassB;
	float iA = thisJoint.b.frictionJointData.invIA, iB = thisJoint.b.frictionJointData.invIB;

	float2 m_rA = (float2)(thisJoint.b.frictionJointData.rA[0], thisJoint.b.frictionJointData.rA[1]);
	float2 m_rB = (float2)(thisJoint.b.frictionJointData.rB[0], thisJoint.b.frictionJointData.rB[1]);

	float2 m_linearImpulse = (float2) (thisJoint.a.z.linearImpulse[0], thisJoint.a.z.linearImpulse[1]);

	float h = dt;

	// Solve angular friction
	{
		float Cdot = wB - wA;
		float impulse = -thisJoint.b.frictionJointData.angularMass * Cdot;

		float oldImpulse = thisJoint.a.z.angularImpulse;
		float maxImpulse = h * thisJoint.b.frictionJointData.maxTorque;
		thisJoint.a.z.angularImpulse = b2clClamp(thisJoint.a.z.angularImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = thisJoint.a.z.angularImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve linear friction
	{
		float2 Cdot = vB + b2clCross_SV(wB, m_rB) - vA - b2clCross_SV(wA, m_rA);

		float2 impulse = -b2clMat22Mul(thisJoint.b.frictionJointData.linearMass, Cdot);
		float2 oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		float maxImpulse = h * thisJoint.b.frictionJointData.maxForce;

		// TODO: can be optimized
		if (b2clDot(m_linearImpulse, m_linearImpulse) > maxImpulse * maxImpulse)
		{
			m_linearImpulse /= b2clDot(m_linearImpulse, m_linearImpulse);
			m_linearImpulse *= maxImpulse;
		}

		impulse = m_linearImpulse - oldImpulse;

		vA -= mA * impulse;
		wA -= iA * b2clCross_VV(m_rA, impulse);

		vB += mB * impulse;
		wB += iB * b2clCross_VV(m_rB, impulse);
	}
	
	thisJoint.a.z.linearImpulse[0] = m_linearImpulse.x;
	thisJoint.a.z.linearImpulse[1] = m_linearImpulse.y;
	velocityA.vx = vA.x ; velocityA.vy = vA.y ; velocityA.w = wA ; velocities[thisJoint.indexA] = velocityA; 
	velocityB.vx = vB.x ; velocityB.vy = vB.y ; velocityB.w = wB ; velocities[thisJoint.indexB] = velocityB; 
	
	jointList[contactIndex] = thisJoint; 
}





