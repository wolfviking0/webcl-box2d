/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/



#ifndef B2CLDEVICE_H_
#define B2CLDEVICE_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wexit-time-destructors"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"

#include <map>
#include <string>
#include "Box2D/Common/b2Settings.h"
#include <Box2D/Common/b2Math.h>

#ifdef linux
//#   include <oclUtils.h>
//#   include <shrUtils.h>
#include <OpenCL/opencl.h>
#include <stdlib.h>
#include <fstream>
#include <cstdarg>
#elif defined (__APPLE__)
#   include <OpenCL/opencl.h>
#   include <fstream>
#elif defined (_WIN32)
#   include <CL/opencl.h>
#   include <fstream>
#   include <string>
#   include <stdarg.h>
#endif

#define BOX2D_OPENCL
#define BOX2D_OPENCL_ON_CPU

#ifdef BOX2D_OPENCL
	#define BROADPHASE_OPENCL
	#define NARROWPHASE_OPENCL
	#define SOLVER_OPENCL
#endif

#if defined(NARROWPHASE_OPENCL) && defined(SOLVER_OPENCL)
   	#define SCAN_OPENCL
    //#define SPLIT_OPENCL
#endif

#define _DEBUG_TIME_STEP_TOTAL
#define _DEBUG_TIME_BROADPHASE
#define _DEBUG_TIME_NARROWPHASE
#define _DEBUG_TIME_SOLVER

//#define CCD_SD_OPENCL
//#define MULTI_SD_OPENCL
//#define MAXSDTIME 2

#if defined(_DEBUG_TIME_STEP_TOTAL) || defined(_DEBUG_TIME_NARROWPHASE) || defined(_DEBUG_TIME_SOLVER)
	#define _DEBUG_TIME
#endif

#define NOT_USE_ISLAND

#define USE_CPU_SORT
#define USE_CPU_SCAN

#if defined(BROADPHASE_OPENCL) && !defined(USE_CPU_SCAN)
	#define NO_FLOAT4
#endif

#define BITONIC_SORT_INTEL_MINNUM 512

#ifdef BOX2D_OPENCL
#define SET_JOINTS_UPDATED m_bodyB->GetWorld()->SetJointsUpdated()
#else
#define SET_JOINTS_UPDATED
#endif

#ifdef linux
#define OPENCL_BUILD_PATH "-I Box2D/Common/OpenCL/"
#elif defined(__APPLE__)
#define OPENCL_BUILD_PATH "-I /usr/local/include/"
#else
//#define OPENCL_BUILD_PATH "-I ../../Box2D/Common/OpenCL/"
#define OPENCL_BUILD_PATH "-I ../../"
#endif
//#define OPENCL_BUILD_PATH "-I /Users/l.qiu/2dphysics_git/2dPhysics/Box2DOCL/Box2D/Common/OpenCL/"

#ifndef OCL_UTILS_H // mockup some stuff missing on Apple's OpenCL SDK compared to nVidia's

static const bool shrTRUE = true;

static int b2clGetPlatformID(cl_platform_id* platform)
{
    //cl_uint numPlatforms = 0;
    //return clGetPlatformIDs(1, platform, &numPlatforms);
    
    cl_uint numPlatforms = 0;
    cl_int  status;   
    
    // Retrieve number of platforms
    status = clGetPlatformIDs(0, NULL, &numPlatforms);
    
    // Populate platforms
    return clGetPlatformIDs(numPlatforms, platform, &numPlatforms);
}

static void shrLog(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

template <typename T, typename U>
static void b2clCheckError(const T& l, const U& r)
{
	if (l!=r)
		int a = 0;
    b2Assert(l == r);
}

template <typename T>
static void b2clCheckError(const T& l, bool val)
{
    b2Assert(l == val);
}

static const char* shrFindFilePath(const char* path, void * dummy)
{
    return path;
}

static char* b2clLoadProgSource(const char* path, const char* dummy, size_t* kernelLength)
{
    static std::string buffer;
    static char line[1024] = {0};
    
    buffer.clear();
    
    std::ifstream file(path);
    
    if(!file.is_open())
    {
        return NULL;
    }
    
    while(!file.eof())
    {
        file.getline(line, sizeof(line));
        buffer.append(line);
        buffer.append("\n");
    }
    
    (*kernelLength) = buffer.size();
    
    return const_cast<char*>(buffer.c_str());
}
#endif

// Definitions of types used by OpenCL
// Put all definistions here for all CL classes use
typedef struct 
{
	float m_min[2];
	float m_max[2];
	unsigned char m_sType;
	unsigned char m_bType;
} b2clAABB;

typedef struct
{
	uint8 indexA;		///< Feature index on shapeA
	uint8 indexB;		///< Feature index on shapeB
	uint8 typeA;		///< The feature type on shapeA
	uint8 typeB;		///< The feature type on shapeB
} b2clContactFeature;

typedef union
{
	b2clContactFeature cf;
	uint32 key;					///< Used to quickly compare contact ids.
} b2clContactID;

typedef struct
{
	float localPoint[2];		///< usage depends on manifold type
	float normalImpulse;	///< the non-penetration impulse
	float tangentImpulse;	///< the friction impulse
	b2clContactID id;		///< uniquely identifies a contact point between two shapes
	float dummy; // for alignment
} b2clManifoldPoint;

typedef struct
{
	float localNormal[2];								///< not use for Type::e_points
	float localPoint[2];								///< usage depends on manifold type
	b2clManifoldPoint points[b2_maxManifoldPoints];	///< the points of contact
	int type;
	int pointCount;								///< the number of manifold points
	//float test[2];
} b2clManifold;

typedef struct 
{
	unsigned short categoryBits;
	unsigned short maskBits;
	short groupIndex;
	short dummy;
} b2clFilter;

typedef struct
{
	float m_centroid[2];
	float m_vertices[b2_maxPolygonVertices][2];
	float m_normals[b2_maxPolygonVertices][2];
	int m_type;
	float m_radius;
	int m_vertexCount;
	int m_bIsSensor;
	b2clFilter m_filter;
} b2clPolygonShape;

typedef struct
{
	float p[2];
	float q[2];
} b2clTransform;

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
typedef struct
{
	float localCenter[2];	///< local center of mass position
	float c0[2], c[2];		///< center world positions
	float a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float alpha0;
	float dummy;
}  b2clSweep;

typedef struct{
	float32 m_friction;
	float32 m_restitution;
	int32 m_last_uid;
	int32 dummy;
} b2clFixtureStatic;

#define MAX_CONNECTED_BODY_INDICES 8
typedef struct{
	float m_localCenter[2];	///< local center of mass position
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

typedef struct{
	b2clSweep m_sweep;		// the swept motion for CCD

	float m_linearVelocity[2];
	float m_force[2];
	float m_angularVelocity;
	float m_torque;
	int m_last_uid;
	int dummy;
} b2clBodyDynamic;

struct clb2ContactNode // One node corresponds to one contact
{
    int32 color;    
    int indexA;
    int indexB;
    float32 friction;
    float32 normal[2];
    
    float32 invMassA;
    float32 invIA;
    float32 invMassB;
    float32 invIB;
};

struct clb2ImpulseNode // One node corresponds to one contact (up to 2 contact points)
{
    float32 normalImpulse1;
    float32 tangentImpulse1;
    
    float32 normalImpulse2;
    float32 tangentImpulse2;
};

struct clb2PointsNode // One node corresponds to one contact (up to 2 contact points)
{
    float32 rA1[2];
    float32 rB1[2];
    float32 normalMass1;
    float32 tangentMass1;
    
    float32 rA2[2];
    float32 rB2[2];
    float32 normalMass2;
    float32 tangentMass2;
    
    float32 velocityBias1;
    float32 velocityBias2;
};

struct clb2Manifold
{
    b2Vec2 localNormal;
    b2Vec2 localPoint;
    b2Vec2 localPoints1;
    b2Vec2 localPoints2;
    int pointCount;
    int type;
    float radiusA;
    float radiusB;
    b2Vec2 localCenterA;
    b2Vec2 localCenterB;
};

//////////////////////////////







//// Only for the joint
///////////////////////////////////////////////////////////////////////
struct b2clMat22
{
	float ex[2];
	float ey[2];
};

struct b2clMat33
{
	float ex[3];
	float ey[3];
	float ez[3];
};

struct b2clDistanceJointData  
{
	float frequencyHz;
	float dampingRatio;
	float bias;

	float localAnchorA[2];
	float localAnchorB[2];
	float gamma;
	float length;

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
} ;

struct b2clRevoluteJointData
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
} ;

struct b2clPrismaticJointData
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
} ;

struct b2clGearJointData
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
} ;
struct b2clPulleyJointData
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
}; 

struct b2clRopeJointData
{	
	float localAnchorA[2];
	float localAnchorB[2]; 
	float maxLength; 
	float length;
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
}; 

struct b2clWheelJointData
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
};

struct b2clWeldJointData
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
};

struct b2clMouseJointData
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
};

struct b2clFrictionJointData
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
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// A consolidated data structure for all joint types.
// The front part of the data structure has common data(indices, impulses).
// The remaining part is an union of joint type specific data.
// All data fields use the same names in the original Box2D joint data structures(b2DistanceJoint, ...) 
// except 'scalarImpulse' which is changed from 'impulse' to avoid conflicts with impulse[3].
////////////////////////////////////////////////////////////////////////////////////////////////////
struct b2clJoint
{
	// common part begin

	// index is an unique id of a joint. 
	// Used in warm starting to find the identical joint in the last frame.
	int index;

	// impulse : 4 floats
	union {
		struct {
			float impulse[3];
		};
		struct {
			float scalarImpulse;
			float springImpulse;
		};
		struct {
			float linearImpulse[2];
			float angularImpulse;
		};
	};
	float motorImpulse;

	int color;
	int type;
	int collideConnected;

	int indexA, indexB, indexC, indexD;
	// common part end

	union {
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
	};
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// A data structure for joint impulses.
// Used in ReadLastJointImpulses() / StoreJointImpulses() for warm starting.
////////////////////////////////////////////////////////////////////////////////////////////////////
struct b2clJointImpulseNode
{
	int index;
    float impulse[4];
};

struct b2clRayCastOutput
{
	int isCollide;
	float normal[2];
	float fraction;
	unsigned int shapeIndex;
};

struct b2clDistanceProxy
{
	float m_buffer[2][2];
	float m_vertices[b2_maxPolygonVertices][2];
	int m_count;
	float m_radius;
};

struct b2clTOIInput
{
	b2clDistanceProxy proxyA;
	b2clDistanceProxy proxyB;
	b2clSweep sweepA;
	b2clSweep sweepB;
	float tMax;		// defines sweep interval [0, tMax]
	float dummy;
};

struct b2clTOIOutput
{
	int state;
	float t;
};

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


class b2CLDevice
{
public:    
	static b2CLDevice& instance();
	const cl_device_id GetCurrentDevice() const { return currentDevice; };
	const cl_context GetContext() const { return m_context; };
 	const cl_command_queue GetCommandQueue() const { return m_commandQueue; };
   
	void bitonicSort(
	    cl_mem d_DstKey,
	    cl_mem d_DstVal,
	    cl_mem d_SrcKey,
	    cl_mem d_SrcVal,
	    unsigned int batch,
	    unsigned int arrayLength,
	    unsigned int dir
	);

    void getMaximumKernelWorkGroupSize(cl_kernel& kernelName, size_t& workGroupSize);
	cl_mem allocateArray(size_t size, bool print_log=false);
	cl_mem allocateArray(size_t size, void* host_buffer, bool print_log=false);
	void freeArray(cl_mem memObj);
	void copyArrayFromDevice(void *hostPtr, cl_mem memObj, unsigned int vbo, size_t size);
	void copyArrayFromDevice(void *hostPtr, cl_mem memObj, unsigned int vbo, size_t size, bool blocking);
	void copyArrayToDevice(cl_mem memObj, const void *hostPtr, size_t offset, size_t size);
	void copyArrayToDevice(cl_mem memObj, const void *hostPtr, size_t offset, size_t size, bool blocking);
	void copyArrayInsideDevice(cl_mem src_memObj, cl_mem dst_memObj, size_t size) ; 
	void copyArrayInsideDevice(cl_mem src_memObj, cl_mem dst_memObj, size_t src_offset, size_t dst_offset, size_t size) ; 
	void finishCommandQueue();

    static const unsigned int LOCAL_SIZE_LIMIT;

private:
    b2CLDevice();
	~b2CLDevice();
    
    cl_uint  uiTargetDevice;
    cl_device_id* cdDevices;
	cl_device_id currentDevice;
	size_t m_deviceMaxWorkGroupSize;
    cl_context m_context;
    cl_command_queue m_commandQueue;

	cl_kernel m_bitonicSortLocal;
	cl_kernel m_bitonicSortLocal1;
	cl_kernel m_bitonicMergeGlobal;
	cl_kernel m_bitonicMergeLocal;
	cl_program m_bitonicSortProgram;
    
    size_t maxWorkGroupSizeForBitonicSort;

	void startupOpenCL();
	void initBitonicSort();
	void closeBitonicSort();
};

inline int ComputeGroupNum(int n, int width) { return (n + (width - 1)) / width; }

#pragma GCC diagnostic pop

#endif /* CLDEVICE_H_ */
