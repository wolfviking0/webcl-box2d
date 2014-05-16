set(BOX2D_Collision_SRCS
	Collision/b2BroadPhase.cpp
	Collision/b2CollideCircle.cpp
	Collision/b2CollideEdge.cpp
	Collision/b2CollidePolygon.cpp
	Collision/b2Collision.cpp
	Collision/b2Distance.cpp
	Collision/b2DynamicTree.cpp
	Collision/b2TimeOfImpact.cpp
)
set(BOX2D_Collision_HDRS
	Collision/b2BroadPhase.h
	Collision/b2Collision.h
	Collision/b2Distance.h
	Collision/b2DynamicTree.h
	Collision/b2TimeOfImpact.h
)
set(BOX2D_Shapes_SRCS
	Collision/Shapes/b2CircleShape.cpp
	Collision/Shapes/b2EdgeShape.cpp
	Collision/Shapes/b2ChainShape.cpp
	Collision/Shapes/b2PolygonShape.cpp
)
set(BOX2D_Shapes_HDRS
	Collision/Shapes/b2CircleShape.h
	Collision/Shapes/b2EdgeShape.h
	Collision/Shapes/b2ChainShape.h
	Collision/Shapes/b2PolygonShape.h
	Collision/Shapes/b2Shape.h
)
set(BOX2D_Common_SRCS
	Common/b2BlockAllocator.cpp
	Common/b2Draw.cpp
	Common/b2Math.cpp
	Common/b2Settings.cpp
	Common/b2StackAllocator.cpp
	Common/b2Timer.cpp
)
set(BOX2D_Common_HDRS
	Common/b2BlockAllocator.h
	Common/b2Draw.h
	Common/b2GrowableStack.h
	Common/b2Math.h
	Common/b2Settings.h
	Common/b2StackAllocator.h
	Common/b2Timer.h
)
set(BOX2D_Dynamics_SRCS
	Dynamics/b2Body.cpp
	Dynamics/b2ContactManager.cpp
	Dynamics/b2Fixture.cpp
	Dynamics/b2Island.cpp
	Dynamics/b2World.cpp
	Dynamics/b2WorldCallbacks.cpp
)
set(BOX2D_Dynamics_HDRS
	Dynamics/b2Body.h
	Dynamics/b2ContactManager.h
	Dynamics/b2Fixture.h
	Dynamics/b2Island.h
	Dynamics/b2TimeStep.h
	Dynamics/b2World.h
	Dynamics/b2WorldCallbacks.h
)
set(BOX2D_Contacts_SRCS
	Dynamics/Contacts/b2CircleContact.cpp
	Dynamics/Contacts/b2Contact.cpp
	Dynamics/Contacts/b2ContactSolver.cpp
	Dynamics/Contacts/b2PolygonAndCircleContact.cpp
	Dynamics/Contacts/b2EdgeAndCircleContact.cpp
	Dynamics/Contacts/b2EdgeAndPolygonContact.cpp
	Dynamics/Contacts/b2ChainAndCircleContact.cpp
	Dynamics/Contacts/b2ChainAndPolygonContact.cpp
	Dynamics/Contacts/b2PolygonContact.cpp
)
set(BOX2D_Contacts_HDRS
	Dynamics/Contacts/b2CircleContact.h
	Dynamics/Contacts/b2Contact.h
	Dynamics/Contacts/b2ContactSolver.h
	Dynamics/Contacts/b2PolygonAndCircleContact.h
	Dynamics/Contacts/b2EdgeAndCircleContact.h
	Dynamics/Contacts/b2EdgeAndPolygonContact.h
	Dynamics/Contacts/b2ChainAndCircleContact.h
	Dynamics/Contacts/b2ChainAndPolygonContact.h
	Dynamics/Contacts/b2PolygonContact.h
)
set(BOX2D_Joints_SRCS
	Dynamics/Joints/b2DistanceJoint.cpp
	Dynamics/Joints/b2FrictionJoint.cpp
	Dynamics/Joints/b2GearJoint.cpp
	Dynamics/Joints/b2Joint.cpp
	Dynamics/Joints/b2MouseJoint.cpp
	Dynamics/Joints/b2PrismaticJoint.cpp
	Dynamics/Joints/b2PulleyJoint.cpp
	Dynamics/Joints/b2RevoluteJoint.cpp
	Dynamics/Joints/b2RopeJoint.cpp
	Dynamics/Joints/b2WeldJoint.cpp
	Dynamics/Joints/b2WheelJoint.cpp
)
set(BOX2D_Joints_HDRS
	Dynamics/Joints/b2DistanceJoint.h
	Dynamics/Joints/b2FrictionJoint.h
	Dynamics/Joints/b2GearJoint.h
	Dynamics/Joints/b2Joint.h
	Dynamics/Joints/b2MouseJoint.h
	Dynamics/Joints/b2PrismaticJoint.h
	Dynamics/Joints/b2PulleyJoint.h
	Dynamics/Joints/b2RevoluteJoint.h
	Dynamics/Joints/b2RopeJoint.h
	Dynamics/Joints/b2WeldJoint.h
	Dynamics/Joints/b2WheelJoint.h
)
set(BOX2D_Rope_SRCS
	Rope/b2Rope.cpp
)
set(BOX2D_Rope_HDRS
	Rope/b2Rope.h
)
set(BOX2D_General_HDRS
	Box2D.h
)
set(BOX2D_OpenCL_SRCS
	Common/OpenCL/b2CLBroadPhase.cpp
	Common/OpenCL/b2CLCommonData.cpp
	Common/OpenCL/b2CLDevice.cpp
	Common/OpenCL/b2CLNarrowPhase.cpp
	Common/OpenCL/b2CLScan.cpp
	Common/OpenCL/b2CLSolveTOI.cpp
	Common/OpenCL/b2CLSolver.cpp
	Common/OpenCL/b2CLSort.cpp
)
set(BOX2D_OpenCL_HDRS
	Common/OpenCL/b2CLBroadPhase.h
	Common/OpenCL/b2CLCommonData.h
	Common/OpenCL/b2CLCompactionFuncitons.h
	Common/OpenCL/b2CLDevice.h
	Common/OpenCL/b2CLNarrowPhase.h
	Common/OpenCL/b2CLScan.h
	Common/OpenCL/b2CLSolveTOI.h
	Common/OpenCL/b2CLSolver.h
	Common/OpenCL/b2CLSort.h
	Common/OpenCL/b2CLTypeDefOCL.h
)
set(BOX2D_OpenCL_CL
	Common/OpenCL/b2CLBitonicSort_Intel.cl
	Common/OpenCL/b2CLBitonicSort_NV.cl
	Common/OpenCL/b2CLBroadPhase.cl
	Common/OpenCL/b2CLCommonData.cl
	Common/OpenCL/b2CLNarrowPhase.cl
	Common/OpenCL/b2CLNarrowPhase_Alone.cl
	Common/OpenCL/b2CLPrefixScanFloat4.cl
	Common/OpenCL/b2CLScan.cl
	Common/OpenCL/b2CLScanKernel.cl
	Common/OpenCL/b2CLScan_CLPP.cl
	Common/OpenCL/b2CLSolvePositionConstraint.cl
	Common/OpenCL/b2CLSolveTOI.cl
	Common/OpenCL/b2CLSolveVelocityConstraint.cl
	Common/OpenCL/b2CLSolveVelocityConstraint_Alone.cl
	Common/OpenCL/BitonicSort_b.cl
	
)
include_directories( ../ )

IF(APPLE)
   INCLUDE_DIRECTORIES( /System/Library/Frameworks/OpenCL.framework/Headers )
   FIND_LIBRARY(OpenCL_LIBRARY OpenCL)
   MARK_AS_ADVANCED (OpenCL_LIBRARY)
   SET(EXTRA_LIBS ${OpenCL_LIBRARY})
ENDIF (APPLE)

if(BOX2D_BUILD_SHARED)
	add_library(Box2D_shared SHARED
		${BOX2D_General_HDRS}
		${BOX2D_Joints_SRCS}
		${BOX2D_Joints_HDRS}
		${BOX2D_Contacts_SRCS}
		${BOX2D_Contacts_HDRS}
		${BOX2D_Dynamics_SRCS}
		${BOX2D_Dynamics_HDRS}
		${BOX2D_Common_SRCS}
		${BOX2D_Common_HDRS}
		${BOX2D_Shapes_SRCS}
		${BOX2D_Shapes_HDRS}
		${BOX2D_Collision_SRCS}
		${BOX2D_Collision_HDRS}
		${BOX2D_Rope_SRCS}
		${BOX2D_Rope_HDRS}
		${BOX2D_OpenCL_SRCS}
		${BOX2D_OpenCL_HDRS}
	)
	set_target_properties(Box2D_shared PROPERTIES
		OUTPUT_NAME "Box2DOCL"
		CLEAN_DIRECT_OUTPUT 1
		VERSION ${BOX2D_VERSION}
	)
	TARGET_LINK_LIBRARIES( Box2D_shared ${EXTRA_LIBS} )
endif()

if(BOX2D_BUILD_STATIC)
	add_library(Box2D STATIC
		${BOX2D_General_HDRS}
		${BOX2D_Joints_SRCS}
		${BOX2D_Joints_HDRS}
		${BOX2D_Contacts_SRCS}
		${BOX2D_Contacts_HDRS}
		${BOX2D_Dynamics_SRCS}
		${BOX2D_Dynamics_HDRS}
		${BOX2D_Common_SRCS}
		${BOX2D_Common_HDRS}
		${BOX2D_Shapes_SRCS}
		${BOX2D_Shapes_HDRS}
		${BOX2D_Collision_SRCS}
		${BOX2D_Collision_HDRS}
		${BOX2D_Rope_SRCS}
		${BOX2D_Rope_HDRS}
		${BOX2D_OpenCL_SRCS}
		${BOX2D_OpenCL_HDRS}
	)
	set_target_properties(Box2D PROPERTIES
		OUTPUT_NAME "Box2DOCL"
		CLEAN_DIRECT_OUTPUT 1
		VERSION ${BOX2D_VERSION}
	)
	TARGET_LINK_LIBRARIES( Box2D ${EXTRA_LIBS} )
endif()

# These are used to create visual studio folders.
source_group(Collision FILES ${BOX2D_Collision_SRCS} ${BOX2D_Collision_HDRS})
source_group(Collision\\Shapes FILES ${BOX2D_Shapes_SRCS} ${BOX2D_Shapes_HDRS})
source_group(Common FILES ${BOX2D_Common_SRCS} ${BOX2D_Common_HDRS})
source_group(Dynamics FILES ${BOX2D_Dynamics_SRCS} ${BOX2D_Dynamics_HDRS})
source_group(Dynamics\\Contacts FILES ${BOX2D_Contacts_SRCS} ${BOX2D_Contacts_HDRS})
source_group(Dynamics\\Joints FILES ${BOX2D_Joints_SRCS} ${BOX2D_Joints_HDRS})
source_group(Include FILES ${BOX2D_General_HDRS})
source_group(Rope FILES ${BOX2D_Rope_SRCS} ${BOX2D_Rope_HDRS})

FIND_LIBRARY(OPENCL_LIBRARIES OpenCL DOC "OpenCL lib for OSX")
FIND_PATH(OPENCL_INCLUDE_DIRS OpenCL/cl.h DOC "Include for OpenCL on OSX")
FIND_PATH(_OPENCL_CPP_INCLUDE_DIRS OpenCL/cl.hpp DOC "Include for OpenCL CPP bindings on OSX")

if(BOX2D_INSTALL)
	# install headers
	install(FILES ${BOX2D_General_HDRS} DESTINATION include/Box2D)
	install(FILES ${BOX2D_Collision_HDRS} DESTINATION include/Box2D/Collision)
	install(FILES ${BOX2D_Shapes_HDRS} DESTINATION include/Box2D/Collision/Shapes)
	install(FILES ${BOX2D_Common_HDRS} DESTINATION include/Box2D/Common)
	install(FILES ${BOX2D_Dynamics_HDRS} DESTINATION include/Box2D/Dynamics)
	install(FILES ${BOX2D_Contacts_HDRS} DESTINATION include/Box2D/Dynamics/Contacts)
	install(FILES ${BOX2D_Joints_HDRS} DESTINATION include/Box2D/Dynamics/Joints)
	install(FILES ${BOX2D_Rope_HDRS} DESTINATION include/Box2D/Rope)
	install(FILES ${BOX2D_OpenCL_HDRS} DESTINATION include/Box2D/Common/OpenCL)
	install(FILES ${BOX2D_OpenCL_CL} DESTINATION include/Box2D/Common/OpenCL)

	# install libraries
	if(BOX2D_BUILD_SHARED)
		install(TARGETS Box2D_shared EXPORT Box2D-targets DESTINATION lib)
	endif()
	if(BOX2D_BUILD_STATIC)
		install(TARGETS Box2D EXPORT Box2D-targets DESTINATION lib)
	endif()

	# install build system hooks for third-party apps
	install(EXPORT Box2D-targets DESTINATION lib/Box2D)
	install(FILES Box2DConfig.cmake DESTINATION lib/Box2D)
endif(BOX2D_INSTALL)