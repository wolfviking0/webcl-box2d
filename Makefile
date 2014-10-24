#
#  Makefile
#  Licence : https://github.com/wolfviking0/webcl-translator/blob/master/LICENSE
#
#  Created by Anthony Liot.
#  Copyright (c) 2013 Anthony Liot. All rights reserved.
#

# Default parameter
DEB = 0
VAL = 0
NAT = 0
ORIG= 0
FAST= 1

# Chdir function
CHDIR_SHELL := $(SHELL)
define chdir
   $(eval _D=$(firstword $(1) $(@D)))
   $(info $(MAKE): cd $(_D)) $(eval SHELL = cd $(_D); $(CHDIR_SHELL))
endef

# Current Folder
CURRENT_ROOT:=$(PWD)

# Emscripten Folder
EMSCRIPTEN_ROOT:=$(CURRENT_ROOT)/../webcl-translator/emscripten

# Native build
ifeq ($(NAT),1)
$(info ************ NATIVE : NO DEPENDENCIES  ************)

CXX = clang++
CC  = clang

BUILD_FOLDER = $(CURRENT_ROOT)/bin/
EXTENSION = .out

ifeq ($(DEB),1)
$(info ************ NATIVE : DEBUG = 1        ************)

CFLAGS = -O0 -framework OpenCL -framework OpenGL -framework GLUT -framework CoreFoundation -framework AppKit -framework IOKit -framework CoreVideo -framework CoreGraphics

else
$(info ************ NATIVE : DEBUG = 0        ************)

CFLAGS = -O2 -framework OpenCL -framework OpenGL -framework GLUT -framework CoreFoundation -framework AppKit -framework IOKit -framework CoreVideo -framework CoreGraphics

endif

# Emscripten build
else
ifeq ($(ORIG),1)
$(info ************ EMSCRIPTEN : SUBMODULE     = 0 ************)

EMSCRIPTEN_ROOT:=$(CURRENT_ROOT)/../emscripten
else
$(info ************ EMSCRIPTEN : SUBMODULE     = 1 ************)
endif

CXX = $(EMSCRIPTEN_ROOT)/em++
CC  = $(EMSCRIPTEN_ROOT)/emcc

BUILD_FOLDER = $(CURRENT_ROOT)/js/
EXTENSION = .js
GLOBAL =

ifeq ($(DEB),1)
$(info ************ EMSCRIPTEN : DEBUG         = 1 ************)

GLOBAL += EMCC_DEBUG=1

CFLAGS = -s OPT_LEVEL=1 -s DEBUG_LEVEL=1 -s CL_PRINT_TRACE=1 -s WARN_ON_UNDEFINED_SYMBOLS=1 -s CL_DEBUG=1 -s CL_GRAB_TRACE=1 -s CL_CHECK_VALID_OBJECT=1
else
$(info ************ EMSCRIPTEN : DEBUG         = 0 ************)

CFLAGS = -s OPT_LEVEL=3 -s DEBUG_LEVEL=0 -s CL_PRINT_TRACE=0 -s DISABLE_EXCEPTION_CATCHING=0 -s WARN_ON_UNDEFINED_SYMBOLS=1 -s CL_DEBUG=0 -s CL_GRAB_TRACE=0 -s CL_CHECK_VALID_OBJECT=0
endif

ifeq ($(VAL),1)
$(info ************ EMSCRIPTEN : VALIDATOR     = 1 ************)

PREFIX = val_

CFLAGS += -s CL_VALIDATOR=1
else
$(info ************ EMSCRIPTEN : VALIDATOR     = 0 ************)
endif

ifeq ($(FAST),1)
$(info ************ EMSCRIPTEN : FAST_COMPILER = 1 ************)

GLOBAL += EMCC_FAST_COMPILER=1
else
$(info ************ EMSCRIPTEN : FAST_COMPILER = 0 ************)
endif

endif

SOURCES_box2d = \
				$(CURRENT_ROOT)/Box2D/Collision/b2BroadPhase.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/b2CollideCircle.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/b2CollideEdge.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/b2CollidePolygon.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/b2Collision.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/b2Distance.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/b2DynamicTree.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/b2TimeOfImpact.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/Shapes/b2ChainShape.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/Shapes/b2CircleShape.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/Shapes/b2EdgeShape.cpp \
				$(CURRENT_ROOT)/Box2D/Collision/Shapes/b2PolygonShape.cpp \
				$(CURRENT_ROOT)/Box2D/Common/b2BlockAllocator.cpp \
				$(CURRENT_ROOT)/Box2D/Common/b2Draw.cpp \
				$(CURRENT_ROOT)/Box2D/Common/b2Math.cpp \
				$(CURRENT_ROOT)/Box2D/Common/b2Settings.cpp \
				$(CURRENT_ROOT)/Box2D/Common/b2StackAllocator.cpp \
				$(CURRENT_ROOT)/Box2D/Common/b2Timer.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLBroadPhase.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLCommonData.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLDevice.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLNarrowPhase.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLScan.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLSolver.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLSolveTOI.cpp \
				$(CURRENT_ROOT)/Box2D/Common/OpenCL/b2CLSort.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/b2Body.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/b2ContactManager.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/b2Fixture.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/b2Island.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/b2World.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/b2WorldCallbacks.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2CircleContact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2Contact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2ContactSolver.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Contacts/b2PolygonContact.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2DistanceJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2FrictionJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2GearJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2Joint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2MouseJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2PrismaticJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2PulleyJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2RevoluteJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2RopeJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2WeldJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Dynamics/Joints/b2WheelJoint.cpp \
				$(CURRENT_ROOT)/Box2D/Rope/b2Rope.cpp \
			

INCLUDES_helloworld = -I$(CURRENT_ROOT)

SOURCES_helloworld = $(SOURCES_box2d) HelloWorld.cpp

ifeq ($(NAT),0)

KERNEL_helloworld		= \
--preload-file Common/OpenCL/b2CLBitonicSort_Intel.cl \
--preload-file Common/OpenCL/b2CLBitonicSort_NV.cl \
--preload-file Common/OpenCL/b2CLBroadPhase.cl \
--preload-file Common/OpenCL/b2CLCommonData.cl \
--preload-file Common/OpenCL/b2CLNarrowPhase_Alone.cl \
--preload-file Common/OpenCL/b2CLNarrowPhase.cl \
--preload-file Common/OpenCL/b2CLPrefixScanFloat4.cl \
--preload-file Common/OpenCL/b2CLScan_CLPP.cl \
--preload-file Common/OpenCL/b2CLScan.cl \
--preload-file Common/OpenCL/b2CLScanKernel.cl \
--preload-file Common/OpenCL/b2CLSolvePositionConstraint.cl \
--preload-file Common/OpenCL/b2CLSolveTOI.cl \
--preload-file Common/OpenCL/b2CLSolveVelocityConstraint_Alone.cl \
--preload-file Common/OpenCL/b2CLSolveVelocityConstraint.cl \
--preload-file Common/OpenCL/BitonicSort_b.cl

CFLAGS_helloworld		=	

VALPARAM_helloworld		=

COPY_helloworld			=	mkdir -p Common/OpenCL && cp -rf $(CURRENT_ROOT)/Box2D/Common/OpenCL/*.cl Common/OpenCL/ &&

else

COPY_helloworld			=	mkdir -p $(BUILD_FOLDER)Common/OpenCL && cp -rf $(CURRENT_ROOT)/Box2D/Common/OpenCL/*.cl $(BUILD_FOLDER)Common/OpenCL/ &&

endif

.PHONY:    
	all clean

all: \
	all_1 all_2 all_3

all_1: \
	helloworld_sample

all_2: \


all_3: \


# Create build folder is necessary))
mkdir:
	mkdir -p $(BUILD_FOLDER);

helloworld_sample: mkdir
	$(call chdir,HelloWorld/)
	$(COPY_helloworld) 	$(GLOBAL) $(CXX) $(CFLAGS) $(CFLAGS_helloworld) 	$(INCLUDES_helloworld) 	$(SOURCES_helloworld) 		$(VALPARAM_helloworld) 	$(KERNEL_helloworld) 		-o $(BUILD_FOLDER)$(PREFIX)helloworld$(EXTENSION) 

clean:
	rm -rf bin/
	mkdir -p bin/
	mkdir -p tmp/
	cp js/memoryprofiler.js tmp/ && cp js/settings.js tmp/ && cp js/index.html tmp/
	rm -rf js/
	mkdir js/
	cp tmp/memoryprofiler.js js/ && cp tmp/settings.js js/ && cp tmp/index.html js/
	rm -rf tmp/
	$(EMSCRIPTEN_ROOT)/emcc --clear-cache

