
/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#include <Box2D/Common/OpenCL/b2CLTypeDefOCL.h>

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


