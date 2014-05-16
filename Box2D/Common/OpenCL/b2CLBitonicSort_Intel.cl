// Copyright (c) 2009-2011 Intel Corporation
// All rights reserved.
// 
// WARRANTY DISCLAIMER
// 
// THESE MATERIALS ARE PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEL OR ITS
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THESE
// MATERIALS, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Intel Corporation is the author of the Materials, and requests that all
// problem reports or change requests be submitted to it directly

__kernel void __attribute__((vec_type_hint(uint4))) BitonicSort(
						 __global uint4 * keyArray,
						 __global uint4 * valueArray,
						 const uint stage,
						 const uint passOfStage,
						 const uint dir,
						 const uint numWorkItems)
{
	uint i = get_global_id(0);

	if (i>=numWorkItems) 
		return;

	uint4 srcLeft, srcRight;
	uint4 mask;
	uint4 srcValueLeft, srcValueRight;
	//int4 imask10 = (int4)(0,  0, -1, -1);
	//int4 imask11 = (int4)(0, -1,  0, -1);
	uint4 imask10 = (uint4)(0,  0, 0xFFFFFFFF, 0xFFFFFFFF);
	uint4 imask11 = (uint4)(0, 0xFFFFFFFF,  0, 0xFFFFFFFF);

	if(stage > 0)
	{
		if(passOfStage > 0)	//upper level pass, exchange between two fours
		{
			uint r = 1 << (passOfStage - 1);
			uint lmask = r - 1;
			uint left = ((i>>(passOfStage-1)) << passOfStage) + (i & lmask);
			uint right = left + r;
			
			srcLeft = keyArray[left];
			srcRight = keyArray[right];
			srcValueLeft = valueArray[left];
			srcValueRight = valueArray[right];
			mask = as_uint4(srcLeft < srcRight);
			
			uint4 imin = (srcLeft & mask) | (srcRight & ~mask);
			uint4 imax = (srcLeft & ~mask) | (srcRight & mask);
			uint4 vmin = (srcValueLeft & mask) | (srcValueRight & ~mask);
			uint4 vmax = (srcValueLeft & ~mask) | (srcValueRight & mask);
			
			if( ((i>>(stage-1)) & 1) ^ dir )
			{
				keyArray[left]  = imin;
				keyArray[right] = imax;
				valueArray[left]  = vmin;
				valueArray[right] = vmax;
			}
			else
			{
				keyArray[right] = imin;
				keyArray[left]  = imax;
				valueArray[right]  = vmin;
				valueArray[left] = vmax;
			}
		}
		else	//last pass, sort inside one four
		{
			srcLeft = keyArray[i];
			srcRight = srcLeft.zwxy;
			srcValueLeft = valueArray[i];
			srcValueRight = srcValueLeft.zwxy;
			//mask = as_uint4(srcLeft < srcRight) ^ imask10;
			mask = as_uint4(srcLeft < srcRight);
			mask = mask.xyxy;

			if(((i >> stage) & 1) ^ dir)
			{
				srcLeft = (srcLeft & mask) | (srcRight & ~mask);
				srcRight = srcLeft.yxwz;
				srcValueLeft = (srcValueLeft & mask) | (srcValueRight & ~mask);
				srcValueRight = srcValueLeft.yxwz;

				//mask = as_uint4(srcLeft < srcRight) ^ imask11;
				mask = as_uint4(srcLeft < srcRight);
				mask = mask.xxzz;

				keyArray[i] = (srcLeft & mask) | (srcRight & ~mask);
				valueArray[i] = (srcValueLeft & mask) | (srcValueRight & ~mask);
			}
			else
			{
				srcLeft = (srcLeft & ~mask) | (srcRight & mask);
				srcRight = srcLeft.yxwz;
				srcValueLeft = (srcValueLeft & ~mask) | (srcValueRight & mask);
				srcValueRight = srcValueLeft.yxwz;

				//mask = as_uint4(srcLeft < srcRight) ^ imask11;
				mask = as_uint4(srcLeft < srcRight);
				mask = mask.xxzz;

				keyArray[i] = (srcLeft & ~mask) | (srcRight & mask);
				valueArray[i] = (srcValueLeft & ~mask) | (srcValueRight & mask);
			}
		}
	}
	else	//first stage, sort inside one four
	{
		//int4 imask0 = (int4)(0, -1, -1,  0);
		uint4 imask0 = (uint4)(0, 0xFFFFFFFF, 0xFFFFFFFF,  0);
		srcLeft = keyArray[i];
		srcRight = srcLeft.yxwz;
		srcValueLeft = valueArray[i];
		srcValueRight = srcValueLeft.yxwz;
		//mask = as_uint4(srcLeft < srcRight) ^ imask0;
		mask = as_uint4(srcLeft < srcRight);
		mask = mask.xxww;
		if( dir )
		{
			srcLeft = (srcLeft & mask) | (srcRight & ~mask);
			srcValueLeft = (srcValueLeft & mask) | (srcValueRight & ~mask);
		}
		else
		{
			srcLeft = (srcLeft & ~mask) | (srcRight & mask);
			srcValueLeft = (srcValueLeft & ~mask) | (srcValueRight & mask);
		}

		srcRight = srcLeft.zwxy;
		srcValueRight = srcValueLeft.zwxy;
		//mask = as_uint4(srcLeft < srcRight) ^ imask10;
		mask = as_uint4(srcLeft < srcRight);
		mask = mask.xyxy;

		if((i & 1) ^ dir)
		{
			srcLeft = (srcLeft & mask) | (srcRight & ~mask);
			srcRight = srcLeft.yxwz;
			srcValueLeft = (srcValueLeft & mask) | (srcValueRight & ~mask);
			srcValueRight = srcValueLeft.yxwz;

			//mask = as_uint4(srcLeft < srcRight) ^ imask11;
			mask = as_uint4(srcLeft < srcRight);
			mask = mask.xxzz;

			keyArray[i] = (srcLeft & mask) | (srcRight & ~mask);
			valueArray[i] = (srcValueLeft & mask) | (srcValueRight & ~mask);
		}
		else
		{
			srcLeft = (srcLeft & ~mask) | (srcRight & mask);
			srcRight = srcLeft.yxwz;
			srcValueLeft = (srcValueLeft & ~mask) | (srcValueRight & mask);
			srcValueRight = srcValueLeft.yxwz;

			//mask = as_uint4(srcLeft < srcRight) ^ imask11;
			mask = as_uint4(srcLeft < srcRight);
			mask = mask.xxzz;

			keyArray[i] = (srcLeft & ~mask) | (srcRight & mask);
			valueArray[i] = (srcValueLeft & ~mask) | (srcValueRight & mask);
		}
	}
}