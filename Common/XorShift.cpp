/*
   Copyright (c) 2010-2013, Laurent Debacker
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "XorShift.h"

/*	From http://b2d-f9r.blogspot.com/2010/08/16-bit-xorshift-rng-now-with-more.html:

	So, in Marsaglia's paper, the basic xorshift rng the code is:
	y^=(y<<a); y^=(y>>b); return (y^=(y<<c));

	I looked at this in my last post, and had a brief look at the (a,b,c) triplets to use. However,
	this only yields a period length of the word size used. In my case, that's 2^16-1 -- not much.
	However, further on in the paper, section 3.1, a method is described for longer periods through
	additional state and promotion:
	t=(x^(x<<a)); x=y; return y=(y^(y>>c))^(t^(t>>b));

	This is listed as 2^64-1 using 32-bit words, but there's no reason we can't use 16-bit words.
	This reduces the period to 2^32-1, and requires new triplets. I scanned (1,1,1) through till
	(15,15,15), here's the list:

	(1,1,7) (1,1,12) (1,1,13) (2,5,8) (2,5,13) (2,13,15) (2,15,13) (3,7,6)
	(5,3,1)* (5,3,8) (5,3,13)* (5,7,4)* (6,3,8)* (7,1,6) (7,1,15) (7,2,1)
	(8,3,9)* (9,14,5) (11,8,5)* (13,12,3) (14,1,15) (15,10,1)

	Values marked with an asterisk score well on the majority of diehard tests. It's probably not
	a good idea to use the other values, even though they do have a complete period. Notably,
	all values fail binary matrices tests for 31x31 and 32x32 - I think this is expected,
	and mentioned in Marsaglia2003.

	So, the routine in C:
	uint16_t rnd_xorshift_32() {
	  static uint16_t x=1,y=1;
	  uint16_t t=(x^(x<<5)); 
	  x=y; 
	  return y=(y^(y>>1))^(t^(t>>3));
	}
*/

XorShiftClass XorShift;

XorShiftClass::XorShiftClass()
{
	x = 0;
	y = 1;
}

void XorShiftClass::seed(uint16_t value)
{
	if(!x & !(y ^= value))
		y = 1;
}

void XorShiftClass::seed(uint32_t value)
{
	if(!(x ^= value) & !(y ^= value >> 16))
		y = 1;
}

void XorShiftClass::random(uint16_t *r)
{
	uint16_t t = x^(x<<5);
	x = y;
	*r = y = (y^(y>>1))^(t^(t>>3));
}

void XorShiftClass::random(uint32_t *r)
{
	random((uint16_t*)r);
	random((uint16_t*)r + 1);
}
