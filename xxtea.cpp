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

#include "xxtea.h"

#define DELTA 0x9e3779b9
#define MX ((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (k[(uint8_t)((p&3)^e)] ^ z));

void xxtea::encrypt(uint32_t *buffer, int n) // n > 1
{
	uint32_t y, z, sum;
	uint8_t p, rounds, e;
	
	if(n) {
		rounds = 8+52/n;
		sum = 0;
		z = buffer[n-1];
		do {
			sum += DELTA;
			e = (sum >> 2) & 3;
			for (p=0; p<n-1; p++)
				y = buffer[p+1], z = buffer[p] += MX;
			y = buffer[0], z = buffer[n-1] += MX;
		} while (--rounds);
	}
}

void xxtea::decrypt(uint32_t *buffer, int n) // n > 1
{
	uint32_t y, z, sum;
	uint8_t p, rounds, e;
	
	if(n) {
		rounds = 8+52/n;
		sum = rounds*DELTA;
		y = buffer[0];
		do {
			e = (sum >> 2) & 3;
			for (p=n-1; p>0; p--)
				z = buffer[p-1], y = buffer[p] -= MX;
			z = buffer[n-1], y = buffer[0] -= MX;
		} while ((sum -= DELTA) != 0);
	}
}
