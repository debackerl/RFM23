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

#ifndef __PORT_H_
#define __PORT_H_

#include <avr/sfr_defs.h>
#include <avr/io.h>

// Pin is a macro defining a class.
// That way identification of which pin is being used
// is determined at compile time instead of run time,
// and no table with all pins available need to be stored.

#define Pin(LETTER, NUMBER) \
class \
{ \
public: \
	void high() { PORT ## LETTER |= _BV(NUMBER); } \
	void low() { PORT ## LETTER &= ~_BV(NUMBER); } \
	void write(bool bit) { if(bit) high(); else low(); } \
	bool state() { return PORT ## LETTER & _BV(NUMBER); } \
	bool read() { return (PIN ## LETTER >> (NUMBER)) & 1; } \
	\
	void output() { DDR ## LETTER |= _BV(NUMBER); } \
	void input() { DDR ## LETTER &= ~_BV(NUMBER); } \
}

#endif
