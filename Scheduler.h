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

#ifndef __SCHEDULER_H_
#define __SCHEDULER_H_

/*
 * Task Scheduler with 1ms resolution
 *
 * Allows critical sections lasting up to 16 ms.
 * Scheduler is using TIMER2.
 * Do not use analogWrite for pin PB 3 or PB 4, as it is also using TIMER2.
 *
 * Laurent Debacker
 */

#include <inttypes.h>

typedef bool (*SchedulerTaskFunction)(void);

class Scheduler;

// WARNING: do not release a SchedulerTask while it is being executed
class SchedulerTask {
private:
	bool IsActive, IsPreemptive;
	int32_t Period_ms; /* in milliseconds, up to 24.85 days */
	volatile bool Ready;
	volatile int32_t Countdown;
	SchedulerTaskFunction Function;
	SchedulerTask *NextTask;

public:
	friend class Scheduler;
	
	/* call from main context only */
	SchedulerTask(SchedulerTaskFunction function, int32_t period_ms, bool isPreemptive = false, bool isActive = false);
	/* call from main context only */
	~SchedulerTask();
	
	inline int32_t getPeriod() { return Period_ms; }
	void setPeriod(int32_t newValue_ms);
	
	inline bool isActive() { return IsActive; }
	inline bool isPreemptive() { return IsPreemptive; }
	
	inline void operator()() { Function(); }
	void reset(uint32_t delay = 0);
	void pause();
	void resume();
};

class Scheduler {
public:
	/* call from main context only */
	static void setup();
	/* call from main context only */
	static void start();
	/* call from main context only */
	static void stop();
	/* call from main context only */
	static bool runTasks();
	
	static uint16_t getTicks();
	
	/* The following function need to be public because they are
	   called by an ISR. They must not be called manually. */
	static void private_tick();
};

#endif
