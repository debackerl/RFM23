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

#include "Scheduler.h"
#include <util/atomic.h>
#include <stdint.h>

//#define SCHED_DEBUG

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)

#if (F_CPU <= 2000000)
#define PRESCALER 128
#elif (F_CPU <= 4000000)
#define PRESCALER 256
#else
#define PRESCALER 1024
#endif

#define NR_SLOTS (((uint32_t)255 * PRESCALER * 1000) / F_CPU) // 32 @ 8Mhz
#define TOP(i) ((i * F_CPU + (uint32_t)PRESCALER * 500) / ((uint32_t)PRESCALER * 1000))

static volatile uint16_t ticks = 0;
static volatile int32_t msecs = 0; /* msecs to wait before next full-scan */
static volatile bool ready = 0; /* at least one task is ready to run */
static volatile uint32_t count; /* msecs since last full-scan */
static uint8_t slot;
static SchedulerTask * volatile FirstTask = 0;

void Scheduler::setup() {
	/* Disable interrupt on Timer2 Output Compare Match A */
	TIMSK2 &= ~(1<<OCIE2A);
	/* Disable interrupt on Timer2 Output Compare Match B */
	TIMSK2 &= ~(1<<OCIE2B);
	/* Disable interrupt on Timer2 Overflow */
	TIMSK2 &= ~(1<<TOIE2);
	
	/* Clock source is internal (via prescaler) */
	ASSR &= ~((1<<AS2) | (1<<EXCLK));
	
	/* WGM21=1 + WGM20=0 = Mode2 (CTC, Clear Timer on Compare) */
	TCCR2A = (TCCR2A&~(1<<WGM20)) | (1<<WGM21);
	TCCR2B &= ~(1<<WGM22);
	
#if (F_CPU <= 2000000) // prescaler set to 128
	TCCR2A = (TCCR2A | (1<<CS22) | (1<<CS20)) & ~(1<<CS21);
#elif (F_CPU <= 4000000) // prescaler set to 256
	TCCR2A = (TCCR2A | (1<<CS22) | (1<<CS21)) & ~(1<<CS20);
#else // prescaler set to 1024
	TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);
#endif
}

void Scheduler::start() {
	/* Set compare value */
	slot = 1;
	OCR2A = TOP(NR_SLOTS) + 1;
	OCR2B = TOP(1);
	/* Reset counter value */
	TCNT2 = 0;
	/* Clear Interrupt flag on Timer2 Output Compare Match B */
	TIFR2 = (1<<OCF2B);
	/* Reset soft-counter */
	count = 0;
	/* Enable interrupt on Timer2 Output Compare Match A/B */
	TIMSK2 |= (1<<OCIE2A);
	TIMSK2 |= (1<<OCIE2B);
}

void Scheduler::stop() {
	/* Disable interrupt on Timer2 Output Compare Match A/B */
	TIMSK2 &= ~(1<<OCIE2A);
	TIMSK2 &= ~(1<<OCIE2B);
}

bool Scheduler::runTasks() {
	if(!ready) return false;
	ready = false; /* put it first in case that the ISR would be called later in this function */
	SchedulerTask * volatile *p_ptr = &FirstTask, *ptr;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		ptr = *p_ptr;
	}
#ifdef SCHED_DEBUG
	uint8_t k = 0;
#endif
	while(ptr != 0) {
		if(ptr->Ready && ptr->isActive()) {
#ifdef SCHED_DEBUG
			Serial.print("enter periodic task ("); Serial.print(ptr->Period_ms); Serial.println(")");
#endif
			if(ptr->Function()) {
				ptr->Ready = false;
			}
#ifdef SCHED_DEBUG
			Serial.print("exit periodic task ("); Serial.print(ptr->Period_ms); Serial.println(")");
#endif
		}
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			if(ptr->isActive()) {
				p_ptr = &ptr->NextTask;
			} else if(*p_ptr == ptr) { /* if a new task has been created since, the test will fail */
				*p_ptr = ptr->NextTask;
				ptr->NextTask = ptr;
			}
			ptr = *p_ptr;
		}
#ifdef SCHED_DEBUG
		if(++k > 10)
			Serial.println("runTasks loops");
#endif
	}
	return true;
}

uint16_t Scheduler::getTicks() {
	return ticks;
}

//bool yyy = true;
void Scheduler::private_tick() {
	/*yyy = !yyy;
	if(yyy) bitSet(PORTB, 2);
	else bitClear(PORTB, 2);*/
	
	uint8_t increment = 0;
	uint8_t cur, prev_OCR2B;
	for(;;) {
		++increment;
		if(++slot > NR_SLOTS) {
			prev_OCR2B = 0;
			slot = 1;
			OCR2B = TOP(1);
			cur = TCNT2 - TOP(NR_SLOTS);
		} else {
			prev_OCR2B = OCR2B;
			OCR2B = TOP(slot);
			cur = TCNT2;
		}
		if((prev_OCR2B <= cur && cur < OCR2B) || (TIFR2 & (1 << OCF2B)))
			break;
	}
	
	ticks += increment;
	
	if ((count += increment) >= msecs) {
		int32_t next_msecs = 0x1FFFFFFF;
		
		SchedulerTask *ptr = FirstTask;
#ifdef SCHED_DEBUG
		k = 0;
#endif
		while(ptr != 0) {
			if(ptr->isActive()) {
				if((ptr->Countdown -= count) <= 0) {
					ptr->Countdown = (ptr->Countdown % ptr->getPeriod()) + ptr->getPeriod();
					if(!ptr->IsPreemptive || !ptr->Function()) {
						ptr->Ready = true;
						ready = true;
					}
				}
				if(ptr->Countdown < next_msecs) next_msecs = ptr->Countdown;
			}
			ptr = ptr->NextTask;
#ifdef SCHED_DEBUG
			if(++k > 10)
				Serial.println("TIMER2_COMPB_vect loops (2)");
#endif
		}
		
		msecs = next_msecs > 0 ? next_msecs : 1;
		count = 0;
	}
}

ISR(TIMER2_COMPA_vect) { } // if you don't specify one, the chip will reset

ISR(TIMER2_COMPB_vect) {
	Scheduler::private_tick();
}

SchedulerTask::SchedulerTask(SchedulerTaskFunction function, int32_t period_ms, bool isPreemptive, bool isActive) {
	Function = function;
	Period_ms = period_ms;
	IsPreemptive = isPreemptive;
	IsActive = isActive;
	
	Ready = false;
	NextTask = this;
	if(isActive) reset();
}

SchedulerTask::~SchedulerTask() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		SchedulerTask * volatile *ptr = &FirstTask;
		while(*ptr != 0) {
			if(*ptr == this) {
				*ptr = (*ptr)->NextTask;
				break;
			}
			ptr = &(*ptr)->NextTask;
		}
	}
}

void SchedulerTask::reset(uint32_t delay) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		Countdown = Period_ms + delay;
		Ready = false;
		resume();
	}
}

void SchedulerTask::pause() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		Countdown -= count;
		IsActive = false;
		Ready = false;
	}
}

void SchedulerTask::resume() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if(NextTask == this) {
			NextTask = FirstTask;
			FirstTask = this;
		}
		Countdown += count;
		if(Countdown < msecs) {
			msecs = Countdown;
		}
		IsActive = true;
	}
}

void SchedulerTask::setPeriod(int32_t newValue_ms) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		Period_ms = newValue_ms;
		if(IsActive) {
			Countdown = ((Countdown - Period_ms) % newValue_ms) + newValue_ms;
			if(Countdown < msecs)
				msecs = Countdown;
		}
	}
}

#endif
