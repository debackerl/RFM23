//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5, D 19)
//   RX (D 0) PD0  2|    |27  PC4 (AI 4, D 18)
//   TX (D 1) PD1  3|    |26  PC3 (AI 3, D 17)
//      (D 2) PD2  4|    |25  PC2 (AI 2, D 16)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1, D 15)
//      (D 4) PD4  6|    |23  PC0 (AI 0, D 14)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
// crystal    PB6  9|    |20  AVCC
// crystal    PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM ICSP
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM ICSP
//      (D 8) PB0 14|    |15  PB1 (D  9) PWM ICSP
//                  +----+
// Pin numbers on http://dl.dropbox.com/u/564060/jeelab-files/jlpcb-050.pdf are wrong!
// Compiler optimization: http://www.nongnu.org/avr-libc/user-manual/optimization.html#optim_code_reorder

#include <math.h>
#include <util/crc16.h>
#include <string.h>
#include "Scheduler.h"
#include "XorShift.h"
#include "xxtea.h"
#include "Driver.h"

#define ALLOW_SLEEP

#ifdef ALLOW_SLEEP
#include <avr/sleep.h>
#endif

RFM23::PacketWriter wtr;

bool broadcast()
{
	if(wtr.status() == RFM23::PacketWriter::BUSY) return true;
	wtr.reset();
	wtr.writeCString("0123456789abcdefghijklmnopqrst0123456789abcdefghijklmnopqrs");
	if(!wtr.broadcast())
		Serial.println("Could not broadcast.");
	return true;
}

SchedulerTask broadcastTask(broadcast, 1000, false, true);

#define RFM_IRQ  2
#define SPI_SS   10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK  13

void setup() {
	// pin mapping: http://arduino.cc/en/Hacking/PinMapping168
	
	uint8_t node_id = 2;
	uint8_t synchro[2] = { 0x8D, 0x23 };
	
	uint32_t seed = node_id;
	
        xxtea cipher;
	cipher.k[0] = 0xbf00f48a;
	cipher.k[1] = 0x14df479f;
	cipher.k[2] = 0x9ad7c1be;
	cipher.k[3] = 0xa87a9445;
	cipher.encrypt(&seed, 1);
	XorShift.seed(seed);
	
	RFM23::Modem.setup(868300, synchro, cipher.k, node_id);
	
	Scheduler::setup();
	
	Scheduler::start();
}

void loop() {
	bool busy = false;
	
#ifdef ALLOW_SLEEP
	// We must disable interrupts at this stage to avoid race conditions:
	// an interrupt may be fired after calling a poll function,
	// but before going in sleep mode. Without disabling interrupts,
	// the ISR would process the interrupt after the poll function,
	// but wouldn't wake up the microcontroller.
	cli();
#endif

	busy |= Scheduler::runTasks();
	
	RFM23::PacketReader rdr;
	if(rdr.next())
	{
		busy = true;
		char text[60];
		rdr.readCString(text, 60);
		Serial.print(rdr.sourceNodeId(), HEX);
		Serial.print(": ");
		Serial.println(text);
		rdr.discard();
	}

#ifdef ALLOW_SLEEP
	if(!busy) {
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
	}
	sei();
#endif
}
