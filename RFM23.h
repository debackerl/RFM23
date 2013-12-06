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

#ifndef __RFM23_H_
#define __RFM23_H_

#include <stdint.h>
#include <stddef.h>

namespace RFM23
{

enum {
	BROADCAST_ADDR = 0x3F
};

typedef struct packet {
	uint8_t source_node_id;
	uint8_t data_length;
	uint8_t data[60];
} rx_packet_t; // 62 bytes

typedef struct tx_packet {
	uint8_t dst_node_id;
	uint8_t data_length;
	uint8_t data[64];
	volatile uint8_t status;
	struct tx_packet *next;
} tx_packet_t; // 69 bytes

class Driver
{
public:
	/**
	 * Setup the RFM23 Driver.
	 * @param khz base frequency, 240000 - 930000 (depends on chip model).
	 * @param synchro 16-bits synchronization word used by the network.
	 * @param key 128-bits encryption key used by the network.
	 * @param node_id identifier of this node, 0 - 62.
	 */
	void setup(uint32_t khz, uint8_t *synchro, uint32_t *key, uint8_t node_id);

	/**
	 * Returns the identifier of this node.
	 */
	uint8_t getNodeId();
	
	/**
	 * Returns the temperature of the radio chip.
	 * @return 0 = 0.0°C, 1 = 0.5°C; min: -64°C, max: 63.5°C.
	 */
	int8_t readTemperature();
	
	/**
	 * Measures voltage on a GPIO.
	 * @param gpio_nr id of the pin, 0 - 3.
	 * @return 0 = 0mV, 159 = 1098mV (max value).
	 * @remark Do not feed more than 1.1V on the GPIO pin.
	 */
	uint8_t adc(uint8_t gpio_nr /* 0-3 */);
};

extern Driver Modem;

class PacketReader
{
public:
	PacketReader()
		{ mPacket = NULL; }
	bool next();
	void discard();
	void reset()
		{ mPos = 0; }
	bool overflow() const
		{ return mPos > 60; }
	size_t left() const
		{ return mPos > 60 ? 0 : (size_t)(mPacket->data_length - mPos); }
	bool operator !() const
		{ return mPacket == 0; }
	bool available() const
		{ return mPacket != 0; }
	bool isBroadcast() const
		{ return mPacket->source_node_id == BROADCAST_ADDR; }
	uint8_t sourceNodeId() const
		{ return mPacket->source_node_id; }
	void read(void *buf, size_t n);
	int8_t readCString(char *buf, size_t size);

private:
	rx_packet_t *mPacket;
	uint8_t mPos;
};

// WARNING: You cannot release a PacketWriter while its status is BUSY !
class PacketWriter
{
public:
	enum Status {
		UNSENT = 0,
		BUSY = 1,
		SUCCESS = 2,
		FAILED = 3
	};
	
	PacketWriter()
		{ mPkt.next = NULL; mPkt.data_length = 0; mPkt.status = UNSENT; }
	void reset()
		{ if(mPkt.status != BUSY) { mPkt.data_length = 0; mPkt.status = UNSENT; } }
	Status status()
		{ return (Status)mPkt.status; }
	bool overflow() const
		{ return mPkt.data_length > 60; }
	size_t left() const
		{ return mPkt.data_length > 60 ? 0 : (size_t)(60 - mPkt.data_length); }
	bool sendTo(uint8_t dst_node_id);
	bool broadcast();
	void write(const void *buf, size_t n);
	void writeCString(const char *buf);

private:
	tx_packet_t mPkt;
};

inline PacketReader &operator >>(PacketReader &rdr, bool &value)
	{ rdr.read(&value, sizeof(bool)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, bool const &value)
	{ wtr.write(&value, sizeof(bool)); return wtr; }

inline PacketReader &operator >>(PacketReader &rdr, char &value)
	{ rdr.read(&value, sizeof(char)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, char const &value)
	{ wtr.write(&value, sizeof(char)); return wtr; }

inline PacketReader &operator >>(PacketReader &rdr, float &value)
	{ rdr.read(&value, sizeof(float)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, float const &value)
	{ wtr.write(&value, sizeof(float)); return wtr; }
	
inline PacketReader &operator >>(PacketReader &rdr, uint8_t &value)
	{ rdr.read(&value, sizeof(uint8_t)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, uint8_t const &value)
	{ wtr.write(&value, sizeof(uint8_t)); return wtr; }

inline PacketReader &operator >>(PacketReader &rdr, uint16_t &value)
	{ rdr.read(&value, sizeof(uint16_t)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, uint16_t const &value)
	{ wtr.write(&value, sizeof(uint16_t)); return wtr; }

inline PacketReader &operator >>(PacketReader &rdr, uint32_t &value)
	{ rdr.read(&value, sizeof(uint32_t)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, uint32_t const &value)
	{ wtr.write(&value, sizeof(uint32_t)); return wtr; }

inline PacketReader &operator >>(PacketReader &rdr, int8_t &value)
	{ rdr.read(&value, sizeof(int8_t)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, int8_t const &value)
	{ wtr.write(&value, sizeof(int8_t)); return wtr; }

inline PacketReader &operator >>(PacketReader &rdr, int16_t &value)
	{ rdr.read(&value, sizeof(int16_t)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, int16_t const &value)
	{ wtr.write(&value, sizeof(int16_t)); return wtr; }

inline PacketReader &operator >>(PacketReader &rdr, int32_t &value)
	{ rdr.read(&value, sizeof(int32_t)); return rdr; }

inline PacketWriter &operator <<(PacketWriter &wtr, int32_t const &value)
	{ wtr.write(&value, sizeof(uint32_t)); return wtr; }

}

#endif
