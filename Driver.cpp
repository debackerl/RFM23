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

// Features:
// * 57.6 kbps
// * Carrier Sense
// * Listen Before Talk
// * CRC
// * Up to 60-bytes long packets
// * Point-to-point and Broadcast
// * 63 addresses
// * Auto retransmit point-to-point packets
// * Replay detection
// * 1% Duty Cycle over last second
// * Encryption

// remember: #define _BV(bit) (1 << (bit))

// Packet composition:
// preamble: 7 nibbles
// synchro: 4 nibbles
// header: 4 nibbles
// packet length
// data
// CRC: 4 nibbles (over header, packet length, and data)

/* Errors stats (at 5 cm):
   Transmits: 503
   Ack Failures: 6
   Tx  Failures: 3
   1 retrans: 7
   2 retrans: 1
   => Packet loss: 1.79%
   => With 2 retransmissions (using 1.79%): about 1 in 174360 packets lost
 */

/*
 Probability of collision with one other peer:
 t_i in Uniform(0, 1)
 0 <= d <= 0.5, P[abs(t_0, t_1) < d | t_0 in (d,1-d)] = 2 * d
 0 <= d <= 0.5, P[abs(t_0, t_1) < d | t_0 in (0,d) U (1-d,1)] = t_0 + d
 0 <= d <= 0.5, P[abs(t_0, t_1) < d] = (2*d*(1-d) - 2*d*d) + 2*(d*d/2 + d*d) = d*d + 2*(1-d)*d

 Probability of collision with n other peers:
 t_i in Uniform(0, 1)
 0 <= d <= 0.5, P[exist i in (1,n), abs(t_0, t_i) < d | t_0 in (d,1-d)] = 1 - (1-2*d)^n
 0 <= d <= 0.5, P[exist i in (1,n), abs(t_0, t_i) < d | t_0 in (0,d) U (1-d,1)] = 1 - (1-t_0-d)^n
 0 <= d <= 0.5, P[exist i in (1,n), abs(t_0, t_i) < d] = ((1 - (1-2*d)^n)*(1-d) - (1 - (1-2*d)^n)*d) + 2*(d+((1-2*d)^(n+1))/(n+1) - ((1-d)^(n+1))/(n+1))
 
 Time to detect packet from other peer:
 * 764 us, time to transfer 11 nibbles at 57600 bps
 * 54 us, approximated lag (interrupt difference measured between two peers)
 */
 
/* Time to send one packet made of n bytes: (84+n*8)/57600 => 1.5 ms à 10.3 ms
 * Packet replies are sent immediatly, except to broadcast packets, where a random delay between 0 and 131 ms is used.
 * When no (n)ack is received, the next packet is sent between 64 ms + (131 ms (1st retry) à 524 ms (3rd retry))
 * 57600 bps <=> 7200 bytes per second, using a 1% Duty Cycle this is at most 72 bytes per second
 *
 * Freq Band    Power Magnetic Field    Duty Cycle      Channel Spacing
 * 868.000 –    <= 25 mW ERP (14dBm)    <= 1% or LBT    No spacing, for 1
 * 868.600 MHz                                          or more channels
 *
 * 868.700 –    <= 25 mW ERP (14dBm)    <= 0.1% or LBT  No spacing, for 1
 * 869.200 MHz                                          or more channels
 *
 * 869.400 –    <= 500 mW ERP (27dBm)   <= 10% or LBT   25 kHz (for 1
 * 869.650 MHz                                          or more channels)
 *
 * 869.700 –    <= 5 mW ERP (7dBm)      Up to 100%      No spacing, for 1
 * 870.000 MHz                                          or more channels
 */

/* PACKET TYPES:
 * A = hdr->src_node_id & 0xF0
 * B = hdr->DST_node_id & 0xE0
 * 
 *  A  B   Data Packet
 *  A !B   Counter synchronization (counters diff)
 * !A  B   Ack Packet
 * !A !B   Nack Packet (unexpected counter value)
 */

// RAM Usage: 303 bytes

#include "Driver.h"
#include "delay_x.h"
#include "Pin.h"
#include "Scheduler.h"
#include "XorShift.h"
#include "xxtea.h"
#include <alloca.h>
#include <math.h>
#include <string.h>
#include <avr/io.h>
#include <util/atomic.h>

//#define RFM23_DEBUG
//#define SHOW_RSSI

#define DUTY_CYCLE_MAX_BYTES 144
#define DUTY_CYCLE_WINDOW_LEN_MSEC 2000 // multiple of 10
#define MIN_ACK_TIMEOUT_MSEC 16
#define SILENCE_TIME_LAPSE_MSEC 16
#define COUNTER_SYNC_TIMEOUT_MSEC 500

#define PACKET_OVERHEAD_BYTES 16 // preamble: 5, sync: 2, header: 2, pk length: 1, counter: 4, crc: 2
#define REMOTE_PEERS_CACHE_LEN 13
#define RX_PACKETS_QUEUE_LEN 2

#define DATA_PACKET 0x03
#define COUNTER_SYNC 0x02
#define ACK 0x01
#define NACK 0x00

#define TX_STATE_NONE 0 // waiting ACK (txTentative > 0), or nothing in progress (txTentative = 0)
#define TX_STATE_PENDING 1 // the packet cannot be sent yet (send ASAP)
#define TX_STATE_SEND_DATA 2 // the radio is busy sending a data packet
#define TX_STATE_SEND_CTRL 3 // the radio is busy sending a control packet

namespace RFM23
{

Driver Modem;

typedef struct {
	uint8_t kind;
	uint8_t src_node_id;
	uint8_t dst_node_id;
	uint8_t length;
	uint32_t counter;
} header_t; // 8 bytes

typedef struct {
	uint8_t node_id;
	uint32_t counter_diff; // remote counter = local counter + diff, used for replay detection
	uint32_t last_counter; // used for duplicates detection
} remote_peer_info_t; // 9 bytes

static volatile struct {
	uint8_t txTentative:2; // > 0, the radio has a packet to send
	uint8_t txState:2;
	
	uint8_t rxPacketsWtr:1;
	uint8_t rxPacketsRdr:1;
	uint8_t packetReceived:1;
	uint8_t syncPending:1; // counter of suspicious packets received, check pending
} state; // 1 byte

static xxtea cipher; // 16 bytes
static uint8_t nodeId; // 1 byte

static rx_packet_t rxPackets[RX_PACKETS_QUEUE_LEN]; // 124 bytes
static uint8_t rxPacketsReading[(RX_PACKETS_QUEUE_LEN+7)/8]; // 1 byte
static uint32_t rxCounter; // 4 bytes

static uint8_t txHeader3; // 1 byte
static uint8_t txHeader2; // 1 byte
static uint8_t txPacketLength; // 1 byte
static uint32_t txCounter; // 4 bytes
static tx_packet_t *txFirstQueuedPkt, *txLastQueuedPkt; // 4 bytes

static header_t replyControlHeader; // 8 bytes
static uint32_t syncCounter; // 4 bytes

static uint8_t totalBytesSent; // 1 byte
static uint8_t bytesSent[10]; // 10 bytes
static uint8_t bytesSentIdx; // 1 byte

static uint32_t counter; // 4 bytes, incremented every 100 ms (lasts for 13 years and a half)

remote_peer_info_t remotePeers[REMOTE_PEERS_CACHE_LEN]; // 117 bytes

static bool sendTimerCallback();
static SchedulerTask sendTimer(sendTimerCallback, 0, true, false);

static bool replyTimerCallback();
static SchedulerTask replyTimer(replyTimerCallback, 0, true, false);

static bool dutyCycleCallback();
static SchedulerTask dutyCycleTimer(dutyCycleCallback, DUTY_CYCLE_WINDOW_LEN_MSEC / 10, true, false);

#define COUNTER_GT(a,b) ((uint32_t)((a) - (b)) - 1 < 2147483648ULL)
#define COUNTER_GE(a,b) ((uint32_t)((a) - (b)) < 2147483648ULL)

#if RX_PACKETS_QUEUE_LEN > 8
#define RX_PACKET_READING_TEST(idx) ((rxPacketsReading[(idx) >> 3] >> ((idx) & 7)) & 1)
#define RX_PACKET_READING_SET(idx) { rxPacketsReading[(idx) >> 3] |= 1 << ((idx) & 7); }
#define RX_PACKET_READING_CLEAR(idx) { rxPacketsReading[(idx) >> 3] &= ~(1 << ((idx) & 7)); }
#else
#define RX_PACKET_READING_TEST(idx) ((rxPacketsReading[0] >> (idx)) & 1)
#define RX_PACKET_READING_SET(idx) { rxPacketsReading[0] |= 1 << (idx); }
#define RX_PACKET_READING_CLEAR(idx) { rxPacketsReading[0] &= ~(1 << (idx)); }
#endif

Pin(D,2) RFM_IRQ;
Pin(B,2) SPI_SS;
Pin(B,3) SPI_MOSI;
Pin(B,4) SPI_MISO;
Pin(B,5) SPI_SCK;

#define INT1_MASK 0x06
#define INT2_MASK 0x00

static void rfm23_manual_interrupt(uint8_t x04);

static remote_peer_info_t *get_remote_peer_info(uint8_t node_id) {
	uint8_t idx = node_id % REMOTE_PEERS_CACHE_LEN;
	if(remotePeers[idx].node_id == node_id)
		return &remotePeers[idx];
	else
		return NULL;
}

static remote_peer_info_t *put_remote_peer_info(uint8_t node_id) {
	uint8_t idx = node_id % REMOTE_PEERS_CACHE_LEN;
	remotePeers[idx].node_id = node_id;
	return &remotePeers[idx];
}

static void spi_initialize () {
	SPI_SS.high();
	SPI_SS.output();
	SPI_MOSI.output();
	SPI_MISO.input();
	SPI_SCK.output();

	// use clk/2 as RFM23 can go up to 10Mhz

	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
	SPSR |= _BV(SPI2X);
}

static void rfm23_write(uint8_t address, uint8_t data) {
#ifdef RFM23_DEBUG
	Serial.print("> ");
	if(SREG & 0x80)
		Serial.println("rfm23_write used with GI !");
#endif
	
	SPI_SS.low();
	
	SPDR = 0x80 | address;
	while (!(SPSR & _BV(SPIF)))
		;
	SPDR = data;
	while (!(SPSR & _BV(SPIF)))
		;
	
	SPI_SS.high();

#ifdef RFM23_DEBUG
	Serial.print("rfm23_write ");
	Serial.print(address, HEX);
	Serial.print(" ");
	Serial.println(data, HEX);
#endif
}

static uint8_t rfm23_read(uint8_t address) {
#ifdef RFM23_DEBUG
	Serial.print("> ");
	if(SREG & 0x80)
		Serial.println("rfm23_write used with GI !");
#endif
	
	uint8_t ret;
	SPI_SS.low();
	
	SPDR = address;
	while (!(SPSR & _BV(SPIF)))
		;
	SPDR = 0;
	while (!(SPSR & _BV(SPIF)))
		;
	ret = SPDR;
	
	SPI_SS.high();
	
#ifdef RFM23_DEBUG
	Serial.print("rfm23_read ");
	Serial.print(address, HEX);
	Serial.print(" ");
	Serial.println(ret, HEX);
#endif
	
	return ret;
}

static void rfm23_burst(bool write, uint8_t address, uint8_t *data, uint8_t n) {
#ifdef RFM23_DEBUG2
	Serial.print("> ");
	uint8_t address0 = address, n0 = n;
	if(SREG & 0x80)
		Serial.println("rfm23_write used with GI !");
#endif
	
	if(n == 0) return;
	SPI_SS.low();
	
	if(write) address |= 0x80;
	SPDR = address;
	while (!(SPSR & _BV(SPIF)))
		;
	if(write)
	{
		do
		{
			SPDR = *(data++);
			while (!(SPSR & _BV(SPIF)))
				;
		} while(--n);
	}
	else
	{
		do
		{
			SPDR = 0;
			while (!(SPSR & _BV(SPIF)))
				;
			*(data++) = SPDR;
		} while(--n);
	}
	
	SPI_SS.high();
	
#ifdef RFM23_DEBUG2
	Serial.print("rfm23_burst ");
	Serial.print(write ? "write " : "read ");
	Serial.print(address0, HEX);
	data -= n0;
	do {
		Serial.print(" ");
		Serial.print(*(data++), HEX);
	} while(--n0);
	Serial.println();
#endif
}

static uint8_t rfm23_read_status2() {
	uint8_t x04 = rfm23_read(0x04);
	if(x04 & INT2_MASK)
		rfm23_manual_interrupt(x04);
	return x04;
}

static void rfm23_rx_mode() {
	// enter Idle Mode
	rfm23_write(0x07, 0x01);
	// reset RX FIFO
	rfm23_write(0x08, 0x22);
	rfm23_write(0x08, 0x20);
	// enter RX mode
	rfm23_write(0x07, 0x05);
}

static void rfm23_tx_mode() {
	// reset frequency offset set by AFC
	rfm23_write(0x73, 0x00);
	rfm23_write(0x74, 0x00);
	// enter TX mode, with PLL on (after TX mode, it will enter TUNE mode)
	rfm23_write(0x07, 0x09); // if disabled, SYNC packet is not sent, but bytes are sent in next DATA packet
}

static void arm_send_timer(uint16_t initPauseMsec) {
	// txTentative=3 => (initPauseMsec, initPauseMsec+15)
	// txTentative=2 => (initPauseMsec, initPauseMsec+127)
	// txTentative=1 => (initPauseMsec, initPauseMsec+1023)
	// txTentative=0 => (initPauseMsec)
	state.txState = TX_STATE_NONE;
	if(state.txTentative) {
		uint16_t delay_ms;
		XorShift.random(&delay_ms);
		initPauseMsec += (delay_ms & 0x1FFF) >> (state.txTentative*3);
		
		uint16_t max = (counter + 9 - txCounter) * (DUTY_CYCLE_WINDOW_LEN_MSEC / 10);
		if(max > 0x7FFF) state.txTentative = 0;
		else if(initPauseMsec > max) initPauseMsec = max;
	}
	sendTimer.reset(initPauseMsec);
}

static void rfm23_send_data_now() {
#ifdef RFM23_DEBUG
	Serial.println("Send data packet");
	Serial.print("rfm23_send_data_now: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	--state.txTentative;
	state.txState = TX_STATE_SEND_DATA;
	rfm23_write(0x3A, txHeader3);
	rfm23_write(0x3B, txHeader2);
	rfm23_write(0x3E, txPacketLength);
	rfm23_burst(true, 0x7F, txFirstQueuedPkt->data, txPacketLength);
	rfm23_tx_mode();
}

static void rfm23_try_send_data() {
#ifdef RFM23_DEBUG
	Serial.println("Try send data packet");
	Serial.print("rfm23_try_send_data: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	uint8_t status2 = 0;
	if(COUNTER_GT(txCounter, counter) || state.txState >= TX_STATE_SEND_DATA)
	{
		state.txState = TX_STATE_PENDING;
	}
	else if(totalBytesSent >= DUTY_CYCLE_MAX_BYTES || (status2 = rfm23_read_status2()) & 0x90)
	{ // if txCounter in the future, or too many bytes sent recently, or radio is receiving a packet
#ifdef RFM23_DEBUG
	Serial.print(txCounter, DEC); Serial.print(" ");Serial.print(counter, DEC); Serial.println();
	Serial.println("=> failed");
#endif
		arm_send_timer(SILENCE_TIME_LAPSE_MSEC);
	}
	else
	{
		rfm23_send_data_now();
		// sendTimer will be activated by rfm23_interrupt() when packet is sent
	}
	//Serial.println(status2 & 0x10, HEX);
	//Serial.println(rfm23_read(0x26), HEX);
}

static void rfm23_send_new_data_packet() {
#ifdef RFM23_DEBUG
	Serial.print("rfm23_send_new_data_packet: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	uint8_t length = txFirstQueuedPkt->data_length;
	txHeader3 = txFirstQueuedPkt->dst_node_id | (DATA_PACKET << 6); // transmit header 3 (0x3F = broadcast)
	txHeader2 = nodeId | ((length - 1) << 6); // transmit header 2
	uint8_t tx_len = 4 + 3 + length - ((length - 1) & 0x03);
	memset(txFirstQueuedPkt->data + 4 + txFirstQueuedPkt->data_length, 0, tx_len - (4 + txFirstQueuedPkt->data_length));
	txPacketLength = tx_len; // packet length
	txCounter = txCounter == counter ? counter + 1 : counter;
	uint32_t *blocks = reinterpret_cast<uint32_t*>(txFirstQueuedPkt->data);
	blocks[0] = txCounter ^ (txFirstQueuedPkt->dst_node_id | ((uint32_t)nodeId << 8));
	cipher.encrypt(blocks, tx_len >> 2);
	state.txTentative = txFirstQueuedPkt->dst_node_id == BROADCAST_ADDR ? 1 : 3;
	
	tx_len += PACKET_OVERHEAD_BYTES;
	totalBytesSent += tx_len;
	bytesSent[bytesSentIdx] += tx_len;
	
	rfm23_try_send_data();
}

static void rfm23_discard_tx_packet(bool success) {
	state.txState = TX_STATE_NONE;
	state.txTentative = 0;
	if(txFirstQueuedPkt) {
		txFirstQueuedPkt->status = success ? PacketWriter::SUCCESS : PacketWriter::FAILED;
		tx_packet_t *nxt = txFirstQueuedPkt->next;
		txFirstQueuedPkt->next = NULL;
		txFirstQueuedPkt = nxt;
		if(txFirstQueuedPkt) rfm23_send_new_data_packet();
		else txLastQueuedPkt = NULL;
	}
}

static void rfm23_queue_packet(tx_packet_t *pkt) {
	if(!pkt->data_length) {
		pkt->status = PacketWriter::SUCCESS;
		return;
	}
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if(!pkt->next && txLastQueuedPkt != pkt) {
			if(txLastQueuedPkt) txLastQueuedPkt->next = pkt;
			else txFirstQueuedPkt = pkt;
			txLastQueuedPkt = pkt;
			pkt->status = PacketWriter::BUSY;
			if(state.txTentative == 0) rfm23_send_new_data_packet();
		}
	}
}

static void rfm23_send_control(header_t *hdr) {
#ifdef RFM23_DEBUG
	Serial.print("Send control packet to ");
	Serial.println(hdr->dst_node_id, HEX);
	Serial.print("rfm23_send_control: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
	Serial.print("counter: "); Serial.println(hdr->counter, HEX);
#endif
	if(sendTimer.isActive() && state.txTentative < 3) return; // we are expecting a reply from some peer, avoid collision
	state.txState = TX_STATE_SEND_CTRL;
	rfm23_write(0x3A, hdr->dst_node_id | (hdr->kind << 6)); // transmit header 3 (0x3F = broadcast)
	rfm23_write(0x3B, hdr->src_node_id | 0xC0); // transmit header 2
	rfm23_write(0x3E, 8);
	uint32_t cipher_text[2] = { hdr->counter ^ (hdr->dst_node_id | ((uint32_t)hdr->src_node_id << 8)), 0 };
	cipher.encrypt(cipher_text, 2);
	rfm23_burst(true, 0x7F, reinterpret_cast<uint8_t*>(cipher_text), 8);
	rfm23_tx_mode();
}

static bool rfm23_recv(header_t *hdr, uint8_t *data) {
#ifdef RFM23_DEBUG
	Serial.print("rfm23_recv: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	hdr->dst_node_id = rfm23_read(0x47); // received header 3 (0x3F = broadcast)
	hdr->kind = hdr->dst_node_id >> 6;
	hdr->dst_node_id &= BROADCAST_ADDR;
	hdr->src_node_id = rfm23_read(0x48); // received header 2
	uint8_t tx_len = rfm23_read(0x4B); // received packet length
	uint8_t rem = 4 + 3 - (hdr->src_node_id >> 6);
	hdr->length = tx_len - rem;
	if(hdr->length >= 0xF9) return false; // 0xF9 == 0x00 - 0x07
	hdr->src_node_id &= BROADCAST_ADDR;
	uint32_t *cipher_text = (uint32_t*)alloca(tx_len);
	rfm23_burst(false, 0x7F, reinterpret_cast<uint8_t*>(cipher_text), tx_len);
	cipher.decrypt(cipher_text, tx_len >> 2);
	hdr->counter = cipher_text[0] ^ (hdr->dst_node_id | ((uint32_t)hdr->src_node_id << 8));
	if(data) {
		memcpy(data, cipher_text + 1, hdr->length);
	}
	return true;
}

static void process_control_packet(header_t *hdr)
{
#ifdef RFM23_DEBUG
	Serial.print("Process control packet from ");
	Serial.println(hdr->src_node_id, HEX);
	Serial.print("process_control_packet: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	if(hdr->dst_node_id != BROADCAST_ADDR)
	{
		switch(hdr->kind)
		{
			case COUNTER_SYNC:
			{
				rx_packet_t *p = &rxPackets[state.rxPacketsWtr];
				if(state.syncPending && hdr->src_node_id == p->source_node_id)
				{
					state.syncPending = 0;
					if(hdr->counter == rxCounter + syncCounter)
					{
						remote_peer_info_t *info = put_remote_peer_info(hdr->src_node_id);
						info->counter_diff = rxCounter - syncCounter;
						info->last_counter = rxCounter;
						if(COUNTER_GT(syncCounter + (COUNTER_SYNC_TIMEOUT_MSEC * 10) / DUTY_CYCLE_WINDOW_LEN_MSEC, counter))
						{
							++state.rxPacketsWtr;
							state.packetReceived = 1;
							
							hdr->kind = ACK;
							hdr->dst_node_id = hdr->src_node_id;
							hdr->src_node_id = nodeId;
							hdr->counter = info->last_counter;
							rfm23_send_control(hdr);
						}
						//else Serial.println("Counter sync failed: time out");
					}
					//else Serial.println("Counter sync failed: bad counter sum");
				}
				//else Serial.println("Counter sync failed: unexpected or bad src node id");
				break;
			}
			case ACK:
			{
				if(state.txTentative && hdr->src_node_id == (txHeader3 & BROADCAST_ADDR) && hdr->counter == txCounter)
				{
					sendTimer.pause(); // must be before rfm23_discard_tx_packet
					rfm23_discard_tx_packet(true);
				}
				break;
			}
			case NACK:
			{
				uint8_t orig_dst = txHeader3 & BROADCAST_ADDR;
				if(state.txTentative && (hdr->src_node_id == orig_dst || orig_dst == BROADCAST_ADDR))
				{
					remote_peer_info_t *info = get_remote_peer_info(hdr->src_node_id);
					if(!info) info = put_remote_peer_info(hdr->src_node_id);
					info->counter_diff = hdr->counter;
					
					header_t hdr2 = { COUNTER_SYNC, nodeId, hdr->src_node_id, 0, hdr->counter + txCounter + txCounter };
					sendTimer.pause();
					rfm23_send_control(&hdr2);
				}
				break;
			}
		}
	}
}

static void process_data_packet(header_t *hdr, rx_packet_t *p) {
#ifdef RFM23_DEBUG
	Serial.print("Process data packet from "); Serial.print(hdr->src_node_id, HEX); Serial.print(" to "); Serial.println(hdr->dst_node_id, HEX);
	Serial.print("process_data_packet: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	p->source_node_id = hdr->src_node_id;
	p->data_length = hdr->length;
	remote_peer_info_t *info = get_remote_peer_info(hdr->src_node_id);
	rxCounter = hdr->counter;
	uint32_t diff;
	if(!info || ((diff = hdr->counter - info->counter_diff - counter) > 10 && diff < 4294967296ULL - 10))
	{ // remote peer unknown or out of sync with local clock
		state.syncPending = 1;
		syncCounter = counter;
		bool broadcast = hdr->dst_node_id == BROADCAST_ADDR;
		hdr->kind = NACK;
		hdr->dst_node_id = hdr->src_node_id;
		hdr->src_node_id = nodeId;
		hdr->counter = counter - rxCounter;
		if(broadcast)
		{
			replyControlHeader = *hdr;
			uint16_t delay;
			XorShift.random(&delay);
			//syncCounter += (delay + 500 * (DUTY_CYCLE_WINDOW_LEN_MSEC / 10)) / (1000 * (DUTY_CYCLE_WINDOW_LEN_MSEC / 10));
			replyTimer.reset(delay & 0x3FF); // 0 - 1023ms
		}
		else
			rfm23_send_control(hdr);
	}
	else
	{ // packet is legit
		if(info->last_counter != rxCounter)
		{
			info->last_counter = rxCounter;
			++state.rxPacketsWtr;
			state.packetReceived = 1;
		}
		
		if(hdr->dst_node_id != BROADCAST_ADDR)
		{
			hdr->kind = ACK;
			hdr->dst_node_id = hdr->src_node_id;
			hdr->src_node_id = nodeId;
			rfm23_send_control(hdr);
		}
	}
}

static bool rfm23_is_rx_packet_available()
{
	return state.packetReceived;
}

static rx_packet_t &rfm23_get_rx_packet()
{
	rx_packet_t *pkt;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		uint8_t idx = state.rxPacketsRdr;
		pkt = &rxPackets[idx];
		if((state.rxPacketsRdr = idx+1) == state.rxPacketsWtr)
			state.packetReceived = 0;
		RX_PACKET_READING_SET(idx);
	}
	return *pkt;
}

static void rfm23_discard_packet(rx_packet_t *packet)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		uint8_t idx = packet - rxPackets;
		RX_PACKET_READING_CLEAR(idx);
	}
}

static bool sendTimerCallback() {
	sendTimer.pause();
#ifdef RFM23_DEBUG
	Serial.println("Delayed send");
#endif
	if(state.txTentative)
		rfm23_try_send_data();
	else
		rfm23_discard_tx_packet(false);
	return true;
}

static bool replyTimerCallback() {
	replyTimer.pause();
	if((rfm23_read_status2() & 0x90) == 0) { // avoid collisions
#ifdef RFM23_DEBUG
		Serial.println("Delayed reply");
#endif
		rfm23_send_control(&replyControlHeader);
	}
	// TODO: just drop it?
	return true;
}

static bool dutyCycleCallback() {
	++counter;
#ifdef RFM23_DEBUG
	Serial.print("dutyCycleCallback: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	
	if(++bytesSentIdx == 10) bytesSentIdx = 0;
	totalBytesSent -= bytesSent[bytesSentIdx];
	bytesSent[bytesSentIdx] = 0;
	
	if(state.txState == TX_STATE_PENDING)
		arm_send_timer(0);
	
	return true;
}

static void rfm23_manual_interrupt(uint8_t x04) {
#ifdef RFM23_DEBUG
	Serial.print("rfm23_manual_interrupt: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	uint8_t x03 = rfm23_read(0x03);
	uint8_t x07 = rfm23_read(0x07);
	
	if(!(x07 & 0x0C)) // not in RX or TX mode
	{
		if(state.txState >= TX_STATE_SEND_DATA) // TX mode (required when packet is sent, to reset 'sending')
		{
			rfm23_rx_mode();
			state.txState = TX_STATE_NONE;
			// state.txTentative == 3: a packet was pending, but we sent a (N)ACK
			// state.txTentative == 0: we just sent the last retry, last chance to wait an ACK
			// else: futher retries need to be scheduled
			if(txFirstQueuedPkt && !sendTimer.isActive())
				arm_send_timer(MIN_ACK_TIMEOUT_MSEC);
		}
		else
		{
			if(x03 & 0x02) // Valid Packet Received
			{
				if(state.syncPending)
				{ // counter sync timed out
					if(COUNTER_GE(counter, syncCounter + (COUNTER_SYNC_TIMEOUT_MSEC * 10) / DUTY_CYCLE_WINDOW_LEN_MSEC))
					{
						state.syncPending = 0;
					}
				}
				
				uint8_t pktRdr = state.rxPacketsRdr;
				if(state.syncPending == 0 && ((!state.packetReceived || state.rxPacketsWtr != pktRdr) && !RX_PACKET_READING_TEST(pktRdr)))
				{
					rx_packet_t *p = &rxPackets[state.rxPacketsWtr];
					header_t hdr;
					if(rfm23_recv(&hdr, p->data)) {
						if(hdr.kind != DATA_PACKET)
							process_control_packet(&hdr);
						else
							process_data_packet(&hdr, p);
					}
				}
				else
				{
					header_t hdr;
					if(rfm23_recv(&hdr, NULL)) {
						//Serial.print("recv: "); Serial.print(hdr.kind, DEC); Serial.print(" "); Serial.print(hdr.src_node_id, DEC); Serial.print(" "); Serial.println(hdr.dst_node_id, DEC); 2 2 2
						if(hdr.kind != DATA_PACKET)
							process_control_packet(&hdr);
					}
				}
			}
			
			if(state.txState <= TX_STATE_PENDING)
			{
				rfm23_rx_mode();
				if(state.txState == TX_STATE_PENDING)
				{
					// we don't want every peers waiting to send at the same time
					// we give some time lapse to let the peers of the previous communication to settle
					arm_send_timer(SILENCE_TIME_LAPSE_MSEC);
				}
			}
		}
	}
}

ISR(INT0_vect) {
#ifdef RFM23_DEBUG
	Serial.println("rfm23_interrupt");
#endif
	rfm23_manual_interrupt(rfm23_read(0x04));
}

static void rfm23_wait_end_of_transmissions() {
#ifdef RFM23_DEBUG
	Serial.print("rfm23_wait_end_of_transmissions: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
#endif
	uint8_t k = 0;
	for( ; k < 50 ; ++k) {
		uint8_t x04;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			x04 = rfm23_read_status2();
		}
		if(!(x04 & (0xC0 | INT2_MASK)))
			break;
		_delay_ms(1);
	}
	for( ; k < 50 ; ++k) {
		uint8_t x07;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			x07 = rfm23_read(0x07);
		}
		if(!(x07 & 0x08))
			break;
		_delay_ms(1);
	}
}

static void rfm23_initialize(uint32_t khz, uint8_t *synchro, uint32_t *key, uint8_t node_id) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		while(rfm23_read(0x01) == 0x00)
			_delay_ms(100);
		if(!(rfm23_read_status2() & 0x01))
		{
			rfm23_write(0x07, 0x80); // swres
			while(rfm23_read(0x07) & 0x80)
				_delay_ms(100);
			while(!(rfm23_read_status2() & 0x01))
				_delay_ms(100);
		}
		
		state.txTentative = 0;
		state.txState = TX_STATE_NONE;
		state.rxPacketsWtr = 0;
		state.rxPacketsRdr = 0;
		state.packetReceived = 0;
		state.syncPending = 0;
		
		memcpy(cipher.k, key, 16);
		
		XorShift.random(&counter);
		txCounter = counter - 1;
		txFirstQueuedPkt = NULL;
		txLastQueuedPkt = NULL;
		
		memset(rxPacketsReading, 0, sizeof(rxPacketsReading));
		
		for(size_t i = 0 ; i < REMOTE_PEERS_CACHE_LEN ; ++i)
			remotePeers[i].node_id = BROADCAST_ADDR;
		
		totalBytesSent = 0;
		memset(bytesSent, 0, sizeof(bytesSent));
		bytesSentIdx = 0;
		
		RFM_IRQ.input();
		RFM_IRQ.high(); // pull-up
		
		// preamble length seems to have no impact on range
		// increasing Fd decreases range (energy is more dispersed)
		
		uint8_t fb;
		uint16_t fc;
		if(khz < 480000) {
			fb = (khz - 240000) / 10000;
			fc = (4000 * khz) / 625 - 64000 * (uint32_t)fb - 1536000;
		} else {
			fb = (khz - 480000) / 20000;
			fc = (4000 * khz) / 1250 - 64000 * (uint32_t)fb - 1536000;
			fb |= 0x20;
		}
		
		rfm23_write(0x09, 0x7F); // c = 12.5p
		rfm23_write(0x6D, 0x0F); // set power max power
		//rfm23_write(0x34, 64); // 64 nibbles = 32byte preamble
		rfm23_write(0x34, 10); // 10 nibbles
		
		rfm23_write(0x05, INT1_MASK); // interrupt on ipksent, ipkvalid
		rfm23_write(0x06, INT2_MASK); // no other interrupt
		
		rfm23_write(0x27, 0x4A); // max background noise level: -78dBm
		
		// 57.6 kbps
		rfm23_write(0x1C, 0x03); // (dwn3_bypass = 0, ndec_exp = 0, filset = 3)
		rfm23_write(0x1D, 0x40); // (afcbd = 0, enafc = 1, afcgearh = 0, afcgearl = 0)
		rfm23_write(0x20, 0x45); // (rxosr[7:0] = 0x45)
		rfm23_write(0x21, 0x01); // (rxosr[10:8] = 0x00, stallctrl = 0, ncoff[19:16] = 1)
		rfm23_write(0x22, 0xD7); // (ncoff[15:8] = 0xD7)
		rfm23_write(0x23, 0xDC); // (ncoff[7:0] = 0xDC)
		rfm23_write(0x24, 0x07); // (crgain[10:8] = 0x07)
		rfm23_write(0x25, 0x6E); // (crgain[7:0] = 0x6E)
		rfm23_write(0x6E, 0x0E); // (txdr[15:8] = 0x0E)
		rfm23_write(0x6F, 0xBF); // (txdr[7:0] = 0xBF)
		rfm23_write(0x70, 0x0D); // (txdtrtscale = 0, enphpwdn = 0, manppol = 1, enmaninv = 1, enmanch = 0, enwhite = 1)
		rfm23_write(0x71, 0x23); // (trclk = 0, dtmod = 2, eninv = 0, fd[8] = 0, modtyp = 3)
		rfm23_write(0x72, 0x2E); // (fd[7:0] = 0x2E)
		rfm23_write(0x75, 0x40 | fb); // (sbse = 1, hbsel = ?, fb = ?)
		rfm23_write(0x76, (uint8_t)(fc >> 8)); // (fc[15:8] = ?)
		rfm23_write(0x77, (uint8_t)fc); // (fc[7:0] = ?)
		
		// 100 kbps
		/*rfm23_write(0x1C, 0x8F); // (dwn3_bypass = 1, ndec_exp = 0, filset = F)
		rfm23_write(0x1D, 0x40); // (afcbd = 0, enafc = 1, afcgearh = 0, afcgearl = 0)
		rfm23_write(0x20, 0x78); // (rxosr[7:0] = 0x78)
		rfm23_write(0x21, 0x01); // (rxosr[10:8] = 0x00, stallctrl = 0, ncoff[19:16] = 1)
		rfm23_write(0x22, 0x11); // (ncoff[15:8] = 0x11)
		rfm23_write(0x23, 0x11); // (ncoff[7:0] = 0x11)
		rfm23_write(0x24, 0x04); // (crgain[10:8] = 0x04)
		rfm23_write(0x25, 0x46); // (crgain[7:0] = 0x46)
		rfm23_write(0x6E, 0x19); // (txdr[15:8] = 0x19)
		rfm23_write(0x6F, 0x9A); // (txdr[7:0] = 0x9A)
		rfm23_write(0x70, 0x0D); // (txdtrtscale = 0, enphpwdn = 0, manppol = 1, enmaninv = 1, enmanch = 0, enwhite = 1)
		rfm23_write(0x71, 0x23); // (trclk = 0, dtmod = 2, eninv = 0, fd[8] = 0, modtyp = 3)
		rfm23_write(0x72, 0x50); // (fd[7:0] = 0x50)
		rfm23_write(0x75, 0x73); // (sbse = 1, hbsel = 1, fb = 19)
		rfm23_write(0x76, 0x67); // (fc[15:8] = 103)
		rfm23_write(0x77, 0xC0); // (fc[7:0] = 192)*/
		
		rfm23_write(0x32, 0x88); // enable broadcast for header 3 ??, enable header 3 check ??
		rfm23_write(0x3F, node_id); // specify header 3 to check
		nodeId = node_id;
		rfm23_write(0x43, 0x3F); // header enable 3
		rfm23_write(0x36, synchro[0]);
		rfm23_write(0x37, synchro[1]);
		rfm23_write(0x0B, 0x12); // GPIO0: TX State (output)
		rfm23_write(0x0C, 0x15); // GPIO1: RX State (output)
		rfm23_write(0x08, 0x20);
		
		rfm23_read(0x03); // reset interrupts
		rfm23_read(0x04); // reset interrupts
		rfm23_rx_mode(); // RX mode
	}
	
	//Serial.print("init: "); Serial.print(txCounter, DEC); Serial.print(" "); Serial.println(counter, DEC); Serial.println();
	
	dutyCycleTimer.reset();
	
	EICRA &= ~(_BV(ISC00) | _BV(ISC01)); // The low level of INT0 generates an interrupt request
	EIMSK |= _BV(INT0);
}

void Driver::setup(uint32_t khz, uint8_t *synchro, uint32_t *key, uint8_t node_id) {
	spi_initialize();
	rfm23_initialize(khz, synchro, key, node_id);
}

uint8_t Driver::getNodeId() {
	return nodeId;
}

int8_t Driver::readTemperature() { // -64°C => 63.5°C; 0 = 0.0°C, 1 = 0.5°C
	rfm23_wait_end_of_transmissions();
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		rfm23_write(0x07, 0x01);
		rfm23_write(0x0F, 0x80); // adcstart = 1
		do { _delay_ms(1); }
		while(!(rfm23_read(0x0F) & 0x80));
		uint8_t ret = (int8_t)(rfm23_read(0x11) - 140); // (0xB4=180 => 20°C (16°C)) (0xBB=187 => 23.8°C (19.4°C)) (0xBD=189 => 24.7°C (19.5°C))
		// 0xBF=191 => 25.3°C (19.5°C)
		//rfm23_write(0x55, 0x46);
		rfm23_rx_mode(); // RX mode
		return ret;
	}
}

uint8_t Driver::adc(uint8_t gpio_nr /* 0-3 */) { /* MAX INPUT: 1.1V */
	rfm23_wait_end_of_transmissions();
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		rfm23_write(0x0B + gpio_nr, 0x07); // GPIOn: ADC Analog Input
		rfm23_write(0x07, 0x01);
		rfm23_write(0x0F, 0x98 + (gpio_nr << 4)); // adcstart = 1, GPIO-x, Vref = VDD/3 => 0x16=22 => 0.15V; 0x40=64 => 0.45V; 0x61=97 => 0.66V; 0xA0=160 => 1.42V
		do { _delay_ms(1); }
		while(!(rfm23_read(0x0F) & 0x80)); // measure takes 350 µs
		rfm23_write(0x0B + gpio_nr, 0x00); // GPIOn: None
		uint8_t ret = rfm23_read(0x11); // 0 = 0mV, 159 = 1098mV (max value)
		//rfm23_write(0x55, 0x46);
		rfm23_rx_mode(); // RX mode
		return ret;
	}
}

bool PacketReader::next()
{
	if(!mPacket && rfm23_is_rx_packet_available())
	{
		mPacket = &rfm23_get_rx_packet();
		mPos = 0;
		return true;
	}
	else
	{
		return false;
	}
}

void PacketReader::discard() {
	if(mPacket) {
		rfm23_discard_packet(mPacket);
		mPacket = NULL;
	}
}

void PacketReader::read(void *buf, size_t n)
{
	if(left() < n) { mPos = 0xFF; return; }
	memcpy(buf, mPacket->data + mPos, n);
	mPos += n;
}

int8_t PacketReader::readCString(char *buf, size_t size)
{
	int8_t copied = 0;
	while(mPacket->data_length > mPos)
	{
		if(copied >= size)
			return -1;
		if((*(buf++) = mPacket->data[mPos++]) == 0)
			return copied - 1;
		++copied;
	}
	return -1;
}

bool PacketWriter::sendTo(uint8_t dst_node_id)
{
	if(mPkt.status == BUSY) return false;
	mPkt.dst_node_id = dst_node_id;
	rfm23_queue_packet(&mPkt);
	return true;
}

bool PacketWriter::broadcast()
{
	if(mPkt.status == BUSY) return false;
	mPkt.dst_node_id = BROADCAST_ADDR;
	rfm23_queue_packet(&mPkt);
	return true;
}

void PacketWriter::write(const void *buf, size_t n)
{
	if(left() < n) { mPkt.data_length = 0xFF; return; }
	memcpy(mPkt.data + 4 + mPkt.data_length, buf, n);
	mPkt.data_length += n;
}

void PacketWriter::writeCString(const char *buf)
{
	while(mPkt.data_length < 60)
	{
		if((mPkt.data[4 + mPkt.data_length++] = *(buf++)) == 0)
			return;
	}
	mPkt.data_length = 0xFF;
}

}
