/*
FrSky SBUS parser based off the SBUS.h library
from Brian R Tayload
brian.taylor@borderflight.com
https://github.com/bolderflight/SBUS


Created and maintained by COPPERPUNK
greg@copperpunk.com
*/

#ifndef FRSKYSBUS_H
#define FRSKYSBUS_H

#include "Arduino.h"

class FrskySbus
{
public:
	FrskySbus(Stream &bus);
	FrskySbus(Stream &bus, Stream &forwarding_bus);
	void begin();
	bool checkForNewMessage();
	void setValidFrameCountMin(uint8_t valid_frame_count_min);
	bool isFailsafeActive() { return _failsafe_active; }
	bool lostFrame() { return _lost_frame; }
	uint16_t getChannelValue(int channel) { return _channels[channel]; }

	static const int kNumChannelsMax = 16;

private:
	bool parse();
	void storeNewMessage();

	uint16_t _channels[kNumChannelsMax];
	bool _failsafe_active = false;
	bool _lost_frame = false;
	bool forward_bytes_ = false;
	const uint32_t kSbusBaud = 100000;
	const uint8_t kSbusHeader = 0x0F;
	const uint8_t kSbusFooter = 0x00;
	const uint8_t kSbusLostFrame = 0x04;
	const uint8_t kSbusFailSafe = 0x08;
	uint8_t parser_state_ = kSbusFooter;
	uint8_t prev_byte_ = kSbusFooter;
	static const uint8_t kPayloadSize = 24;
	uint8_t payload_[kPayloadSize];
	static const uint8_t kForwardBufferSize = 26;
	uint8_t forwarding_buffer_[kForwardBufferSize];

	Stream *bus_;
	Stream *forwarding_bus_;
	uint8_t valid_frame_count_min_ = 3;
	uint8_t valid_frame_count_ = 0;
};

#endif
