#include "FrskySbus.h"

FrskySbus::FrskySbus(Stream &bus)
{
	bus_ = &bus;
	parser_state_ = 0;
}

FrskySbus::FrskySbus(Stream &bus, Stream &forwarding_bus)
{
	bus_ = &bus;
	forwarding_bus_ = &forwarding_bus;
	parser_state_ = 0;
	forward_bytes_ = true;
	forwarding_buffer_[0] = kSbusHeader;
	forwarding_buffer_[kForwardBufferSize - 1] = kSbusFooter;
}

void FrskySbus::begin()
{
	static_cast<HardwareSerial *>(bus_)->begin(kSbusBaud, SERIAL_8E2);
	while (bus_->available() > 0)
	{
		bus_->read();
	}
}

bool FrskySbus::checkForNewMessage()
{
	return parse();
}

void FrskySbus::storeNewMessage()
{
	// 16 channels of 11 bit data
	_channels[0] = (uint16_t)((payload_[0] | payload_[1] << 8) & 0x07FF);
	_channels[1] = (uint16_t)((payload_[1] >> 3 | payload_[2] << 5) & 0x07FF);
	_channels[2] = (uint16_t)((payload_[2] >> 6 | payload_[3] << 2 | payload_[4] << 10) & 0x07FF);
	_channels[3] = (uint16_t)((payload_[4] >> 1 | payload_[5] << 7) & 0x07FF);
	_channels[4] = (uint16_t)((payload_[5] >> 4 | payload_[6] << 4) & 0x07FF);
	_channels[5] = (uint16_t)((payload_[6] >> 7 | payload_[7] << 1 | payload_[8] << 9) & 0x07FF);
	_channels[6] = (uint16_t)((payload_[8] >> 2 | payload_[9] << 6) & 0x07FF);
	_channels[7] = (uint16_t)((payload_[9] >> 5 | payload_[10] << 3) & 0x07FF);
	_channels[8] = (uint16_t)((payload_[11] | payload_[12] << 8) & 0x07FF);
	_channels[9] = (uint16_t)((payload_[12] >> 3 | payload_[13] << 5) & 0x07FF);
	_channels[10] = (uint16_t)((payload_[13] >> 6 | payload_[14] << 2 | payload_[15] << 10) & 0x07FF);
	_channels[11] = (uint16_t)((payload_[15] >> 1 | payload_[16] << 7) & 0x07FF);
	_channels[12] = (uint16_t)((payload_[16] >> 4 | payload_[17] << 4) & 0x07FF);
	_channels[13] = (uint16_t)((payload_[17] >> 7 | payload_[18] << 1 | payload_[19] << 9) & 0x07FF);
	_channels[14] = (uint16_t)((payload_[19] >> 2 | payload_[20] << 6) & 0x07FF);
	_channels[15] = (uint16_t)((payload_[20] >> 5 | payload_[21] << 3) & 0x07FF);

	if (payload_[22] & kSbusLostFrame)
	{
		_lost_frame = true;
	}
	else
	{
		_lost_frame = false;
	}

	if (payload_[22] & kSbusFailSafe)
	{
		_failsafe_active = true;
	}
	else
	{
		_failsafe_active = false;
	}
}

/* parse the SBUS data */
bool FrskySbus::parse()
{
	// see if serial data is available
	while (bus_->available() > 0)
	{
		// _sbusTime = 0;
		uint8_t cur_byte = bus_->read();
		// find the header
		if (parser_state_ == 0)
		{
			if ((cur_byte == kSbusHeader) && (prev_byte_ == kSbusFooter))
			{
				parser_state_++;
			}
		}
		else
		{
			// strip off the data
			if ((parser_state_ - 1) < kPayloadSize)
			{
				payload_[parser_state_ - 1] = cur_byte;
				parser_state_++;
			}
			// check the end byte
			if ((parser_state_ - 1) == kPayloadSize)
			{
				if (cur_byte == kSbusFooter)
				{
					parser_state_ = 0;
					valid_frame_count_++;
					// If we are forwarding to another board, send the entire message at once
					if (forward_bytes_)
					{
						memcpy(&forwarding_buffer_[1], payload_, kPayloadSize);
						forwarding_bus_->write(forwarding_buffer_, kForwardBufferSize);
					}
					// We will only process the packet if we have received at least kValidFrameCountMin valid packets
					// before this one. This is so that we can't get use a rogue packet by accident just because channel values
					// happened to line up with the Header and Footer bytes.
					if (valid_frame_count_ > valid_frame_count_min_)
					{
						valid_frame_count_--; // Subtract one to keep from overflowing the uint8_t byte
						storeNewMessage();
						return true;
					}
					return false;
				}
				else
				{
					parser_state_ = 0;
					valid_frame_count_ = 0; // Invalid packet. Reset count.
					return false;
				}
			}
		}
		prev_byte_ = cur_byte;
	}
	// return false if a partial packet
	return false;
}

void FrskySbus::setValidFrameCountMin(uint8_t valid_frame_count_min)
{
	valid_frame_count_min_ = valid_frame_count_min;
}