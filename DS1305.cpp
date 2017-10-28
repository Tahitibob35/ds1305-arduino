/*
 * File			DS1305.cpp
 *
 * Synopsis		Support for Dallas Semiconductor DS1305 Real Time Clock, using SPI
 *
 * Author		Chris Bearman
 *
 * Version		1.0
 *
 * License		This software is released under the terms of the Mozilla Public License (MPL) version 2.0
 * 				Full details of licensing terms can be found in the "LICENSE" file, distributed with this code
 */
#include <Arduino.h>
#include "DS1305.h"
#include <SPI.h>

// Constructor with the option to set whether or not we use 24 hour based write (default)
// or not
DS1305::DS1305(bool writeHours24) : writeHours24(writeHours24)
{
}

// Default constructor, sets the 24 hour based write methodology as default
DS1305::DS1305() : writeHours24(true)
{
}

// Must call initialize prior to using any other method in this class (except constructor)

void DS1305::init(unsigned char ce) 
{
	SPI.begin( );

	// Initialize SPI Bus
	#if defined(__AVR__)
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);
	pinMode(SCK, OUTPUT);
	#endif
	
	pinMode(SS, OUTPUT);

	// Initialize the chip enable, LOW
	pinMode(ce, OUTPUT);
	digitalWrite(ce, LOW);

	// Record chip enable line
	this->ce = ce;

	// Read control register
	unsigned char cr = read(DS1305_CR);

	// Rewrite control register, forcing EOSC low to start the clock and
	// disabling write protect
	cr = cr & ~ (1 << DS1305_CR_EOSC);
	cr = cr & ~ (1 << DS1305_CR_WP);
	write(DS1305_CR, cr);
}

// Set the current time
// Time set uses hours (when writeHours24 = true), hours12/ampm (when writeHours24 = false)
void DS1305::setTime(const ds1305time *time)
{
	unsigned char buf[DS1305_SIZE_DATETIME];
	encodeTimePacket(buf, time);
	write(DS1305_DATETIME, buf, DS1305_SIZE_DATETIME);
}

// Retrieve current time
void DS1305::getTime(ds1305time *time)
{
	unsigned char buf[DS1305_SIZE_DATETIME];
	memset(time, 0, sizeof(ds1305time));
	read(DS1305_DATETIME, buf, DS1305_SIZE_DATETIME);
	decodeTimePacket(buf, time);
}

// Set an alarm
// alarm must be 0 or 1 else nothing is done
// Time set uses hours (when writeHours24 = true), hours12/ampm (when writeHours24 = false)
// Set any field (except ampm of course) to DS1305_ANY to indicate that alarm fires on any value in that field
void DS1305::setAlarm(int alarm, const ds1305alarm *time)
{
	if (alarm < 2) {
		unsigned char buf[DS1305_SIZE_ALARM];
		encodeAlarmPacket(buf, time);
		write(alarm == 0 ? DS1305_ALARM0 : DS1305_ALARM1, buf, DS1305_SIZE_ALARM);
	}
}

// Retrieve an alarm
// alarm must be 0 or 1 else nothing is done except resetting (zeroing) the response structure
// Be on the look out for DS1305_ANY in any alarm field (except ampm) to indicate whether the alarm fires on any value
// in that field
void DS1305::getAlarm(int alarm, ds1305alarm *time)
{
	memset(time, 0, sizeof(ds1305alarm));
	if (alarm < 2) {
		unsigned char buf[DS1305_SIZE_ALARM];
		read(alarm == 0 ? DS1305_ALARM0 : DS1305_ALARM1, buf, DS1305_SIZE_ALARM);
		decodeAlarmPacket(buf, time);
	}
}

// Retrieve the alarm state of an individual alarm where alarm = 0 or 1
// A true return means the alarm has triggered
bool DS1305::getAlarmState(unsigned int alarm)
{
	if (alarm > 1) return false;

	return ((read(DS1305_SR) & (1 << alarm)) ? true : false);
}

// Retrieve state of both alarms
// true values indicate that the alarm has triggered
void DS1305::getAlarmBothState(bool *state1, bool *state2)
{
	unsigned char sr = read(DS1305_SR);
	*state1 = (sr & 0x01) ? true : false;
	*state2 = (sr & 0x02) ? true : false;
}

// Clear the state of an individual alarm where alarm = 0 or 1
void DS1305::clearAlarmState(unsigned int alarm)
{
	if (alarm > 1) return;

	write(DS1305_SR, read(DS1305_SR) & ~ (1 << alarm));
}

// Clear state of both alarms
void DS1305::clearAlarmBothState()
{
	write(DS1305_SR, 0);
}

// Returns the enabled state of an individual alarm where alarm = 0 or 1
// A true return means the alarm is enabled
bool DS1305::getAlarmEnabled(unsigned int alarm)
{
	if (alarm > 1) return false;
	return((read(DS1305_CR) & (1 << alarm)) ? true : false);
}

// Returns the alarm enablement state of both alarms (true = enabled, false = disabled)
void DS1305::getAlarmBothEnabled(bool *enabled1, bool *enabled2)
{
	unsigned char cr = read(DS1305_CR);
	*enabled1 = (cr & 0x01) ? true : false;
	*enabled2 = (cr & 0x01) ? true : false;
}

// Enable an alarm where alarm = 0 or 1
void DS1305::enableAlarm(unsigned int alarm)
{
	if (alarm > 1) return;
	write(DS1305_CR, read(DS1305_CR) | (1 << alarm));
}

// Disable an alarm where alarm = 0 or 1
void DS1305::disableAlarm(unsigned int alarm)
{
	if (alarm > 1) return;
	write(DS1305_CR, read(DS1305_CR) & ~ (1 << alarm));
}

// Enable both alarms
void DS1305::enableBothAlarms()
{
	write(DS1305_CR, read(DS1305_CR) | 0x03);
}

// Disable both alarms
void DS1305::disableBothAlarms()
{
	write(DS1305_CR, read(DS1305_CR) & ~ 0x03);
}

// Sets the alarm interrupt joining state (joined = true joins both alarms to the first interrupt line)
void DS1305::setAlarmInterruptState(bool joined)
{
	unsigned char cr = read(DS1305_CR);
	if (joined) {
		cr &= ~ (1 << DS1305_CR_INTCN);
	} else {
		cr |= (1 << DS1305_CR_INTCN);
	}
	write(DS1305_CR, cr);
}

// Returns true if both alarms are joined on the first interrupt line
bool DS1305::areAlarmInterruptsJoined()
{
	return ((read(DS1305_CR) & (1 << DS1305_CR_INTCN)) ? false : true);
}

// Enable trickle charging
// Must provide number of diodes (1 or 2)
// and KOhm resistance (2, 4 or 8)
// Any other values will fail (false returned, no changes made)
bool DS1305::enableTrickleCharge(unsigned char numDiodes, unsigned char kRes)
{
	if (numDiodes < 1 || numDiodes > 2) return false;

	unsigned char byte = 0xA0 | (numDiodes << 2);
	switch(kRes) {
		case 2	:	byte |= 0x01; break;
		case 4	:	byte |= 0x02; break;
		case 8	:	byte |= 0x03; break;
		default	:	return false;
	}

	write(DS1305_TCR, &byte, 1);
}

// Disable trickle charging
void DS1305::disableTrickleCharge()
{
	unsigned char byte = 0;
	write(DS1305_TCR, &byte, 1);
}

// Retrieve trickle charging state
// Returns true (Trickle enabled), false (Trickle disabled)
// When enabled, numDiodes and kRes will be set to (1, 2) and (2, 4, 8) respectively
// When disabled numDiodes and kRes will be set to 0
bool DS1305::getTrickleChargeState(unsigned char *numDiodes, unsigned char *kRes)
{
	unsigned char byte;
	read(DS1305_TCR, &byte, 1);

	*numDiodes = 0;
	*kRes = 0;

	if ((byte & 0xF0) != 0xA0) return false;	// TCR disabled

	switch(byte & 0x03) {
		case 0x01	:	*kRes = 2; break;
		case 0x02	:	*kRes = 4; break;
		case 0x03	:	*kRes = 8; break;
		default		:	return false;
	}

	// TCR is enabled if we got this far
	*numDiodes = (byte & 0x0C) >> 2;

	return true;
}

// Write num elements of user memory, starting at addr
// Will fail and return false if write does not fall within the bounds of user memory space
// Will return true provided write is valid
bool DS1305::writeUser(unsigned char addr, const char *buf, int num)
{
	if (addr >= DS1305_USER_START && addr < DS1305_USER_END && (addr + num - 1) <= DS1305_USER_END) {
		write(addr, (const unsigned char *) buf, num);
		return true;
	} else {
		return false;
	}
}

// Read num elements of user memory, starting at addr
// Will fail and return false if write does not fall within the bounds of user memory space
// Will return true provided read is valid
bool DS1305::readUser(unsigned char addr, char *buf, int num)
{
	memset(buf, 0, num);
	if (addr >= DS1305_USER_START && addr < DS1305_USER_END && (addr + num - 1) <= DS1305_USER_END) {
		read(addr, (unsigned char *) buf, num);
		return true;
	} else {
		return false;
	}
}

// Returns true if DS1305 is write protected
bool DS1305::isWriteProtected()
{
	return ((read(DS1305_CR) & (1 << DS1305_CR_WP)) ? true : false);
}

// Set's the write protection on (true) or off for the DS1305
void DS1305::setWriteProtection(bool on)
{
	unsigned char cr = read(DS1305_CR);
	if (on) {
		cr |= (1 << DS1305_CR_WP);
	} else {
		cr &= ~ (1 << DS1305_CR_WP);
	}
	write(DS1305_CR, cr);
}

// Returns true if the DS1305 oscillator is running
bool DS1305::isRunning()
{
	return ((read(DS1305_CR) & (1 << DS1305_CR_EOSC)) ? false : true);
}

// Sets the oscillator run state for the DS1305
void DS1305::setRunState(bool running)
{
	unsigned char cr = read(DS1305_CR);
	if (running) {
		cr &= ~ (1 << DS1305_CR_EOSC);
	} else {
		cr |= (1 << DS1305_CR_EOSC);
	}
	write(DS1305_CR, cr);
}


// Reads len bytes from register in address into data
void DS1305::read(unsigned char address, unsigned char *data, int len)
{
	#if defined(__AVR__)
	SPISettings mySetting(20000000, MSBFIRST, SPI_MODE1);
	#elif defined(ESP8266)
	SPISettings mySetting(SPI_CLOCK_DIV16, MSBFIRST, SPI_MODE1);
	#endif

	SPI.beginTransaction(mySetting); 

	// Select the DS1305 by raising it's chip enable line
	digitalWrite(ce, HIGH);

	// Write the address to the SPI bus and wait for SPI write to copmlete
	SPI.transfer( address );

	// Write 0x00 to trigger reads
	for (int i = 0; i < len; i++) {
		data[i] = SPI.transfer( 0x00 );
	}

	// Deselect the DS1305 by lowering it's chip enable line
	digitalWrite(ce, LOW);

	// Restore SPCR
	SPI.endTransaction();
}


// Read a single byte register
unsigned char DS1305::read(unsigned char address)
{
	unsigned char buf;
	read(address, &buf, 1);
	return buf;
}

// Write SPI to register "address" with specified data, bursting for the given length
void DS1305::write(unsigned char address, const unsigned char *data, int len)
{
	/*// Take a backup of SPCR
	uint8_t spcr = SPCR;

	// Enable SPI as master, clock phase falling edge, CPOL idle low, MSB first, max rate, no interrupt
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPHA);*/

	#if defined(__AVR__)
	SPISettings mySetting(20000000, MSBFIRST, SPI_MODE1);
	#elif defined(ESP8266)
	SPISettings mySetting(SPI_CLOCK_DIV16, MSBFIRST, SPI_MODE1);
	#endif

	SPI.beginTransaction(mySetting); 

	// Select the DS1305 by raising it's chip enable line
	digitalWrite(ce, HIGH);

	// Write the address to the SPI bus (applying wirte offset automatically) and wait for SPI write to copmlete
	//SPDR = address | DS1305_WRITE_OFFSET;
	//waitSPI();
	SPI.transfer( address | DS1305_WRITE_OFFSET );

	// Loop through all provided data and write to SPI bus
	for (int i = 0; i < len; i++) {
		//SPDR = data[i];
		//waitSPI();
		SPI.transfer( data[i] );
	}

	// Deselect the DS1305 by lowering it's chip enable line
	digitalWrite(ce, LOW);

	// Restore SPCR
	//SPCR = spcr;
	SPI.endTransaction();
}

// Write a single byte register
void DS1305::write(unsigned char address, const unsigned char value)
{
	write(address, &value, 1);
}

// Encodes a time packet
// CJB - need to extend 12 hour based encoding
void DS1305::encodeTimePacket(unsigned char *buf, const ds1305time *time)
{
	buf[0] = encodeBCD7(time->seconds, 0x7F);
	buf[1] = encodeBCD7(time->minutes, 0x7F);
	buf[2] = encodeHourByte(time->hours, time->hours12, time->ampm);
	buf[3] = encodeBCD7(time->dow, 0x07);
	buf[4] = encodeBCD7(time->day, 0x3F);
	buf[5] = encodeBCD7(time->month, 0x3F);
	buf[6] = encodeBCD8(time->year);
}

// Encode an alarm packet
// CJB - need to extend 12 hour based encoding
void DS1305::encodeAlarmPacket(unsigned char *buf, const ds1305alarm *alarm)
{
	buf[0] = encodeBCD7(alarm->seconds, 0x7F);
	buf[1] = encodeBCD7(alarm->minutes, 0x7F);
	buf[2] = encodeHourByte(alarm->hours, alarm->hours12, alarm->ampm);
	buf[3] = encodeBCD7(alarm->dow, 0x07);
}

// Decode a time response from the DS1305
void DS1305::decodeTimePacket(const unsigned char *buf, ds1305time *time)
{
	time->seconds = decodeBCD7(buf[0], 0x7F);
	time->minutes = decodeBCD7(buf[1], 0x7F);
	decodeHourByte(buf[2], &(time->hours), &(time->hours12), &(time->ampm));
	time->dow = decodeBCD7(buf[3], 0x07);
	time->day = decodeBCD7(buf[4], 0x3F);
	time->month = decodeBCD7(buf[5], 0x3F);
	time->year = decodeBCD8(buf[6]);
}

// Decodes an alarm response from the DS1305
void DS1305::decodeAlarmPacket(const unsigned char *buf, ds1305alarm *alarm)
{
	alarm->seconds = decodeBCD7(buf[0], 0x7F);
	alarm->minutes = decodeBCD7(buf[1], 0x7F);
	decodeHourByte(buf[2], &(alarm->hours), &(alarm->hours12), &(alarm->ampm));
	alarm->dow = decodeBCD7(buf[3], 0x07);
}

// Encodes an hour byte using selected default format for write, honoring "ALL" (for alarms)
unsigned char DS1305::encodeHourByte(unsigned char hour24, unsigned char hour12, char ampm)
{
	if ((writeHours24 && ((hour24 & DS1305_ANY))) || (!writeHours24 && (hour12 & DS1305_ANY))) {
		return DS1305_ANY;
	} else {
		if (writeHours24) {
			// Writing 24 hour format times
			return encodeBCD7(hour24, 0x3F);
		} else {
			return (encodeBCD7(hour12, 0x1F) | 0x40 | (ampm == 'P' ? 0x20 : 0x00));
		}
	}
}

// Decodes an hour byte, honoring "ALL" (for alarms)
void DS1305::decodeHourByte(unsigned char hourByte, unsigned char *hour24, unsigned char *hour12, char *ampm)
{
	if (hourByte & DS1305_ANY) {
		*hour12 = DS1305_ANY;
		*hour24 = DS1305_ANY;
		*ampm = 0;
	} else {
		if (hourByte & 0x40) {
			// 12 hour mode
			*hour12 = decodeBCD7(hourByte, 0x1F);
			if (hourByte & 0x20) {
				// PM
				if (*hour12 == 12) {
					*hour24 = 12;
				} else {
					*hour24 = 12 + *hour12;
				}
				*ampm = 'P';
			} else {
				// AM
				*hour24 = (*hour12 == 12) ? 0 : *hour12;
				*ampm = 'A';
			}
		} else {
			// 24 hour mode
			*hour24 = decodeBCD7(hourByte, 0x3F);
			if (*hour24 == 0) {
				*hour12 = 12;
				*ampm = 'A';
			} else if (*hour24 == 12) {
				*hour12 = 12;
				*ampm = 'P';
			} else if (*hour24 < 12) {
				*ampm = 'A';
				*hour12 = *hour24;
			} else {
				*ampm = 'P';
				*hour12 = *hour24 - 12;
			}
		}
	}
}

// Encode value as BCD
unsigned char DS1305::encodeBCD7(unsigned char value)
{
	return ((value & DS1305_ANY) ? DS1305_ANY : (((value / 10) << 4) | (value % 10)));
}

// Encode value as BCD, apply bitmask (AND)
unsigned char DS1305::encodeBCD7(unsigned char value, unsigned char mask)
{
	return ((value & DS1305_ANY) ? DS1305_ANY : (((value / 10) << 4) | (value % 10)) & mask);
}

// Encode value as BCD, full 8 bits
unsigned char DS1305::encodeBCD8(unsigned char value)
{
	return (((value / 10) << 4) | (value % 10));
}

// Decode BCD
unsigned char DS1305::decodeBCD7(unsigned char value)
{
	return ((value & DS1305_ANY) ? DS1305_ANY : ((((value & 0xF0) >> 4) * 10) + (value & 0x0F)));
}

unsigned char DS1305::decodeBCD7(unsigned char value, unsigned char mask)
{

	return ((value & DS1305_ANY) ? DS1305_ANY : decodeBCD7(value & mask));
}

// Decode BCD, full 8 bits
unsigned char DS1305::decodeBCD8(unsigned char value)
{
	return ((((value & 0xF0) >> 4) * 10) + (value & 0x0F));
}

/*
// Wait for SPI transaction to finish
void DS1305::waitSPI()
{
	while(!(SPSR & (1<<SPIF))) { };
}
*/

