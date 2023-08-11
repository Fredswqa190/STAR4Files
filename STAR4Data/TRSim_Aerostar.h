// Written by Mark DeLoura and Arnie Martin for Future Engineers
// Last Updated: 11/16/2022

#ifndef _TRSIM_AEROSTAR_H
#define _TRSIM_AEROSTAR_H

#include <Arduino.h>

namespace TRSim_Aerostar {

// Internal constants
#define _BUFFER_SIZE 1024
#define _BUFFER_SIZE_MASK 0x3ff
#define _RECEIVER_BUFFER_SIZE 512
#define _PACKET_SIZE 79
#define _DEFAULT_BAUD_RATE 115200
#define _DEFAULT_TIMEOUT 0.1

#define _SYNCBYTE1 0x55
#define _SYNCBYTE2 0xAA

#define _MODE_FINDING_SYNCBYTE1 1
#define _MODE_FINDING_SYNCBYTE2 2
#define _MODE_FINDING_PAYLOAD 3

// pin mappings
//   Note: this pin already defined in header files
// #define PIN_NEOPIXEL     40
#define PIN_PBF 2
#define PIN_GO  3

// Set up events
const unsigned int STATUS_NUM_EVENTS = 5;
const unsigned int STATUS_UNKNOWN = 0;
const unsigned int STATUS_INITIALIZING = 1;
const unsigned int STATUS_LAUNCHING = 2;
const unsigned int STATUS_FLOATING = 3;
const unsigned int STATUS_DESCENDING = 4;

// Set up threshold ascent rates and altitude levels to determine flight status
#define _THRESHOLD_LAUNCH_VUP 1
#define _THRESHOLD_FLOAT_VUP 1
#define _THRESHOLD_DESCEND_VUP -4
#define _THRESHOLD_GROUND_ALT 1000
#define _THRESHOLD_FLOAT_ALT 18500

// Baud rate for USB serial connection
#define SERIAL_BAUD 115200


// Simulator class
//
class Simulator {
	bool _new_data;
	unsigned int _stream_timeout;

	unsigned char _buffer[_BUFFER_SIZE];
	unsigned char _currPacketBuffer[_BUFFER_SIZE];
	unsigned char _returnBuffer[_PACKET_SIZE];

	unsigned int _flightStatus;
	
	unsigned int _mode;
	unsigned int _saveStart;
	unsigned int _saveEnd;
	unsigned int _packetStart;
	unsigned int _packetEnd;
	unsigned int _packetBytesFound;

	int _pbfPin;
	int _goPin;
	int _pbfState;

public:
	Simulator();
	~Simulator();
	void init(int pbfPin=PIN_PBF, int goPin=PIN_GO);
	void update();
	bool isNewData();
	bool isStreaming();
	int getPBF();
	int getGo();
	void setGo(int val);

	unsigned char* getData();
	unsigned int getStatus();
	unsigned int getTimeSecs();
	unsigned int getTimeUsecs();
	float getLatitude();
	float getLongitude();
	float getAltitude();
	float getRoll();
	float getPitch();
	float getYaw();
	float getAccelerationX();
	float getAccelerationY();
	float getAccelerationZ();
	float getPressure();
	float getCourse();
	float getSpeed();
	float getVelocityNorth();
	float getVelocityEast();
	float getVelocityDown();
	float getDeclination();
	int getNumSats();
	int getGPSFix();
	void serialPrintCurrentPacket();
	void serialPrintHexString(unsigned char* buff);
};

} // namespace

#endif

