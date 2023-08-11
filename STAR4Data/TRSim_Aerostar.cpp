#include "TRSim_Aerostar.h"

// Written by Mark DeLoura and Arnie Martin for Future Engineers
// Last Updated: 3/3/2023

namespace TRSim_Aerostar {

/** @file TRSim_Aerostar.cpp
 * @brief
 * An Arduino library to read Aerostar flight telemetry data from the Future Engineers TechRise flight simulator.
 * Data is read over UART (RX pin on most CircuitPython boards).
 * Defines a Simulator class with properties to determine when a new packet is available.
 * Helper property functions are included to convert from telemetry format to usable values.
 * @details
@verbatim embed:rst
Aerostar data format:
    * Transmitted via serial connection: 115200 baud, 8N1
    * 79-byte data packet, big endian
    * 10Hz telemetry packets

Data fields:
    Separator (2):
        | uint16    *Sync Word*, 0x55 0xAA
    Header (10):
        | uint16    *CRC Checksum*, Not used by simulator
        | uint32[2] *Time*, Linux Epoch in seconds and microseconds (UTC currentmillis/1000)
    Payload (67):
        | int32[2]  *Latitude* and *Longitude*, in degrees
        | int32     *Altitude*, in meters
        | int32[3]  *Roll*, *Pitch*, *Yaw*, in degrees
        | int32[3]  *Acceleration* *X*, *Y*, *Z*, in meters/second^2
        | int32     *Pressure*, in kPa
        | int32     *Course*, relative to true north, in degrees
        | int32     *Speed*, in knots
        | int32[3]  *Velocity* *North*, *East*, *Down*, in meters/second
        | int32     *Declination*, magnetic declination for this position, in degrees
        | uint16    Number of *GPS Satellites Visible*
        | uint8     *GPS Fix Type*, current GPS Fix

Payload values:
    | See property helper functions below for conversion from fixed-point to floating-point value.

GPS Fix Type:
    | GPS_NO_FIX = 0
    | GPS_DEAD_RECKONING = 1
    | GPS_2D = 2
    | GPS_3D = 3
    | GPS_GNSS_DEAD_RECK = 4
    | GPS_TIME_ONLY = 5
@endverbatim
*/

/** 
 * @class Simulator TRSim_Aerostar.h
 * @brief Conducts Aerostar flight telemetry processing
 * @details
 * - Simulator::update(): Processes incoming flight telemetry data
 * - Simulator::isNewData(): True if new data has been received
 * - Simulator::isStreaming(): True if data is streaming
 * - Simulator::getPBF(): Value of the Pull-Before-Flight Header, True = removed
 * - Simulator::getGo(): Value of the GO LED, True = lit
 * - Simulator::setGo(val): Sets GO LED, HIGH = lit
 * - Simulator::getData(): Value of the new full data packet as a bytearray
 *
 * Use getData() to acquire all telemetry fields of the current packet. 
 * To acquire individual data fields from the current packet, use the helper functions detailed below.
 */
Simulator::Simulator() :
	_new_data(false), _stream_timeout(0), _mode(_MODE_FINDING_SYNCBYTE1),
	_saveStart(0), _saveEnd(0), _packetStart(0), _packetEnd(0),
	_packetBytesFound(0) {
}

Simulator::~Simulator() {

}

/** 
 * @brief Initializes the simulator
 * @param pbfPin A board pin number, connected to the PBF pin on PIB J5
 * @param goPin A board pin number, connected to the GO pin on PIB J5
 */
void Simulator::init(int pbfPin, int goPin) {
	// set the onboard UART pin
	Serial1.begin(_DEFAULT_BAUD_RATE);

	// Store incoming pin parameters
	_pbfPin = pbfPin;
	_goPin = goPin;

	// Setup the _pbfPin and _goPin
	pinMode(_pbfPin, INPUT);
	if (_pbfPin != PIN_PBF) {
		Serial.print("Pull Before Flight header active on ");
		Serial.println(_pbfPin);
	}
	pinMode(_goPin, OUTPUT);
	if (_goPin != PIN_GO) {
		Serial.print("Go LED active on ");
		Serial.println(_goPin);
	}
	// New behavior as of 3/3/2023 - GO LED is always lit
	setGo(HIGH);

	// Print out the starting state of the PBF header
	_pbfState = getPBF();
	if (_pbfState == HIGH) {
		Serial.println("Pull-before flight header removed");
	} else {
		Serial.println("Pull-before flight header inserted");
	}
	Serial.println();

}

/**
 * @brief Processes incoming flight telemetry data
 */
void Simulator::update() {
	// Check for removal of PBF header
	//   True = removed, so new value is True, stored state is False
	if ((getPBF() == HIGH) && (_pbfState == LOW)) {
		// flash the GO led a few times
		// go_pin will be set according to the state of the pbf_pin when TRsim.update() is called
		Serial.println("Pull Before Flight header has been removed");
		for (int i=0; i<6; i++) {
			setGo((getGo()==HIGH)?LOW:HIGH);
			delay(500);
		}
	}
	_pbfState = getPBF();

	// New behavior as of 3/3/2023 - GO LED is always lit
/*
	// Light Go LED if the PBF header is removed
	if (_pbfState == LOW) {
		// it is not, so turn on the go led
		setGo(LOW);
	} else {
		// it must be so turn off the go led
		setGo(HIGH);
	}
*/

	int bytesWaiting = Serial1.available();
	if (bytesWaiting > 0) {
		int firstPackets = 0;
		bool isWrappedLoad = false;
		bool isWrappedPacket = false;

		// Copy waiting bytes to ring buffer, from saveStart to saveEnd
		//   Wrap ring buffer if load goes past end of buffer
		_saveEnd = _saveStart + bytesWaiting;
		isWrappedLoad = (_saveEnd >= _BUFFER_SIZE);
		if (isWrappedLoad == true) {
			firstPackets = _BUFFER_SIZE - _saveStart;
			Serial1.readBytes(_buffer+_saveStart, firstPackets);
			Serial1.readBytes(_buffer, bytesWaiting - firstPackets);
		} else {
			Serial1.readBytes(_buffer+_saveStart, bytesWaiting);
		}

		// Process new bytes one by one
		//   Advance through modes based on data:
		//     _MODE_FINDING_SYNCBYTE1 - looking for 0x55 packet start
		//     _MODE_FINDING_SYNCBYTE2 - looking for 0xAA, second packet byte
		//     _MODE_FINDING_PAYLOAD   - looking for rest of 45 byte packet
		//   When full packet found, copy to currMV. Deal with wrap if necessary.
		unsigned int wrappedIndex;
		for (int index=_saveStart; index<_saveEnd; index++) {
			wrappedIndex = index & _BUFFER_SIZE_MASK;

			if (_mode == _MODE_FINDING_SYNCBYTE1) {
				if (_buffer[wrappedIndex] == _SYNCBYTE1) {
					_mode = _MODE_FINDING_SYNCBYTE2;
					_packetStart = wrappedIndex;
				}
			} else if (_mode == _MODE_FINDING_SYNCBYTE2) {
				if (_buffer[wrappedIndex] == _SYNCBYTE2) {
					_mode = _MODE_FINDING_PAYLOAD;
					_packetBytesFound = 2;
				} else { // if _SYNCBYTE2 is not immediately after _SYNCBYTE1, start over
					_mode = _MODE_FINDING_SYNCBYTE1;
				}
			} else if (_mode == _MODE_FINDING_PAYLOAD) {
				_packetBytesFound = _packetBytesFound + 1;
				if (_packetBytesFound == _PACKET_SIZE) {     // Full packet found
					_packetEnd = wrappedIndex;
					isWrappedPacket = ((_packetEnd+1) < _PACKET_SIZE);
					if (isWrappedPacket == true) {
						firstPackets = _BUFFER_SIZE - _packetStart;
						memcpy(_currPacketBuffer, _buffer+_packetStart, firstPackets);
						memcpy(_currPacketBuffer+firstPackets, _buffer, _packetEnd+1);
					} else {
						memcpy(_currPacketBuffer, _buffer+_packetStart, _PACKET_SIZE);
					}
					_new_data = true;
					_stream_timeout = millis() + 1500;
					_mode = _MODE_FINDING_SYNCBYTE1;
				}
			} else {
				//raise Exception('UART data state machine error')
				exit(-1);
			}
		}
		_saveStart = _saveEnd & _BUFFER_SIZE_MASK;
	}
}

/**
 * @brief Returns True if new data has been received
 */
bool Simulator::isNewData() {
	if (_new_data == true) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Returns True if data is streaming, and recieve timeout has not been exceeded
 */
bool Simulator::isStreaming() {
	if (millis() <= _stream_timeout) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Returns value of the Pull-Before-Flight Header, HIGH if Header removed
 */
int Simulator::getPBF() {
	if (_pbfPin != -1) {
		return digitalRead(_pbfPin);
	} else {
		return -1;
	}
}

/**
 * @brief Returns value of the GO LED, HIGH if LED is lit
 */
int Simulator::getGo() {
	if (_goPin != -1) {
		return digitalRead(_goPin);
	} else {
		return -1;
	}
}

/**
 * @brief Sets GO LED
 * @param val Value for GO LED (HIGH or LOW)
 */
void Simulator::setGo(int val) {
	if (_goPin != -1) {
		digitalWrite(_goPin, val);
	}
}

/**
 * @brief Returns the new full data packet as an array of bytes
 */
unsigned char* Simulator::getData() {
	if (_new_data == true) {
		// Flip new data bit
		_new_data = false;

		// Grab the velocityD and altiude properties from the new data packet
		float curr_vup = -getVelocityDown();
		float curr_alt = getAltitude();

		// This is a crude way to use the data to estimate the state of the balloon
		// State starts out as UNKNOWN
		if ((curr_alt < _THRESHOLD_GROUND_ALT) && (curr_vup < _THRESHOLD_LAUNCH_VUP)) {
			_flightStatus = STATUS_INITIALIZING;
		} else if ((_flightStatus == STATUS_INITIALIZING) && (curr_vup >= _THRESHOLD_LAUNCH_VUP)) {
			_flightStatus = STATUS_LAUNCHING;
		} else if ((_flightStatus == STATUS_LAUNCHING) && 
			(curr_alt > _THRESHOLD_FLOAT_ALT) &&
			(curr_vup <= _THRESHOLD_FLOAT_VUP)) {
			_flightStatus = STATUS_FLOATING;
		} else if ((_flightStatus == STATUS_FLOATING) && (curr_vup < _THRESHOLD_DESCEND_VUP)) {
			_flightStatus = STATUS_DESCENDING;
		}

		// Copy packet buffer
		memcpy(_returnBuffer, _currPacketBuffer, _PACKET_SIZE);
		return _returnBuffer;
	} else {
		return (unsigned char*)0; // Error
	}
}

/**
 * @brief Returns flight status, as an unsigned int
 */
unsigned int Simulator::getStatus() {
	if (isStreaming() == true) {
		return _flightStatus;
	} else {
		return STATUS_UNKNOWN; // TODO: ERROR
	}
}

/**
 * @brief Returns current Linux Epoch (UTC) time, seconds portion
 */
unsigned int Simulator::getTimeSecs() {
	if (isStreaming() == true) {
		return ((_currPacketBuffer[4]<<24)|
			(_currPacketBuffer[5]<<16)|
			(_currPacketBuffer[6]<<8)|
			(_currPacketBuffer[7]));
	} else {
		return 0; // Error
	}
}

/**
 * @brief Returns current Linux Epoch (UTC) time, microseconds portion
 */
unsigned int Simulator::getTimeUsecs() {
	if (isStreaming() == true) {
		return ((_currPacketBuffer[8]<<24)|
			(_currPacketBuffer[9]<<16)|
			(_currPacketBuffer[10]<<8)|
			(_currPacketBuffer[11]));
	} else {
		return 0; // Error
	}
}

/**
 * @brief Returns latitude, in degrees
 */
float Simulator::getLatitude() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[12]<<24) | 
					(_currPacketBuffer[13]<<16) | 
					(_currPacketBuffer[14]<<8) | 
					(_currPacketBuffer[15]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns longitude, in degrees
 */
float Simulator::getLongitude() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[16]<<24) | 
					(_currPacketBuffer[17]<<16) | 
					(_currPacketBuffer[18]<<8) | 
					(_currPacketBuffer[19]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns altitude, in meters
 */
float Simulator::getAltitude() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[20]<<24) | 
					(_currPacketBuffer[21]<<16) | 
					(_currPacketBuffer[22]<<8) | 
					(_currPacketBuffer[23]);
		retval = intval / 1024.0f;
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns roll, in degrees
 */
float Simulator::getRoll() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[24]<<24) | 
					(_currPacketBuffer[25]<<16) | 
					(_currPacketBuffer[26]<<8) | 
					(_currPacketBuffer[27]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns pitch, in degrees
 */
float Simulator::getPitch() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[28]<<24) | 
					(_currPacketBuffer[29]<<16) | 
					(_currPacketBuffer[30]<<8) | 
					(_currPacketBuffer[31]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns yaw, in degrees
 */
float Simulator::getYaw() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[32]<<24) | 
					(_currPacketBuffer[33]<<16) | 
					(_currPacketBuffer[34]<<8) | 
					(_currPacketBuffer[35]);
		retval = intval / (1024.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns acceleration in X direction, in meters/second^2
 */
float Simulator::getAccelerationX() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[36]<<24) | 
					(_currPacketBuffer[37]<<16) | 
					(_currPacketBuffer[38]<<8) | 
					(_currPacketBuffer[39]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns acceleration in Y direction, in meters/second^2
 */
float Simulator::getAccelerationY() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[40]<<24) | 
					(_currPacketBuffer[41]<<16) | 
					(_currPacketBuffer[42]<<8) | 
					(_currPacketBuffer[43]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns acceleration in Z direction, in meters/second^2
 */
float Simulator::getAccelerationZ() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[44]<<24) | 
					(_currPacketBuffer[45]<<16) | 
					(_currPacketBuffer[46]<<8) | 
					(_currPacketBuffer[47]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns atmospheric pressure, in kPa
 */
float Simulator::getPressure() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[48]<<24) | 
					(_currPacketBuffer[49]<<16) | 
					(_currPacketBuffer[50]<<8) | 
					(_currPacketBuffer[51]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns course, relative to true north, in degrees
 */
float Simulator::getCourse() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[52]<<24) | 
					(_currPacketBuffer[53]<<16) | 
					(_currPacketBuffer[54]<<8) | 
					(_currPacketBuffer[55]);
		retval = intval / (1024.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns speed, in knots
 */
float Simulator::getSpeed() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[56]<<24) | 
					(_currPacketBuffer[57]<<16) | 
					(_currPacketBuffer[58]<<8) | 
					(_currPacketBuffer[59]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns northward velocity, in meters/second
 */
float Simulator::getVelocityNorth() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[60]<<24) | 
					(_currPacketBuffer[61]<<16) | 
					(_currPacketBuffer[62]<<8) | 
					(_currPacketBuffer[63]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns eastward velocity, in meters/second
 */
float Simulator::getVelocityEast() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[64]<<24) | 
					(_currPacketBuffer[65]<<16) | 
					(_currPacketBuffer[66]<<8) | 
					(_currPacketBuffer[67]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns downward velocity, in meters/second
 */
float Simulator::getVelocityDown() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[68]<<24) | 
					(_currPacketBuffer[69]<<16) | 
					(_currPacketBuffer[70]<<8) | 
					(_currPacketBuffer[71]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns magnetic declination for this position, in degrees
 */
float Simulator::getDeclination() {
	if (isStreaming() == true) {
		int32_t intval;
		float retval;
		intval = (_currPacketBuffer[72]<<24) | 
					(_currPacketBuffer[73]<<16) | 
					(_currPacketBuffer[74]<<8) | 
					(_currPacketBuffer[75]);
		retval = intval / (8192.0f * 1024.0f);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns number of GPS Satellites visible
 */
int Simulator::getNumSats() {
	if (isStreaming() == true) {
		uint16_t intval = (_currPacketBuffer[76]<<8) | 
						(_currPacketBuffer[77]);
		return (int)intval;
	} else {
		return 0; // Error
	}
}

/**
 * @brief Returns GPS Fix Type
 */
int Simulator::getGPSFix() {
	if (isStreaming() == true) {
		return (int)_currPacketBuffer[78];
	} else {
		return 0; // Error
	}
}

/**
 * @brief Prints current data packet to USB serial connection 
 */
void Simulator::serialPrintCurrentPacket() {
	char outputString[255];

	serialPrintHexString(_currPacketBuffer);

	sprintf(outputString, "  time %d.%06d", getTimeSecs(), getTimeUsecs());
	Serial.println(outputString);

	Serial.print("  latitude ");
	Serial.println(getLatitude());
	Serial.print("  longitude ");
	Serial.println(getLongitude());
	Serial.print("  altitude ");
	Serial.println(getAltitude());
	Serial.print("  roll ");
	Serial.println(getRoll());
	Serial.print("  pitch ");
	Serial.println(getPitch());
	Serial.print("  yaw ");
	Serial.println(getYaw());
	Serial.print("  acceleration x ");
	Serial.println(getAccelerationX());
	Serial.print("  acceleration y ");
	Serial.println(getAccelerationY());
	Serial.print("  acceleration z ");
	Serial.println(getAccelerationZ());
	Serial.print("  pressure "); 
	Serial.println(getPressure());
	Serial.print("  course ");
	Serial.println(getCourse());
	Serial.print("  speed ");
	Serial.println(getSpeed());
	Serial.print("  velocity n ");
	Serial.println(getVelocityNorth());
	Serial.print("  velocity e ");
	Serial.println(getVelocityEast());
	Serial.print("  velocity d ");
	Serial.println(getVelocityDown());
	Serial.print("  declination ");
	Serial.println(getDeclination());
	Serial.print("  numSats ");
	Serial.println(getNumSats());
	Serial.print("  gpsFix ");
	Serial.println(getGPSFix());
}

/**
 * @brief Prints current data packet in hex to USB serial connection 
 */
void Simulator::serialPrintHexString(unsigned char* buff) {
	char outputString[255];
	int cursor = 0;

	for (int i=0; i<_PACKET_SIZE; i++) {
		sprintf(&(outputString[cursor]), "%02x", buff[i]);
		cursor += 2;
	}
	outputString[cursor] = 0;

	Serial.println(outputString);  
}


} // namespace
