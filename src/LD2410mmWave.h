#pragma once

///////////////////////////////////////////////////////////////////////////////
// Library for LD2410C and LD2410B 24GHz mmWave Sensor
// Human Presence Radar Sensing Module with S3KM1110 chip
// Copyright (C) https://muman.ch + https://github.com/mumanchu, 2026.11.17
// If you re-use this software, please include the above copyright notice
// 
// For details see github
// https://github.com/mumanchu/LD2410mmWave
// 
// NOTE
// For LD2410S boards, use this library
// https://github.com/mumanchu/LD2410SmmWave


#include <Arduino.h>

// For error reporting to the Serial Monitor, define this before 
// #include "LD2410mmWave.h", as shown in the example sketch
#ifndef LD2410_LOGERROR 
#define LD2410_LOGERROR(s)
#endif


class LD2410mmWave
{
public:
	// Use 1-byte structure member packing to optimize memory usage
	#pragma pack(push)
	#pragma pack(1)

	// For readConfiguration()
	struct CONFIGDATA
	{
		byte maxGate;
		byte maxMotionGate;
		byte maxStaticGate;
		byte motionSensitivity[9];
		byte staticSensitivity[9];
		short unoccupiedDelay;
	};

	// Radar data 
	struct RADARDATA
	{
		// 0x01=engineering mode, 0x02=normal mode
		byte mode;
		// 0=no target, 1=moving target, 2=stationary target, 
		// 3=moving and stationary target, 4..6=background noise detection
		byte targetState;

		short motionTargetDistance;		// cm
		byte motionTargetEnergy;		// 0..100
		short staticTargetDistance;		// cm
		byte staticTargetEnergy;		// 0..100
		short detectionDistance;		// cm
	};

	// Additional radar data when in engineering mode
	struct ENGINEERINGDATA
	{
		byte maxMotionGate;				// fixed at 8?
		byte maxStaticGate;				// fixed at 8?
		byte motionGateEnergyValues[9];
		byte staticGateEnergyValues[9];
		byte photosensitivityLevel;		// this seems to do nothing
		byte outPinState;
	};

	#pragma pack(pop)	// back to default structure member packing

private:
	Stream* stream;

	// for receive() state machine
	int rxState;
	short rxMsgLength;
	short rxMsgIndex;
	int rxFrameIndex;
	byte rxMsg[64];		// 64 = receive buffer size
	bool cmdResponse;
	bool gotCmdResponse;

	// frame start/end bytes
	static const byte cmdFrameStart[4];
	static const byte cmdFrameEnd[4];
	static const byte datFrameStart[4];
	static const byte datFrameEnd[4];

public:
	// this may be useful if the protocol changes, set by enableConfigurationMode()
	// at present it is always 1
	short protocolVersion;

	bool begin(Stream* stream);
	bool enableConfigurationMode();
	bool disableConfigurationMode();

	//>>> The sensor must be on Configuration Mode to use the following methods
	bool startEngineeringMode();
	bool endEngineeringMode();
	bool restoreFactorySettings();
	bool restartModule();
	bool getFirmwareVersion(char* fwVersion /*16 chars*/);
	bool getMacAddress(byte* macAddress /*6 bytes*/);
	bool getConfiguration(CONFIGDATA* configData);
	bool turnOnBluetooth(bool on);

	// Configuration methods are optional
	#ifdef LD2410_ENABLE_CONFIG

	bool setBaudRate(byte baudRateIndex);
	bool setGateSensitivity(short gate, byte motionSensitivity, byte staticSensitivity);
	bool setMaxGate(byte maxMotionGate, byte maxStaticGate, short unoccupiedDelay);
	bool setDistanceResolution(byte distanceResolutionIndex);
	bool getDistanceResolution(byte* distanceResolutionIndex);
	bool setAuxControlConfiguration(byte lightSensingControl, 
		byte lightSensitivityThreshold, byte outPinLevel);
	bool getAuxControlConfiguration(byte* lightSensingControl,
		byte* lightSensitivityThreshold, byte* outPinLevel);
	bool startBackgroundNoiseDetection(short duration);
	bool queryBackgroundNoiseDetection(byte* status);
	bool enterBluetoothPassword(char* password /*6 chars + NUL*/);
	bool setBluetoothPassword(char* password /*6 chars + NUL*/);
	#ifdef DEBUG
	void dumpRadarData(RADARDATA* radarData, Stream& stream);
	void dumpEngineeringData(ENGINEERINGDATA* radarData, Stream& stream);
	#endif

	#endif // #ifdef LD2410_ENABLE_CONFIG
	//<<<

	bool receive(RADARDATA* radarData = NULL, ENGINEERINGDATA* engineeringData = NULL);

protected:
	// 'short' is always 16 bits
	short getShort(uint offset) { return *(short*)(rxMsg + offset); }

	bool sendCommandAndGetResponse(const byte* command, int length);
	bool processRadarDataMessage(RADARDATA* radarData,
		ENGINEERINGDATA* engineeringData);
};

// Message frame start and end sequences
const byte LD2410mmWave::cmdFrameStart[4] = { 0xFD, 0xFC, 0xFB, 0xFA };
const byte LD2410mmWave::cmdFrameEnd[4]   = { 0x04, 0x03, 0x02, 0x01 };
const byte LD2410mmWave::datFrameStart[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
const byte LD2410mmWave::datFrameEnd[4]   = { 0xF8, 0xF7, 0xF6, 0xF5 };


// Call this from setup() after Serialx.begin(baudRate)
// sensor is in Configuration Mode on return
bool LD2410mmWave::begin(Stream* stream)
{
	this->stream = stream;
	rxState = 0;

	// set 'configuration mode', returns false if not responding
	return enableConfigurationMode();
}

// 2.2.1 Enable configuration mode command
// The sensor is usually in Radar Data Output Mode, where it continuously 
// reports its status. 
// This command ends radar data output mode and puts the sensor into 
// Command Mode so it is ready to receive commands.
bool LD2410mmWave::enableConfigurationMode()
{
	static const byte cmd[4] = { 0xFF, 0x00, 0x01, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 4)) {
		protocolVersion = 0;
		return false;
	}

	// this may be useful if the protocol changes, at present it is always 1
	protocolVersion = getShort(4);

	// check the receive buffer size is as expected
	short bufferSize = getShort(6);
	if (bufferSize != sizeof(rxMsg)) {
		LD2410_LOGERROR("Invalid RX buffer size");
	}
	return true;
}

// 2.2.2 End configuration mode command
// back to "radar data output" mode
bool LD2410mmWave::disableConfigurationMode()
{
	static const byte cmd[2] = { 0xFE, 0x00 };
	return sendCommandAndGetResponse(cmd, 2);
}

// 2.2.5 Enable engineering mode command
// Switches to "engineering mode" so additional data is output
// by "radar data output protocol".
bool LD2410mmWave::startEngineeringMode()
{
	static const byte cmd[2] = { 0x62, 0x00 };
	return sendCommandAndGetResponse(cmd, 2);
}

// 2.2.6 End engineering mode command
// (the manual calls this 'Close project mode')
bool LD2410mmWave::endEngineeringMode()
{
	static const byte cmd[2] = { 0x63, 0x00 };
	return sendCommandAndGetResponse(cmd, 2);
}

// 2.2.10 Restore factory settings
// New values take effect after restartModule() or power up
/*
Max Motion Gate       8
Max Static Gate       8
Unoccupied Delay      5
Baud Rate             256000
Distance Resolution   0.75m

       Sensitivity
Gate   Motion  Static
0      50      100 (fixed)
1      50      100 (fixed)
2      40      40
3      30      40
4      20      30
5      15      30
6      15      20
7      15      20
8      15      20
Note: Higher values = lower sensitivity, 100 = off!
*/
bool LD2410mmWave::restoreFactorySettings()
{
	static const byte cmd[] = { 0xA2, 0x00 };
	return sendCommandAndGetResponse(cmd, 2);
}

// 2.2.11 Restart module
// takes 2000 milliseconds before sensor responds again
// after restart, sensor is in NOT in configuration mode
bool LD2410mmWave::restartModule()
{
	static const byte cmd[2] = { 0xA3, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 2))
		return false;
	delay(2000);
	return true;
}

// 2.2.8 Read firmware version command
// char fwVersion[16] : for example, "2.44.24073110", 240731 is probably the date
bool LD2410mmWave::getFirmwareVersion(char* fwVersion /*16 chars*/)
{
	*fwVersion = '\0';

	static const byte cmd[2] = { 0xA0, 0x00 };
	if (!sendCommandAndGetResponse(cmd, sizeof(cmd)))
		return false;
	if (rxMsgLength != 12) {
		LD2410_LOGERROR("invalid rxMsgLength");
		return false;
	}

	// version is binary, convert it to text
	// 0  1  2  3  4  5  6  7  8  9  10 11
	// A0 01 00 00 00 01 44 02 10 31 07 24 
	//      | ack |tt tt|bb aa|ff ee dd cc|
	// "2.44.24073110"
	sprintf(fwVersion, "%x.%02x.%02x%02x%02x%02x",
		rxMsg[7], rxMsg[6], rxMsg[11], rxMsg[10], rxMsg[9], rxMsg[8]);

	// 'tt tt' (fw type) is not needed, always 00 01?
	//short fwType = getShort(4);

	return true;
}

// 2.2.13 Get the unique MAC Address
// byte macAddress[6]   // must be 6 bytes
bool LD2410mmWave::getMacAddress(byte* macAddress /*6 bytes*/)
{
	memset(macAddress, 0, 6);

	static const byte cmd[4] = { 0xA5, 0x00, 0x01, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 4))
		return false;
	if (rxMsgLength != 10) {
		LD2410_LOGERROR("invalid rxMsgLength");
		return false;
	}
	memcpy(macAddress, rxMsg + 4, 6);
	return true;
}

// 2.2.4 Read parameters command
bool LD2410mmWave::getConfiguration(CONFIGDATA* configData)
{
	memset(configData, 0, sizeof(CONFIGDATA));

	static const byte cmd[2] = { 0x61, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 2))
		return false;
	if (rxMsgLength != 28) {
		// note: this happens if not in cofiguration mode, cmd returns ack
		LD2410_LOGERROR("invalid rxMsgLength");
		return false;
	}
	if (rxMsg[4] != 0xAA) {
		LD2410_LOGERROR("missing AA");
		return false;
	}
	configData->maxGate = rxMsg[5];
	configData->maxMotionGate = rxMsg[6];
	configData->maxStaticGate = rxMsg[7];
	memcpy(configData->motionSensitivity, rxMsg + 8, 9);
	memcpy(configData->staticSensitivity, rxMsg + 17, 9);
	configData->unoccupiedDelay = getShort(26);

	return true;
}

#ifdef LD2410_ENABLE_CONFIG

// 2.2.9 Set serial port baud rate
// baudRateIndex : 1=9600,   2=19200,  3=38400,  4=57600, 
//                 5=115200, 6=230400, 7=256000, 8=460800
// The new baud rate takes effect after restartModule() or the next power up
// Lower the baud rate if you want to use SoftwareSerial
// the default is 256000
bool LD2410mmWave::setBaudRate(byte baudRateIndex)
{
	if (baudRateIndex < 1 && baudRateIndex > 8) {
		LD2410_LOGERROR("invalid baudrate");
		return false;
	}
	byte cmd[4] = { 0xA1, 0x00, 0x07, 0x00 };
	cmd[2] = baudRateIndex;
	return sendCommandAndGetResponse(cmd, 4);
}

// 2.2.3 Maximum number of distance gates and unoccupied delay parameters 
// Sets the number of 'distance gates' for motion and static detection
// maxMotionGate : 2..8
// maxStsticGate : 2..8
// unoccupiedDelay : 0..65535 seconds
bool LD2410mmWave::setMaxGate(byte maxMotionGate, byte maxStaticGate,
	short unoccupiedDelay)
{
	if (maxMotionGate < 2 || maxMotionGate > 8 ||
		maxStaticGate < 2 || maxStaticGate > 8) {
		LD2410_LOGERROR("invalid gate");
		return false;
	}
	byte cmd[20] = {
		0x60, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00, 0x00, 0x00
	//  |--param--| |--value--| |-unused-|
	};
	cmd[4] = maxMotionGate;
	cmd[10] = maxStaticGate;
	cmd[16] = (byte)unoccupiedDelay;
	cmd[17] = (byte)(unoccupiedDelay >> 8);

	return sendCommandAndGetResponse(cmd, 20);
}

// 2.2.7 Distance gate sensitivity configuration command
// Set the motion and static sensitivity for each gate
// gate number : 0..8, or 0xFFFF for all gates
// sensitivity : 0..100  0   = lowest energy value, highest sensistivity
//                       100 = highest energy value, lowest sensitivity
// NOTE: staticSensitivity for gates 0 and 1 has no effect (100 is used)
bool LD2410mmWave::setGateSensitivity(short gate,
	byte motionSensitivity, byte staticSensitivity)
{
	if (gate > 8 && gate != 0xFFFF) {
		LD2410_LOGERROR("invalid gate");
		return false;
	}
	if (motionSensitivity > 100 || staticSensitivity > 100) {
		LD2410_LOGERROR("invalid sensitivity");
		return false;
	}
	byte cmd[20] = {
		0x64, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// gate number
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00,	// motion sensitivity
		0x02, 0x00, 0x00, 0x00, 0x00, 0x00	// static sensitivity
	//  |--param--| |--value--| |-unused-|
	};
	cmd[4] = (byte)gate;
	cmd[5] = (byte)(gate >> 8);
	cmd[10] = (byte)motionSensitivity;
	cmd[16] = (byte)staticSensitivity;

	return sendCommandAndGetResponse(cmd, 20);
}

// 2.2.16 Distance resolution setting
// the distance between each distance gate
// distanceResolutionIndex : 0=0.75m (default), 1=0.2m 
bool LD2410mmWave::setDistanceResolution(byte distanceResolutionIndex)
{
	if (distanceResolutionIndex > 1) {
		LD2410_LOGERROR("invalid parameter");
		return false;
	}
	byte cmd[4] = { 0xAA, 0x00, 0x00, 0x00 };
	cmd[2] = distanceResolutionIndex;
	return sendCommandAndGetResponse(cmd, 4);
}

// 2.2.17 Query distance resolution setting
// distanceResolutionIndex : 0=0.75m, 1=0.2m 
bool LD2410mmWave::getDistanceResolution(byte* distanceResolutionIndex)
{
	*distanceResolutionIndex = 0;
	static const byte cmd[2] = { 0xAB, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 2))
		return false;
	*distanceResolutionIndex = rxMsg[4];
	return true;
}

// 2.2.18 Auxiliary control function settings
// lightSensingControl       : 0=turn off light sensor control (default)
//                             1=turn on when less than threshold
//                             2=turn on when greater than threshold
// lightSensitivityThreshold : 0..255 (0xFF), default is 128 (0x80)
// outPinLevel               : 0=OUT pin active high (default)
//                             1=OUT pin active low
bool LD2410mmWave::setAuxControlConfiguration(byte lightSensingControl,
	byte lightSensitivityThreshold, byte outPinLevel)
{
	byte cmd[6] = { 0xAD, 0x00, 0x00, 0x00, 0x00, 0x00 };
	cmd[2] = lightSensingControl;
	cmd[3] = lightSensitivityThreshold;
	cmd[4] = outPinLevel;
	return sendCommandAndGetResponse(cmd, 6);
}

// 2.2.19 Query auxiliary control function configuration
bool LD2410mmWave::getAuxControlConfiguration(byte* lightSensingControl,
	byte* lightSensitivityThreshold, byte* outPinLevel)
{
	static const byte cmd[2] = { 0xAE, 0x00 };
	bool ok = sendCommandAndGetResponse(cmd, 2);
	// values are 0 if comms fails
	*lightSensingControl = rxMsg[4];
	*lightSensitivityThreshold = rxMsg[5];
	*outPinLevel = rxMsg[6];
	return ok;
}

// 2.2.20 Start performing background noise detection and automatic 
// sensitivity configuration (starts after 10 seconds)
// duration = test duration in seconds, 0..65535
bool LD2410mmWave::startBackgroundNoiseDetection(short duration)
{
	byte cmd[4] = { 0x0B, 0x00, 0x00, 0x00 };
	cmd[2] = (byte)duration;
	cmd[3] = (byte)(duration >> 8);
	return sendCommandAndGetResponse(cmd, 4);
}

// 2.2.21 Query execution status of background noise detection
// status : 0=not in progress, 1=in progress, 2=completed
bool LD2410mmWave::queryBackgroundNoiseDetection(byte* status)
{
	*status = 0;
	static const byte cmd[2] = { 0x1B, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 2))
		return false;
	*status = rxMsg[4];
	return true;
}

// 2.2.14 Obtaining bluetooth permissions
// password : 6 chars + NUL
bool LD2410mmWave::enterBluetoothPassword(char* password /*6 chars + NUL*/)
{
	byte cmd[8] = { 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	for (int i = 0; i < 6; ++i) {
		if (password[i] == '\0')
			break;
		cmd[i + 2] = password[i];
	}
	return sendCommandAndGetResponse(cmd, 8);
}

// 2.2.15 Setting Bluetooth password
// password = 6 chars + NUL
bool LD2410mmWave::setBluetoothPassword(char* password /*6 chars + NUL*/)
{
	if (strlen(password) != 6) {
		LD2410_LOGERROR("password must be six ASCII characters");
		return false;
	}
	byte cmd[8] = { 0xA9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	memcpy(cmd + 2, password, 6);
	return sendCommandAndGetResponse(cmd, 8);
}

#endif	// #ifdef LD2410_ENABLE_CONFIG


// 2.2.12 Bluetooth settings
// turn Bluetooth off/on, the default is ON
bool LD2410mmWave::turnOnBluetooth(bool on)
{
	byte cmd[] = { 0xA4, 0x00, 0x00, 0x00 };
	if (on)
		cmd[2] = 0x01;
	return sendCommandAndGetResponse(cmd, 4);
}

// Simple state machine to receive frames
// used for both radar data and command ack response frames
// call this from loop()
// returns true when a new message is available
// only the message body is stored in rxMsg[]
bool LD2410mmWave::receive(RADARDATA* radarData,
	ENGINEERINGDATA* engineeringData)
{
	while (stream->available()) {
		byte b = stream->read();

		switch (rxState) {

		// look for data frame or ack frame header
		case 0:
			gotCmdResponse = false;

			// start of command response (ack/nak)
			if (b == cmdFrameStart[0]) {
				cmdResponse = true;
			startReceive:
				rxMsgLength = 0;
				rxFrameIndex = 1;
				rxState = 1;
				break;
			}
			// start of data message
			if (b == datFrameStart[0]) {
				cmdResponse = false;
				goto startReceive;
			}
			break;

		// receive start frame
		case 1:
			if (b == (cmdResponse ? cmdFrameStart : datFrameStart)[rxFrameIndex]) {
				if (++rxFrameIndex == 4)
					rxState = 2;
				break;
			}
			rxState = 0;
			break;

		// receive message length LS byte
		case 2:
			rxMsgLength = b;
			rxState = 3;
			break;

		// receive message length MS byte
		case 3:
			rxMsgLength += (short)b << 8;
			rxMsgIndex = 0;
			rxState = 4;
			break;

		// receive message body and store in rxMsg
		case 4:
			if (rxMsgIndex >= sizeof(rxMsg)) {
				LD2410_LOGERROR("rxbuf overflow");
				rxState = 0;
				break;
			}
			rxMsg[rxMsgIndex++] = b;

			// message received, next is the end frame
			if (rxMsgIndex == rxMsgLength) {
				rxFrameIndex = 0;
				rxState = 5;
			}
			break;

		// receive end frame
		case 5:
			if (b == (cmdResponse ? cmdFrameEnd : datFrameEnd)[rxFrameIndex]) {
				if (++rxFrameIndex == 4) {
					// full message received
					rxState = 0;
					if (cmdResponse)
						gotCmdResponse = true;
					else
						processRadarDataMessage(radarData, engineeringData);
					return true;
				}
				break;
			}
			rxState = 0;
			break;

		default:
			LD2410_LOGERROR("invalid rxState");
			rxState = 0;
			break;
		}
	}
	return false;
}

// Radar data frame received "Radar data output message"
// message is in rxData[] buffer, with length in rxMsgLength
bool LD2410mmWave::processRadarDataMessage(RADARDATA* radarData,
	ENGINEERINGDATA* engineeringData)
{
	// all 0s if it fails
	if (radarData)
		memset(radarData, 0, sizeof(RADARDATA));
	if (engineeringData)
		memset(engineeringData, 0, sizeof(ENGINEERINGDATA));

	if (rxMsgLength != 0x0d && rxMsgLength != 0x23) {
		LD2410_LOGERROR("invalid rxMsgLength");
		return false;
	}
	if (rxMsg[1] != 0xAA) {
		LD2410_LOGERROR("invalid head");
		return false;
	}
	if (rxMsg[rxMsgLength - 2] != 0x55) {
		LD2410_LOGERROR("invalid tail");
		return false;
	}
	if (rxMsg[rxMsgLength - 1] != 0x00) {
		LD2410_LOGERROR("invalid check");
		return false;
	}

	// create local copies first, 
	// this saves many pointer derefernces, and data is not returned if it fails
	RADARDATA radarData1;
	ENGINEERINGDATA engineeringData1;

	// 0x01=engineering mode, 0x02=normal mode
	radarData1.mode = rxMsg[0];
	if (radarData1.mode != 0x01 && radarData1.mode != 0x02) {
		LD2410_LOGERROR("invalid mode");
		return false;
	}
	radarData1.targetState = rxMsg[2];
	radarData1.motionTargetDistance = getShort(3);
	radarData1.motionTargetEnergy = rxMsg[5];
	radarData1.staticTargetDistance = getShort(6);
	radarData1.staticTargetEnergy = rxMsg[8];
	radarData1.detectionDistance = getShort(9);
	if (radarData)
		memcpy(radarData, &radarData1, sizeof(RADARDATA));

	// Normal mode (2)
	if (radarData1.mode == 0x02)
		return true;

	// Engineering mode (1) returns additional data

	// 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34
	// 01 AA 03 1E 00 3C 00 00 39 00 00 08 08 3C 22 05 03 03 04 03 06 05 00 00 39 10 13 06 06 08 04 60 01 55 00
	// mm hh ss cm cm ev cm cm ev dd dd nm ns 00 11 22 33 44 55 66 77 88 00 11 22 33 44 55 66 77 88 ll pp  
	//|      normal mode data          | max |    motion gate energy    |    static gate energy    |     | tail|
	engineeringData1.maxMotionGate = rxMsg[11];
	engineeringData1.maxStaticGate = rxMsg[12];

	// check the message length is as expected
	if (rxMsgLength != (13 + engineeringData1.maxMotionGate + engineeringData1.maxStaticGate + 6)) {
		LD2410_LOGERROR("invalid rxMsgLength");
		return false;
	}
	memcpy(engineeringData1.motionGateEnergyValues, rxMsg + 13, 9);
	memcpy(engineeringData1.staticGateEnergyValues, rxMsg + 22, 9);
	engineeringData1.photosensitivityLevel = rxMsg[31];		// ll
	engineeringData1.outPinState = rxMsg[32];				// pp
	if (engineeringData)
		memcpy(engineeringData, &engineeringData1, sizeof(ENGINEERINGDATA));

	return true;
}

// Returns true if everything's ok, false if there's an error
bool LD2410mmWave::sendCommandAndGetResponse(const byte* command, int length)
{
	// return 0s if it fails
	memset(rxMsg, 0x00, sizeof(rxMsg));

	// clear the receive buffer
	while (stream->available())
		stream->read();

	// send command
	stream->write(cmdFrameStart, 4);
	stream->write((byte)length);
	stream->write((byte)(length >> 8));
	stream->write(command, length);
	stream->write(cmdFrameEnd, 4);

	// wait for ack response, or timeout
	gotCmdResponse = false;
	rxState = 0;
	unsigned long startms = millis();
	do {
		receive();
		unsigned long elapsed = millis() - startms;
		if (elapsed > 1000) {
			LD2410_LOGERROR("ack timeout");
			return false;
		}
	} while (!gotCmdResponse);

	// response validation
	if (rxMsgLength < 4) {
		LD2410_LOGERROR("invalid rxMsgLength");
		return false;
	}
	// check for command ack bytes: |<command-byte> 0x01|0x00 0x00|
	if (rxMsg[0] != command[0] || rxMsg[1] != 0x01) {
		LD2410_LOGERROR("invalid response");
		return false;
	}
	// 0x00 0x00 = ack
	// 0x01 0x00 = nak
	if (getShort(2) != 0) {
		LD2410_LOGERROR("nak response");
		return false;
	}
	return true;
}

// Display the Radar and Engineering data
#ifdef DEBUG
void LD2410mmWave::dumpRadarData(RADARDATA* radarData, Stream& stream)
{
	char buf[64];

	const char* state;
	switch (radarData->targetState) {
	case 0:
		state = "no target";
		break;
	case 1:
		state = "moving target";
		break;
	case 2:
		state = "static target";
		break;
	case 3:
		state = "moving and static target";
		break;
	case 4:
		state = "background noise detection in progress";
		break;
	case 5:
		state = "background noise detection successful";
		break;
	case 6:
		state = "background noise detection failed";
		break;
	default:
		state = "** unknown state **";
		break;
	}

	// some plaforms do not have stream.printf(), so use sprintf()
	stream.print("Current State          = ");
	stream.print(state);
	stream.println("                              \n\r");   // overwrites previous text
	sprintf(buf, "movingTarget           = %-3u %-3u\n\r",
		radarData->motionTargetDistance, radarData->motionTargetEnergy);
	stream.print(buf);
	sprintf(buf, "staticTarget           = %-3u %-3u\n\r",
		radarData->staticTargetDistance, radarData->staticTargetEnergy);
	stream.print(buf);
	sprintf(buf, "detectionDistance      = %-3u\n\r", radarData->detectionDistance);
	stream.print(buf);
	stream.println();
	stream.flush();
}

void LD2410mmWave::dumpEngineeringData(ENGINEERINGDATA* data, Stream& stream)
{
	char buf[64];

	// some plaforms do not have stream.printf(), so use sprintf()
	sprintf(buf, "maxMotionGate          = %-3u\n\r", data->maxMotionGate);
	stream.print(buf);
	sprintf(buf, "maxStaticGate          = %-3u\n\r", data->maxStaticGate);
	stream.print(buf);
	stream.print("motionGateEnergyValues = ");
	for (int i = 0; i < 9; ++i) {
		sprintf(buf, "%03u ", data->motionGateEnergyValues[i]);
		stream.print(buf);
	}
	stream.println();
	stream.print("staticGateEnergyValues = ");
	for (int i = 0; i < 9; ++i) {
		sprintf(buf, "%03u ", data->staticGateEnergyValues[i]);
		stream.print(buf);
	}
	stream.println();
	sprintf(buf, "photosensitivityLevel  = %-3u\n\r", data->photosensitivityLevel);
	stream.print(buf);
	sprintf(buf, "outPinState            = %-3u\n\r", data->outPinState);
	stream.print(buf);
	stream.println();
	stream.flush();
}
#endif // #ifdef DEBUG

