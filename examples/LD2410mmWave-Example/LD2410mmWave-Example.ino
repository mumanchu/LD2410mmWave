///////////////////////////////////////////////////////////////////////////////
// Example for LD2410mmWave Library for LD2410B and LD2410C mmWave Sensors
// Copyright (C) https://muman.ch + https://github.com/mumanchu, 2026.11.17
// If you re-use this software, please include the above copyright notice
// 
// For details see github
// https://github.com/mumanchu/LD2410mmWave


// Comment this out to hide the debug output
#define DEBUG


// The usual 'Serial' is for the Serial Monitor so you can see the data

// 'Serial1' is used for the mmWave sensor's UART, but the pins may be 
// different on each microcontroller board. Here are some examples...

#if defined(ARDUINO_SAM_DUE)
// The Arduino Due has Serial1 on TX1 (18) and RX1 (19), connect these to RX 
// and TX of the sensor
// !!!NOTE!!!  
// If you select "No Optimisation" for the Arduino Due, then the build may
// fail with this message:
//  "HardwareSerial.h:26: undefined reference to vtable for HardwareSerial"
// It's a problem with the board package code. Set it to "Default Optimisation",
// do a 'Clean solution' and re-build. Or use the fix described here:
// https://learn.adafruit.com/proper-step-debugging-atsamd21-arduino-zero-m0/paths-and-optimizations#fixing-some-core-files-2469468-17
#endif

#if defined(ARDUINO_ARCH_STM32)
// On the STM32F103 Nucleo board, 'Serial' is the USB port
// so we use USART1, TX=D8 (PA9), RX=D2 (PA10) for the mmWave sensor
//HardwareSerial Serial1(USART1);
Uart Serial1(USART1);
#endif

#if defined(ARDUINO_ARCH_AVR)
// SoftwareSerial will not work at 115200 baud on AVR processors
//TODO
#endif

#if defined(ARDUINO_SAMD_ZERO)
// Arduino Zero has 'Serial1' on pin 0 (RX) and pin 1 (TX), so you don't need 
// to do anything. 'Serial' is the USB port, so connect your Serial Monitor to 
// the port called 'Atmel Corp. EDBG CMSIS-DAP'
#endif

#if defined(ARDUINO_ARCH_ESP32)
// For the ESP32s, Serial1's pins can be defined
// see the code in setup() below
#endif


// For LD2410mmWave library debug output
#ifdef DEBUG
void LD2410LogError(char* msg, char* file, unsigned int line)
{
	char buf[100];
	char* fname = strrchr(file, '\\');
	fname = fname ? fname + 1 : file;
	sprintf(buf, "%lu %s %s %u", millis(), msg, fname, line);
	Serial.println(buf);
	Serial.flush();
}
//extern void LD2410LogError(char* msg, char* file, unsigned int line);
#define LD2410_LOGERROR(msg) LD2410LogError(msg, __FILE__, __LINE__)
#else
#define LD2410_LOGERROR(s)
#endif

// The LD2410mmWave Library
#define LD2410_ENABLE_CONFIG
#include "LD2410mmWave.h"
LD2410mmWave sensor;


void setup()
{
	// I use different pins for Serial logging on the STM32 Nucleo-64 boards,
	// else Serial is the USB port which is also used for download
	#ifdef ARDUINO_NUCLEO_64
	Serial.setTx(PC_10);
	Serial.setRx(PC_11);
	#endif

	// Serial Monitor port
	Serial.begin(115200);
	delay(3000);

	// PuTTY clear screen and scrollback
	// comment this out if not using PuTTY
	Serial.print("\033[2J\033[H\033[3J");
	Serial.flush();


	// mmWave sensor port, 256000bps
	Serial1.begin(256000);

	// initialize the sensor, sensor is in configuration mode
	bool ok = sensor.begin(&Serial1);

	// read some data
	char fwVersion[16];
	sensor.getFirmwareVersion(fwVersion);
	byte macAdds[6];
	sensor.getMacAddress(macAdds);

	// enable this to do a factory reset
	#if 0
	ok = sensor.restoreFactorySettings();
	if (!ok) {
		Serial.println("restoreFactorySettings() failed");
		Serial.flush();
	}
	ok = sensor.restartModule();
	if (!ok) {
		Serial.println("restartModule() failed");
		Serial.flush();
	}
	// after restart, sensor is in NOT in configuration mode
	sensor.enableConfigurationMode();
	#endif

	// get the current configuration
	LD2410mmWave::CONFIGDATA configData;
	ok = sensor.getConfiguration(&configData);
	if (!ok) {
		Serial.println("getConfiguration() failed");
		Serial.flush();
	}

	// begin 'radar data mode', with additional engineering data
	sensor.startEngineeringMode();
	sensor.disableConfigurationMode();
}


void loop()
{
	LD2410mmWave::RADARDATA radarData;
	LD2410mmWave::ENGINEERINGDATA engineeringData;

	// radar data mode message received?
	if (sensor.receive(&radarData, &engineeringData)) {

		// sometimes it drops engineering mode, so we must set it again
		if (radarData.mode != 0x01) {
			sensor.enableConfigurationMode();
			sensor.startEngineeringMode();
			sensor.disableConfigurationMode();
		}
		else {
			#ifdef DEBUG
			// PuTTY cursor home, so the display doesn't scroll
			// patch out if not using putty
			Serial.print("\033[H");

			sensor.dumpRadarData(&radarData, Serial);
			sensor.dumpEngineeringData(&engineeringData, Serial);
			#endif
		}
	}
}

