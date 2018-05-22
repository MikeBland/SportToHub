// SPort protocol to hub translator by Mike Blandford
// This polls Sport sensors and translates values to
// the hub protocol for 'D' series receivers

/*
Every 12 mS, poll another device
Expect any response to complete before 8mS later
During the remaining 4mS translate any received data
and send out 2 (or 3 or 4) bytes of hub protocol if available

Hub protocol
Frame1 (send every 200mS, 49 bytes)
Three-axis Acceleration Values, Altitude (variometer-0.01m), Temperature1, Temperature2, Voltage ,
Current & Voltage (Ampere Sensor) , RPM
e.g:5e 24 00 04 5e 25 80 ff 5e 26 e0 fe 5e 10 3c 00 5e 21 3c 00 5e 02 ef ff 5e 05 e9 ff 5e 06 18 34
 5e 28 02 00 5e 3a 0a 00 5e 3b 05 00 5e 03 63 00 5e

Note: the first 4 bit of the voltage data refers to battery cell number, while the last 12 bit refers to the
voltage value. 0-2100 corresponding to 0-4.2V.
e.g:
......0x5E 0x06 0x18 0x34 0x5E........
0x06 refers to the voltage DataID
0x18 0x34
0001 1000 0011 0100
0001(1) means the first cell of pack, the last 12bit 0x834 (2100) means the value is 4.2V

Note: Real RPM value should be the RPM value in Frame1*60
Note: Real three-axis acceleration values should be the three-axis acceleration values in Frame1/1000

Frame2 (send every 1 second, 53 bytes)
Course, Latitude, Longitude, Speed, Altitude (GPS), Fuel Level
e.g:5e 14 2c 00 5e 1c 03 00 5e 13 38 0c 5e 1b c9 06 5e 23 4e 00 5e 12 ef 2e 5e 1a 98 26 5e 22 45 00
 5e 11 02 00 5e 19 93 00 5e 01 18 00 5e 09 05 00 5e 04 64 00 5e

Frame3 (send every 5 seconds, 17 bytes)
Date, Time
e.g: 5e 15 0f 07 5e 16 0b 00 5e 17 06 12 5e 18 32 00 5e
DATE: 15.07.2011
TIME: 06:18:50

NOTE: Maximum HUB data rate must not exceed 120 bytes/sec

*/

#include <Arduino.h>
#include "Aserial.h"

#define A2_ID       0xF103

typedef void prog_void __attribute__((__progmem__));//,deprecated("prog_void type is deprecated.")));
typedef char prog_char __attribute__((__progmem__));//,deprecated("prog_char type is deprecated.")));
typedef unsigned char prog_uchar __attribute__((__progmem__));//,deprecated("prog_uchar type is deprecated.")));
typedef int8_t    prog_int8_t   __attribute__((__progmem__));//,deprecated("prog_int8_t type is deprecated.")));
typedef uint8_t   prog_uint8_t  __attribute__((__progmem__));//,deprecated("prog_uint8_t type is deprecated.")));
typedef int16_t   prog_int16_t  __attribute__((__progmem__));//,deprecated("prog_int16_t type is deprecated.")));
typedef uint16_t  prog_uint16_t __attribute__((__progmem__));//,deprecated("prog_uint16_t type is deprecated.")));
typedef int32_t   prog_int32_t  __attribute__((__progmem__));//,deprecated("prog_int32_t type is deprecated.")));
typedef uint32_t  prog_uint32_t __attribute__((__progmem__));//,deprecated("prog_uint32_t type is deprecated.")));

#define APM __attribute__(( section(".progmem.data") ))

uint16_t Xanalog ;
uint16_t Analog ;
uint16_t AnaAve ;
uint8_t AnaCount ;

//struct t_sportData MyData ;

extern void processSportData() ;

void init()
{
}

uint32_t LastMillis ;
uint32_t SlotMillis ;
uint32_t LedMillis ;
uint32_t HubMillis ;

//************************************************************************************************** Setup()
void setup()
{
//  pinMode(PIN_LED, OUTPUT); // The signal LED
//	MyData.next = 0 ;
//	initSportUart( &MyData ) ;
	initSportUart() ;
	DDRB |= 0x20 ;
	PORTB &= ~0x20 ;
	sei() ;
}

//  #define TIME_TO_SEND() (1)
//************************************************************************************************** Loop()

void loop()
{
	uint32_t now ;
	now = millis() ;
	if ( ( now - LedMillis ) > 499 )
	{
		LedMillis += 500 ;
		PORTB ^= 0x20 ;
	}
	
	if ( ( now - LastMillis ) > 11 )
	{
		LastMillis += 12 ;
		SlotMillis = now ;
		pollSport() ;
	}
	
	if ( now > ( SlotMillis + 6 ) )
	{
		SlotMillis = now + 40 ;	// Way ahead
		sendHubData() ;
		processSportData() ;
	}

	if ( ( now - HubMillis ) > 249 )
	{
		if ( Sending == 0 )
		{
			HubMillis += 250 ;
			if ( ( now - HubMillis ) > 40 )
			{
				HubMillis = now ;
			}
			sendHubPacket() ;
		}
	}

}

