/* ============================================================
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================*/

// Based loosely on:
// Atmel AVR304: Half Duplex Interrupt Driven Software UART

// Author: Mike Blandford

#include <Arduino.h>
#include "Aserial.h"

// Hub IDs
#define GPSALT         0x1
#define TEMP1          0x2
#define RPM            0x3
#define FUEL           0x4
#define TEMP2          0x5
#define INDVOLT        0x6
#define ALTITUDE       0x10
#define LONGITUDE      0x12
#define LATITUDE       0x13
#define LONGMINS		   0x1A
#define LATMINS				 0x1B
#define ALTIDEC        0x21
#define EASTWEST       0x22
#define NORTHSOUTH     0x23
#define CURRENT        0x28 
#define FR_VSPD        0x30
#define VOLTAGE        0x3A 
#define VOLTAGEDEC     0x3B 

// SPort IDs
#define ALT_FIRST_ID            0x0100
#define VARIO_FIRST_ID          0x0110
#define CELLS_FIRST_ID          0x0300
#define RPM_FIRST_ID            0x0500
#define T1_FIRST_ID             0x0400
#define CURR_FIRST_ID           0x0200
#define VFAS_FIRST_ID           0x0210
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_ALT_FIRST_ID        0x0820


#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

struct t_sportData *FirstData = 0 ;
struct t_sportData *ThisData = 0 ;

#define DEBUG 1

#define BR_57600     //!< Desired baudrate

//This section chooses the correct timer values for the chosen baudrate.
// Assumes a 16MHz clock and 57600 baud
#define TICKS2COUNT         278  //!< Ticks between two bits.
#define TICKS2WAITONE       278  //!< Wait one bit period.
#define TICKS2WAITONE_HALF  416	 //!< Wait one and a half bit period.

// Assumes a 16MHz clock and 9600 baud
#define TICKS96_2COUNT         278*6  //!< Ticks between two bits.
#define TICKS96_2WAITONE       278*6  //!< Wait one bit period.
#define TICKS96_2WAITONE_HALF  416*6	 //!< Wait one and a half bit period.


#define INTERRUPT_EXEC_CYCL   90       //!< Cycles to execute interrupt routines from interrupt.

//Some IO, timer and interrupt specific defines.

#define ENABLE_TIMER_INTERRUPT( )       ( TIMSK1 |= ( 1<< OCIE1A ) )
#define DISABLE_TIMER_INTERRUPT( )      ( TIMSK1 &= ~( 1<< OCIE1A ) )
#define CLEAR_TIMER_INTERRUPT( )        ( TIFR1 = (1 << OCF1A) )
  
#define TX_PIN           4                //!< Transmit data pin
#define RX_PIN           4                //!< Receive data pin
#define TCCR             TCCR1A             //!< Timer/Counter Control Register
#define TCCR_P           TCCR1B             //!< Timer/Counter Control (Prescaler) Register
#define OCR              OCR1A              //!< Output Compare Register
#define EXT_IFR          EIFR               //!< External Interrupt Flag Register
#define EXT_ICR          EICRA              //!< External Interrupt Control Register
//  #define TIMER_COMP_VECT  TIMER1_COMPA_vect  //!< Timer Compare Interrupt Vector
#define TXHUB_PIN           5                //!< Transmit data pin
//#define RXHUB_PIN           5                //!< Receive data pin


#define TRXDDR  DDRD
#define TRXPORT PORTD
#define TRXPIN  PIND

#define HUBDDR  DDRD
#define HUBPORT PORTD
#define HUBPIN  PIND


#define SET_TX_PIN( )    ( TRXPORT |= ( 1 << TX_PIN ) )
#define CLEAR_TX_PIN( )  ( TRXPORT &= ~( 1 << TX_PIN ) )
#define GET_RX_PIN( )    ( TRXPIN & ( 1 << RX_PIN ) )

#define SET_HUB_TX_PIN( )    ( HUBPORT |= ( 1 << TXHUB_PIN ) )
#define CLEAR_HUB_TX_PIN( )  ( HUBPORT &= ~( 1 << TXHUB_PIN ) )

#define BYTESTUFF       0x7D
#define STUFF_MASK      0x20

#define HUB_SEPARATOR   0x5E
#define HUB_BYTESTUFF   0x5D
#define HUB_STUFF_MASK  0x60


// UART's state.
#define   IDLE                   0             //!< Idle state, both transmit and receive possible.
#define   TRANSMIT               1             //!< Transmitting byte.
#define   TRANSMIT_STOP_BIT      2             //!< Transmitting stop bit.
#define   RECEIVE                3             //!< Receiving byte.
#define		TxPENDING              4
#define		WAITING                5
#define		TxPENDING9600          6
#define   TRANSMIT9600           7             //!< Transmitting byte.
#define   TRANSMIT9600_STOP_BIT  8             //!< Transmitting stop bit.


static volatile uint8_t state ;     //!< Holds the state of the UART.
static volatile unsigned char SwUartTXData;     //!< Data to be transmitted.
static volatile unsigned char SwUartTXBitCount; //!< TX bit counter.
static volatile uint8_t SwUartRXData;     //!< Storage for received bits.
static volatile unsigned char SwUartRXBitCount; //!< RX bit counter.

uint16_t MillisPrecount ;
uint16_t lastTimerValue ;
uint32_t TotalMicros ;
uint32_t TotalMillis ;


uint8_t LastRx ;
uint8_t TxCount ;
uint8_t TxData[2] ;
uint16_t Crc ;
uint8_t Sending ;

uint8_t TxHubPacket[60] ;
uint8_t TxHubSize ;
uint8_t TxHubIndex ;

uint8_t TxHubData[4] ;
uint8_t TxHubCount ;

uint8_t RxData[16] ;
uint8_t RxIndex ;

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


const prog_uint8_t APM Indices[] = {	0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45,
																			0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB,
                                      0xAC, 0x0D, 0x8E, 0x2F, 0xD0, 0x71,
                                      0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7,
																			0x98, 0x39, 0xBA, 0x1B } ;

uint8_t SportIndexReceived[28] ;
uint8_t SportIndexPolling ;

uint8_t PhyPollIndex ;
uint8_t PhyKnownIndex ;
uint8_t PhyType ;

uint8_t PhyIndex ;

uint8_t nextPhysicalId()
{
	uint8_t pindex ;
	uint8_t i ;
	
	if ( PhyType )	// poll known
	{
		pindex = 99 ;
		for ( i = 0 ; i < 28 ; i += 1 )
		{
			if ( SportIndexReceived[PhyKnownIndex] )
			{
				pindex = PhyKnownIndex ;
			}
			PhyKnownIndex += 1 ;
			if ( PhyKnownIndex >= 28 )
			{
				PhyKnownIndex = 0 ;
				PhyType = 0 ;
				break ;
			}
			if ( pindex != 99 )
			{
				break ;
			}
		}
		if ( pindex != 99 )
		{
			return pindex ;
		}
	}

	if ( PhyType == 0 )
	{
		for ( i = 0 ; i < 28 ; i += 1 )
		{
			if ( SportIndexReceived[PhyPollIndex] == 0 )
			{
				pindex = PhyPollIndex ;
				PhyType = 1 ;
			}
			if ( ++PhyPollIndex > 27 )
			{
				PhyPollIndex = 0 ;
			}
			if ( pindex != 99 )
			{
				return pindex ;
			}
		}
		if ( pindex == 99 )
		{
			PhyType = 1 ;
			return 0 ;
		}
	}
	return pindex ;
}


//void setNewData( struct t_sportData *pdata, uint16_t id, uint32_t value )
//{
//	pdata->dataLock = 1 ;
//	pdata->data[0] = 0x10 ;
//	pdata->data[1] = id ;
//	pdata->data[2] = id >> 8 ;
//	pdata->data[3] = value ;
//	pdata->data[4] = value >> 8 ;
//	pdata->data[5] = value >> 16 ;
//	pdata->data[6] = value >> 24 ;
//	pdata->dataLock = 0 ;
//}

 /*! \brief  External interrupt service routine.
 *
 *  The falling edge in the beginning of the start
 *  bit will trig this interrupt. The state will
 *  be changed to RECEIVE, and the timer interrupt
 *  will be set to trig one and a half bit period
 *  from the falling edge. At that instant the
 *  code should sample the first data bit.
 *
 *  \note  initSoftwareUart( void ) must be called in advance.
 */
// This is the pin change interrupt for port D
// This assumes it is the only pin change interrupt
// on this port
ISR(PCINT2_vect)
{
	if ( TRXPIN & ( 1 << RX_PIN ) )			// Pin is high = start bit (inverted)
	{
		PCICR &= ~(1<<PCIE2) ;						// pin change interrupt disabled
		state = RECEIVE ;                 // Change state
  	
  	DISABLE_TIMER_INTERRUPT() ;       // Disable timer to change its registers.
  	
  	OCR1A = TCNT1 + TICKS2WAITONE_HALF - INTERRUPT_EXEC_CYCL ; // Count one and a half period into the future.


  	SwUartRXBitCount = 0 ;            // Clear received bit counter.
  	CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
  	ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
	}
}

void pollSport()
{
	uint8_t pindex ;
	pindex = nextPhysicalId() ;
	RxIndex = 0 ;
	TxData[0] = 0x7E ;
	TxData[1] = pgm_read_byte(&Indices[pindex]) ;
	SportIndexPolling = pindex ;
	
	state = TxPENDING ;
	OCR1A = TCNT1 + 50 ;
 	CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
 	ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
}

void sendHubData()
{
	PCICR &= ~(1<<PCIE2) ;						// pin change interrupt disabled
	TxHubCount = 0 ;
	if ( TxHubIndex < TxHubSize )
	{
		TxHubData[0] = TxHubPacket[TxHubIndex++] ;
		TxHubCount = 1 ;
	}
	if ( TxHubIndex < TxHubSize )
	{
		TxHubData[1] = TxHubPacket[TxHubIndex++] ;
		TxHubCount = 2 ;
	}
	if ( TxHubIndex < TxHubSize )
	{
		TxHubData[2] = TxHubPacket[TxHubIndex++] ;
		TxHubCount = 3 ;
	}
	if ( TxHubIndex < TxHubSize )
	{
		TxHubData[3] = TxHubPacket[TxHubIndex++] ;
		TxHubCount = 4 ;
	}
	if ( TxHubCount )
	{
		state = TxPENDING9600 ;
		OCR1A = TCNT1 + 50 ;
 		CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
 		ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
	}
	else
	{
		Sending = 0 ;
	}
}


/*! \brief  Timer0 interrupt service routine.
 *
 *  Timer0 will ensure that bits are written and
 *  read at the correct instants in time.
 *  The state variable will ensure context
 *  switching between transmit and recieve.
 *  If state should be something else, the
 *  variable is set to IDLE. IDLE is regarded
 *  as a safe state/mode.
 *
 */
  
ISR(TIMER1_COMPA_vect)
{
  switch (state)
	{
  // Transmit Sport Byte.
	  case TRANSMIT :		// Output the TX buffer.
    	if( SwUartTXBitCount < 8 )
			{
    	  if( SwUartTXData & 0x01 )
				{           // If the LSB of the TX buffer is 1:
    	    CLEAR_TX_PIN() ;                    // Send a logic 1 on the TX_PIN.
    	  }
    	  else
				{                                // Otherwise:
    	    SET_TX_PIN() ;                      // Send a logic 0 on the TX_PIN.
    	  }
    	  SwUartTXData = SwUartTXData >> 1 ;    // Bitshift the TX buffer and
    	  SwUartTXBitCount += 1 ;               // increment TX bit counter.
    	}
    	else		//Send stop bit.
			{
    	  CLEAR_TX_PIN();                         // Output a logic 1.
    	  state = TRANSMIT_STOP_BIT;
    	}
	  	OCR1A += TICKS2WAITONE ;  // Count one period into the future.
  	break ;

  // Go to idle after stop bit was sent.
  case TRANSMIT_STOP_BIT:
		if ( ++TxCount < 2 )					// Have we sent 2 bytes?
		{
			SwUartTXData = TxData[TxCount] ;
    	SET_TX_PIN() ;                // Send a logic 0 on the TX_PIN.
  		OCR1A += TICKS2WAITONE ;      // Count one period into the future.
	  	SwUartTXBitCount = 0 ;
	  	state = TRANSMIT ;
		}
		else
		{
			TRXDDR &= ~( 1 << RX_PIN );   // PIN is input, tri-stated.
		  TRXPORT &= ~( 1 << RX_PIN );  // PIN is input, tri-stated.
    	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	    state = IDLE ;                // Go back to idle.
			PCIFR = (1<<PCIF2) ;					// clear pending interrupt
			PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled
		}
  break ;

  // Transmit Hub Byte.
	  case TRANSMIT9600 :		// Output the TX buffer.
    	if( SwUartTXBitCount < 8 )
			{
    	  if( SwUartTXData & 0x01 )
				{           // If the LSB of the TX buffer is 1:
    	    CLEAR_HUB_TX_PIN() ;                    // Send a logic 1 on the TX_PIN.
    	  }
    	  else
				{                                // Otherwise:
    	    SET_HUB_TX_PIN() ;                      // Send a logic 0 on the TX_PIN.
    	  }
    	  SwUartTXData = SwUartTXData >> 1 ;    // Bitshift the TX buffer and
    	  SwUartTXBitCount += 1 ;               // increment TX bit counter.
    	}
    	else		//Send stop bit.
			{
    	  CLEAR_HUB_TX_PIN();                         // Output a logic 1.
    	  state = TRANSMIT9600_STOP_BIT;
    	}
	  	OCR1A += TICKS96_2WAITONE ;  // Count one period into the future.
  	break ;

  // Go to idle after stop bit was sent.
  case TRANSMIT9600_STOP_BIT:
		if ( ++TxCount < TxHubCount )		// Have we sent all bytes?
		{
			SwUartTXData = TxHubData[TxCount] ;
    	SET_HUB_TX_PIN() ;                // Send a logic 0 on the TX_PIN.
  		OCR1A += TICKS96_2WAITONE ;      // Count one period into the future.
	  	SwUartTXBitCount = 0 ;
	  	state = TRANSMIT9600 ;
		}
		else
		{
    	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	    state = IDLE ;                // Go back to idle.
		}
  break ;

		case TxPENDING9600 :
//			TRXDDR |= ( 1 << TXHUB_PIN ) ;       // PIN is output
      SET_HUB_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
	  	OCR1A = TCNT1 + TICKS96_2WAITONE ;   // Count one period into the future.
		  SwUartTXBitCount = 0 ;
			SwUartTXData = TxHubData[0] ;
			TxCount = 0 ;
		  state = TRANSMIT9600 ;
  	break ;


  //Receive Byte.
  case RECEIVE :
    OCR1A += TICKS2WAITONE ;                // Count one period after the falling edge is trigged.
    //Receiving, LSB first.
		{
			uint8_t data ;				// Use a temporary local storage
		 	data = SwUartRXBitCount ;
    	if( data < 8 )
			{
    	  SwUartRXBitCount = data + 1 ;
				data = SwUartRXData ;
				data >>= 1 ;		         // Shift due to receiving LSB first.
    	  if( GET_RX_PIN( ) == 0 )
				{
    	    data |= 0x80 ;          // If a logical 1 is read, let the data mirror this.
    	  }
				SwUartRXData = data ;
    	}
    	else	//Done receiving
			{
				if ( RxIndex < 16 )
				{
					RxData[RxIndex++] = SwUartRXData ;
				}
				{
		    	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	  		  state = IDLE ;                // Go back to idle.
					PCIFR = (1<<PCIF2) ;					// clear pending interrupt
					PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled
				}
//				LastRx = SwUartRXData ;
    	}
		}
  break ;
  
		case TxPENDING :
			PCICR &= ~(1<<PCIE2) ;						// pin change interrupt disabled
			TRXDDR |= ( 1 << RX_PIN ) ;       // PIN is output
      SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
	  	OCR1A = TCNT1 + TICKS2WAITONE ;   // Count one period into the future.
		  SwUartTXBitCount = 0 ;
			SwUartTXData = TxData[0] ;
			TxCount = 0 ;
		  state = TRANSMIT ;
  	break ;

		case WAITING :
    	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	    state = IDLE ;                // Go back to idle.
			PCIFR = (1<<PCIF2) ;					// clear pending interrupt
			PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled
  	break ;

  // Unknown state.
	  default:        
  	  state = IDLE;                           // Error, should not occur. Going to a safe state.
  }
}


/*! \brief  Function to initialize the UART.
 *
 *  This function will set up pins to transmit and
 *  receive on. Control of Timer0 and External interrupt 0.
 *
 *  \param  void
 *
 *  \retval void
 */
void initSportUart( struct t_sportData *pdata )
{
	FirstData = ThisData = pdata ;
  //PORT
	TRXDDR &= ~( 1 << RX_PIN );       // PIN is input, tri-stated.
  TRXPORT &= ~( 1 << RX_PIN );      // PIN is input, tri-stated.

  // Timer1
	TIMSK1 &= ~( 1<< OCIE1A ) ;
  TCCR1A = 0x00 ;    //Init.
  TCCR1B = 0xC1 ;    // I/p noise cancel, rising edge, Clock/1

  //External interrupt

	PCMSK2 |= 0x10 ;			// IO4 (PD4) on Arduini mini
	PCIFR = (1<<PCIF2) ;	// clear pending interrupt
//	PCICR |= (1<<PCIE2) ;	// pin change interrupt enabled

  //Internal State Variable
  state = IDLE ;

#if DEBUG
	DDRC = 0x0F ;		// PC0,1,2,3 as o/p debug
	PORTC = 0 ;
#endif

	HUBDDR |= ( 1 << TXHUB_PIN ) ;     // PIN is output
  HUBPORT &= ~( 1 << TXHUB_PIN ) ;   // logic MARK

}

uint32_t micros()
{
	uint16_t elapsed ;
	uint8_t millisToAdd ;
	uint8_t oldSREG = SREG ;
	cli() ;
	uint16_t time = TCNT1 ;	// Read timer 1
	SREG = oldSREG ;

	elapsed = time - lastTimerValue ;
	
 #if F_CPU == 20000000L   // 20MHz clock 
   #error Unsupported clock speed
  #elif F_CPU == 16000000L  // 16MHz clock                                                  
        elapsed >>= 4 ;
  #elif F_CPU == 8000000L   // 8MHz clock
        elapsed >>= 3 ;
    #else
    #error Unsupported clock speed
  #endif

        //elapsed >>= 4 ;
	
	uint32_t ltime = TotalMicros ;
	ltime += elapsed ;
	cli() ;
	TotalMicros = ltime ;	// Done this way for RPM to work correctly
	lastTimerValue = time ;
	SREG = oldSREG ;	// Still valid from above
	
	elapsed += MillisPrecount;
	millisToAdd = 0 ;
	if ( elapsed  > 3999 )
	{
		millisToAdd = 4 ;
		elapsed -= 4000 ;
	}
	else if ( elapsed  > 2999 )
	{
		millisToAdd = 3 ;		
		elapsed -= 3000 ;
	}
	else if ( elapsed  > 1999 )
	{
		millisToAdd = 2 ;
		elapsed -= 2000 ;
	}
	else if ( elapsed  > 999 )
	{
		millisToAdd = 1 ;
		elapsed -= 1000 ;
	}
	TotalMillis += millisToAdd ;
	MillisPrecount = elapsed ;
	return TotalMicros ;
}

uint32_t millis()
{
	micros() ;
	return TotalMillis ;
}

static bool checkSportPacket()
{
	uint8_t *packet = RxData ;
  uint16_t crc = 0 ;
	if ( RxIndex < 8 )
	{
		return 0 ;
	}
  for ( uint8_t i = 0 ; i<8 ; i += 1 )
	{
    crc += packet[i]; //0-1FF
    crc += crc >> 8; //0-100
    crc &= 0x00ff;
  }
  return (crc == 0x00ff) ;
}

uint8_t unstuff()
{
	uint8_t i ;
	uint8_t j ;
	j = 0 ;
	for ( i = 0 ; i < RxIndex ; i += 1 )
	{
		if ( RxData[i] == BYTESTUFF )
		{
			i += 1 ;
			RxData[j] = RxData[i] ^ STUFF_MASK ; ;
		}
		else
		{
			RxData[j] = RxData[i] ;
		}
		j += 1 ;
	}
	return j ;
}


uint8_t HubInIndex ;

void frskyPushByteValue(uint8_t value)
{
	uint8_t j ;
	j = 0 ;
  // byte stuff the only byte than might need it
  if (value == HUB_SEPARATOR)
	{
    j = 1 ;
    value ^= HUB_STUFF_MASK ;
  }
  else if (value == HUB_BYTESTUFF )
	{
    j = 1 ;
    value ^= HUB_STUFF_MASK ;
  }
	if ( j )
	{
		TxHubPacket[HubInIndex++] = HUB_BYTESTUFF ;
	}
  TxHubPacket[HubInIndex++] = value ;
}

void frskyPushWordValue( uint16_t value )
{
	frskyPushByteValue( value ) ;
	frskyPushByteValue( value >> 8 ) ;
}

void setBufferData(const char id, uint16_t value)
{
	TxHubPacket[HubInIndex++] = HUB_SEPARATOR ;
	TxHubPacket[HubInIndex++] = id ;
	frskyPushWordValue( value ) ;
}

int16_t AltBp ;
uint16_t AltAp ;
int16_t Vspeed ;
uint8_t FlvssCellCount ;
uint16_t FlvssCells[6] ;
uint16_t VfasBp ;
uint16_t VfasAp ;
uint16_t Current ;
uint16_t Rpm ;
uint16_t Temp1 ;
uint16_t LongDD ;
uint16_t LatDD ;
uint16_t LongMM ;
uint16_t LatMM ;
uint16_t LongEW ;
uint16_t LatNS ;

#define	SP_TEMP1_VALID		0x0001
#define	SP_RPM_VALID			0x0002
#define	SP_CURR_VALID			0x0004
#define	SP_VFAS_VALID			0x0008
#define	SP_GPSLONG_VALID	0x0010
#define	SP_GPSLAT_VALID		0x0020

uint16_t SportReceived ;

void processSportData()
{
	uint16_t id ;
	RxIndex = unstuff() ;
	if ( checkSportPacket() )
	{
		if ( RxData[0] = 0x10 )
		{
			SportIndexReceived[SportIndexPolling] = 1 ;		// Flag active
			id = ( RxData[1] >> 4 ) | ( RxData[2] << 4 ) ;
			switch ( id )
			{
				case ALT_FIRST_ID >> 4 :		// Alt
				{
					int32_t lvalue = *( (uint32_t *) &RxData[3] ) ;
					int16_t value = lvalue / 100 ;
					AltBp = value ;
					if ( value >= 0 )
					{
						value = lvalue - (uint32_t) value * 100 ;
					}
					else if ( value < -1)
					{
			    	value = -lvalue + value * 100 ;
					}
					else
					{
						value = 0 ;
					}
					AltAp = value ;
				}
				break ;
			
				case VARIO_FIRST_ID >> 4 :		// Vario
				{
					uint16_t value = * ( (uint16_t *) &RxData[3] ) ;
					Vspeed = value ;
				}
				break ;

			  case CELLS_FIRST_ID >> 4 :		// FLVSS
				{
					uint8_t cellIndex ;
					uint32_t lvalue ;

					lvalue = *( (uint32_t *) &RxData[3] ) ;
					if ( lvalue )
					{
						FlvssCellCount = RxData[3] >> 4 ;
						cellIndex = RxData[3] & 0x0F ;
						if ( cellIndex < 5 )
						{
							lvalue = *( (uint32_t *) &RxData[3] ) ;
							FlvssCells[cellIndex] = (lvalue >> 8) & 0x0FFF ;
							FlvssCells[cellIndex+1] = lvalue >> 20 ;
						}
					}
				}
				break ;
			  
				case CURR_FIRST_ID >> 4 :		// Current
					Current  = *( (uint16_t *) &RxData[3] ) ;
					SportReceived |= SP_CURR_VALID ;
					
				break ;
			  
				case VFAS_FIRST_ID >> 4 :		// VFAS voltage
				{	
					uint32_t lvalue ;
					lvalue = *( (uint32_t *) &RxData[3] ) ;
					lvalue *= 11 ;
					lvalue /= 210 ;
					uint16_t value = lvalue / 10 ;
					VfasBp = value ;
					value = lvalue - (uint32_t) value * 10 ;
					VfasAp = value ;
					SportReceived |= SP_TEMP1_VALID ;
				}
				break ;
			
				case T1_FIRST_ID >> 4 :		// Temp 1
					Temp1 = *( (uint16_t *) &RxData[3] ) ;
					SportReceived |= SP_TEMP1_VALID ;
				break ;

				case RPM_FIRST_ID >> 4 :		// RPM
				break ;

			  case GPS_LONG_LATI_FIRST_ID :
				{	
//					Bits 31-30 00 = LAT min/10000 N
//					Bits 31-30 01 = LAT min/10000 S
//					Bits 31-30 10 = LON min/10000 E
//					Bits 31-30 11 = LON min/10000 W
					uint32_t lvalue ;
					lvalue = *( (uint32_t *) &RxData[3] ) ;
					uint8_t code = lvalue >> 30 ;
					lvalue &= 0x3FFFFFFF ;
					uint16_t bp ;
					uint16_t ap ;
					uint32_t temp ;
					temp = lvalue / 10000 ;
					bp = (temp/ 60 * 100) + (temp % 60) ;
		      ap = lvalue % 10000;
					if ( code & 2 )	// Long
					{
						LongDD = bp ;
						LongMM = ap ;
						LongEW = ( code & 1 ) ? 'W' : 'E' ;
					}
					else
					{
						LatDD = bp ;
						LatMM = ap ;
						LatNS = ( code & 1 ) ? 'S' : 'N' ;
					}
				}
				break ;

			}

		}
		RxIndex = 0 ;
	}
}

uint8_t HubCellIndex ;
uint8_t LatLongIndex ;

void sendHubPacket()
{
	HubInIndex = 0 ;
	setBufferData( ALTITUDE, AltBp ) ;
	setBufferData( ALTIDEC, AltAp ) ;
	setBufferData( FR_VSPD, Vspeed ) ;
	
	if ( FlvssCellCount )
	{
		uint16_t value = FlvssCells[HubCellIndex] ;
		TxHubPacket[HubInIndex++] = HUB_SEPARATOR ;
		TxHubPacket[HubInIndex++] = INDVOLT ;
		frskyPushByteValue( ( (value>>8) & 0x000F) + ( HubCellIndex << 4 ) ) ;
		frskyPushByteValue( value ) ;
		if ( ++HubCellIndex >= FlvssCellCount )
		{
			HubCellIndex = 0 ;
		}
	}
	
	if ( SportReceived & SP_TEMP1_VALID )
	{
		setBufferData( TEMP1, Temp1 ) ;
	}
	
	if ( SportReceived & SP_CURR_VALID )
	{
		setBufferData( CURRENT, Current ) ;
	}
	
	if ( SportReceived & SP_VFAS_VALID )
	{
		setBufferData( VOLTAGE, VfasBp ) ;
		setBufferData( VOLTAGEDEC, VfasAp ) ;
	}
	if ( LatLongIndex )
	{
		if ( SportReceived & SP_GPSLONG_VALID )
		{
			setBufferData( LONGITUDE, LongDD ) ;
			setBufferData( LONGMINS, LongMM ) ;
			setBufferData( EASTWEST, LongEW ) ;
		}
		LatLongIndex = 0 ;
	}
	else
	{
		if ( SportReceived & SP_GPSLAT_VALID )
		{
			setBufferData( LATITUDE, LatDD ) ;
			setBufferData( LATMINS, LatMM ) ;
			setBufferData( NORTHSOUTH, LatNS ) ;
		}
		LatLongIndex = 1 ;
	}
	TxHubPacket[HubInIndex++] = HUB_SEPARATOR ;
	TxHubSize = HubInIndex ;
	TxHubIndex = 0 ;
}

