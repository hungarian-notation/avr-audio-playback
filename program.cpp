#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void send_marks(int count, int len=1);
template <typename Integer>
void send_signal(Integer value);
void play_audio();

int main(void) {
	play_audio();
	while (true);
}

// Audio playback is implemented by reading a page of 512 samples from the
// external EEPROM, while the previously loaded page is clocked out through a
// PWM DAC implementation.

const unsigned int 		PAGE_SIZE 	= 512;

// The logic that reads from the EEPROM is implemented in the I2C (TWI) 
// interrupt handler, while the page swapping and the output is handled in the
// main loop. Synchronization is provided by `wait_flag` and `page_ready`.

volatile unsigned char 	PAGE_BUFFER[PAGE_SIZE][2];
volatile unsigned char 	*wpg;
volatile unsigned char 	*rpg;

volatile unsigned int 	wpt; // write-point 	(in wpg)
volatile unsigned int 	rpt; // read-point 		(in rpg)
volatile unsigned int 	mpt; // memory-point	(next EEPROM address)

volatile bool			isr_flag = false;
 
// `wait_flag` is set to true when the next value is placed in the PWM compare
// register. It is cleared at a regular interval by the TIMER1_COMPA ISR, which
// results in a stable output sample rate.

// `page_ready` is set by the TWI ISR when a new page is fully loaded. It is
// cleared after the main loop finishes outputting the previous page and swaps
// the page handle pointers.

volatile bool			wait_flag	= true;
volatile bool			page_ready 	= false;

#define TWI_STATUS (TWSR & 0b11111000)

#define TWI_ERROR_STATE		(0xFF)
#define TWI_SEND_START 		(0x01)
#define TWI_WRITE_SLAW 		(0x02)
#define TWI_WRITE_ADDR_HIGH (0x10)
#define TWI_WRITE_ADDR_LOW	(0x11)
#define TWI_REPEAT_START	(0x20)
#define TWI_WRITE_SLAR		(0x21)
#define TWI_READ_FIRST_ACK	(0x40)
#define TWI_READ_ACK		(0x41)
#define TWI_SEND_STOP		(0x80)

volatile uint8_t TWI_STATE			= TWI_SEND_START;
volatile uint8_t TWI_LAST_STATE		= 0;
volatile uint16_t ISR_INVOCATIONS	= 0;

void twi_init()
{
}

#define TWI_RAISE_ERROR(EXPECT) 		        \
{ 												\
	send_marks(5); 								\
	send_signal(ISR_INVOCATIONS); 				\
	send_signal(TWI_STATE); 					\
	send_signal(TWI_LAST_STATE);				\
	send_signal(wpt); 							\
	send_marks(1); 								\
	send_signal((uint8_t)EXPECT); 				\
	send_signal((uint8_t)TWI_STATUS); 			\
	TWI_STATE = TWI_ERROR_STATE; 				\
	return; 									\
}

#define TWI_ASSERT_STATUS(STATUS) { if (TWI_STATUS != STATUS) { TWI_RAISE_ERROR((uint8_t)STATUS); } }

void load_next_page()
{
		TWCR 		= _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
		TWI_STATE	= TWI_WRITE_SLAW;
}

// The ISR for the TWI Interrupt Vector implements a state-machine that handles
// communication with the external EEPROM chip. It uses asynchronous reads to
// read in batches of 512 bytes.

// The communication sequence is as follows:

// 	ASSERT START
// 	WRITE SLA+W, 		RECV ACK
// 	WRITE ADDR[8:16]	RECV ACK
// 	WRITE ADDR[0:7 ]	RECV ACK
// 	RE-ASSERT START
// 	WRITE SLA+R			RECV ACK
// 	LOOP 511 TIMES:
//		READ SAMPLE		SEND ACK
//	READ SAMPLE			SEND NACK
//	ASSERT STOP

// At each step, we check the status bits in the TWSR register to ensure the
// correct action and response was observed by the two-wire hardware
// peripheral.

ISR(TWI_vect)
{
	auto incoming_state = TWI_STATE;

	TWCR &= ~_BV(TWINT);
	++ISR_INVOCATIONS;

	if (TWI_STATE == TWI_ERROR_STATE) return;
	
	switch (TWI_STATE)
	{
	
	// The first state in the state machine is implemented in an external
	// function, as the ISR will not be called until after the first start
	// condition is asserted.
	
	/*
	case TWI_SEND_START:
		TWCR 		= _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
		TWI_STATE	= TWI_WRITE_SLAW;
		break;
	*/
		
	case TWI_WRITE_SLAW:
		TWI_ASSERT_STATUS(0x08); // START has been transmitted
		TWDR 		= 0b10101110; // ((0x57 << 1) | 0b0)
		TWI_STATE	= TWI_WRITE_ADDR_HIGH;
		TWCR 		= _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
		break;
		
	case TWI_WRITE_ADDR_HIGH:
		TWI_ASSERT_STATUS(0x18); // SLAW+W was ACKED
		TWDR 		= ((mpt >> 8) & 0xFF);
		TWCR 		= _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
		TWI_STATE	= TWI_WRITE_ADDR_LOW;
		break;
		
	case TWI_WRITE_ADDR_LOW:
		TWI_ASSERT_STATUS(0x28); // DATA was ACKED
		TWDR 		= (mpt & 0xFF);		
		TWCR 		= _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
		TWI_STATE	= TWI_REPEAT_START;
		
		mpt += PAGE_SIZE;
		if (mpt > 0x7FFF - PAGE_SIZE) { mpt = 0x0000; }
		
		break;
		
	case TWI_REPEAT_START:
		TWI_ASSERT_STATUS(0x28); // DATA was ACKED
		TWCR 		= _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
		TWI_STATE	= TWI_WRITE_SLAR;
		break;
		
	case TWI_WRITE_SLAR:
		TWI_ASSERT_STATUS(0x10); // START was repeated
		TWDR 		= 0b10101111; // ((0x57 << 1) | 0b1)
		TWCR 		= _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
		TWI_STATE	= TWI_READ_FIRST_ACK;
		break;
		
	case TWI_READ_FIRST_ACK:
		TWI_ASSERT_STATUS(0x40); // SLAW+R was ACKED (by slave)
		TWCR 		= _BV(TWINT) | _BV(TWEN) | _BV(TWEA) | _BV(TWIE);
		wpt			= 0;
		TWI_STATE	= TWI_READ_ACK;
		break;
		
	case TWI_READ_ACK:
		TWI_ASSERT_STATUS(0x50); // READ was ACKED (by us)
		wpg[wpt++]	= TWDR;
		TWCR 		= _BV(TWINT) | _BV(TWEN) | _BV(TWIE)
			| ((wpt < PAGE_SIZE - 1) ? _BV(TWEA) : 0x00 /*NACK*/);
		TWI_STATE	= ((wpt < PAGE_SIZE - 1) ? TWI_READ_ACK : TWI_SEND_STOP);
		break;
		
	case TWI_SEND_STOP:
		TWI_ASSERT_STATUS(0x58); // READ was NACKED (by us)
		wpg[wpt]	= TWDR;
		wpt 		= PAGE_SIZE;
		page_ready  = true;
		TWCR 		= _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWIE);
		break;
		
	default:
		TWI_RAISE_ERROR((uint8_t)0x00);
		break;
		
	}
	
	TWI_LAST_STATE = incoming_state;
}

// TIMER1_COMPA_vect ISR is triggered at our desired sample rate. It clears the
// wait flag, allowing the playback loop to consume the next sample.

ISR(TIMER1_COMPA_vect)
{
	TIFR1 &= ~_BV(2);
	wait_flag = false;
}

void init_audio()
{
	// Enable interrupts.
	sei();
	
	// Configure the baud rate of the two-wire interface to 400kHz, the max
	// specified baud rate for our EEPROM chips.
    TWSR = 0x00;
    TWBR = 0x0C;
	
	// Prepare the sample buffers and cursors.
	wpg = PAGE_BUFFER[0];
	rpg = PAGE_BUFFER[1];
	wpt = 0;
	rpt = PAGE_SIZE;
	mpt = 0;

	DDRB	= 0xFF;
	DDRD	= 0xFF;
	PORTD	= 0xFF;
	
	// Configure Timer0 for PWM output. We're using fast-PWM mode, so we get
	// 256 bits of resolution and a PWM DAC refresh rate of 62.5kHz.
	
	TCCR0A 	= 0b11000011;
	TCCR0B 	= 0b00000001;
	OCR0A 	= 0xFF/2;
	 
	// Configure Timer1 as our sample-rate clock. It should call its ISR 
	// at a rate of 20kHz.
	
	TCCR1A	= 0b00000000;
	TCCR1B	= 0b00001001;
	OCR1A	= 800; 			// 16MHz / 20kHz = 800 ticks per division
	TIMSK1	= 0b00000010; 	// Enables Output Compare A Interrupt
}

void play_audio() 
{
	unsigned int position 			= 0x0000;
	unsigned int sample_length 		= 0x4FFF; 
	
	init_audio();
	load_next_page();
		
	while (position < sample_length)
	{			
		while (rpt >= PAGE_SIZE) {
		
			// This loop captures execution when we run out of samples in our
			// read buffer.
			
			// If the wait flag is cleared before this loop breaks, an output
			// underrun has occured. This should not occur in normal operation.
		
			if (page_ready) {
				// Swap the page handles.
				auto tmp = wpg;
				wpg = rpg;
				rpg = tmp;
				
				// Clear the buffer cursors.
				rpt = 0;
				wpt = 0;
				
				// Clear the page_ready flag, and trigger the next read
				// operation.
				page_ready = false;
				load_next_page();
			}
		}
		
		// wait for the previous sample to be consumed
		while (wait_flag);
	
		OCR0A 		= rpg[rpt++];
		wait_flag	= true;
		
		++position;
	}
	
	PORTB	&= ~_BV(5);
	OCR0A 	= 0;
}

// DEBUG FUNCTIONS
//
// These functions write out integral values as binary pulse-length-coded values
// on PB7. This is very slow, so should only be used for fatal error dumps.

#define WRITE_HIGH( PORT_LETTER, BIT ) (PORT ## PORT_LETTER |= _BV(BIT))
#define WRITE_LOW( PORT_LETTER, BIT ) (PORT ## PORT_LETTER &= ~_BV(BIT))

void send_marks(int count, int len)
{
	for (int i = 0 ; i < count; ++i) {
		WRITE_HIGH(D,7);
		for (int j = 0; j < len; ++j)
			_delay_ms(15);
		WRITE_LOW(D,7);
		_delay_ms(10);
	}
}

template <typename Integer>
void send_signal(Integer value)
{
	WRITE_LOW(D,7);
	_delay_ms(5);
	send_marks(1);
		
	for (unsigned int i = 0; i < sizeof(value) * 8; ++i) {
		if ((value >> i) & 0b1) {
			WRITE_HIGH(D,7);
			_delay_ms(6);
			WRITE_LOW(D,7);
			_delay_ms(4);
		} else {
			WRITE_HIGH(D,7);
			_delay_ms(2);
			WRITE_LOW(D,7);
			_delay_ms(8);
		}
		
	}
	
	WRITE_LOW(D,7);
}
