/************************************************************************//**
 * Generates a pulsed sequence for two LED groups, using TIMER1 PWM mode.
 * For using with IRON-TAG.
 *
 * doragasu, 2016
 ****************************************************************************/
#include <avr/signature.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <stdint.h>
#include <string.h>

// Fuses configuration. Uses 8MHz/8 internal clock. 128 kHz internal clock / 8
// should be enough and drain less power, but using this configuration makes
// impossible to flash the MCU via SPI unless you have a programmer that can
// use a 4 kHz or lower clock.
FUSES = {
	.low = (FUSE_CKDIV8 & FUSE_SUT0 & FUSE_CKSEL3 & FUSE_CKSEL2 & FUSE_CKSEL0),
    .high = (FUSE_SPIEN)
};

/*
 * I/O WIRING:
 * PB1 (OC1A) --> LD1 (center and external LEDs).
 * PB4 (OC1B) --> LD2 (internal LEDs).
 * SW1 --> INT0 (low level turns on LED sequence).
 */
#define LD1_PIN 		PIN1
#define LD2_PIN			PIN4
#define PUSHBUTTON_PIN	PIN2

// sin^2(x) LUT from 0 to pi/2
const uint8_t sin2_lut[] = {
	0x00, 0x00, 0x01, 0x01, 0x03, 0x04, 0x06, 0x08,
	0x0A, 0x0D, 0x10, 0x13, 0x16, 0x1A, 0x1E, 0x22,
	0x26, 0x2B, 0x30, 0x35, 0x3A, 0x40, 0x45, 0x4B,
	0x51, 0x57, 0x5D, 0x63, 0x69, 0x70, 0x76, 0x7C,
	0x83, 0x89, 0x8F, 0x96, 0x9C, 0xA2, 0xA8, 0xAE,
	0xB4, 0xBA, 0xBF, 0xC5, 0xCA, 0xCF, 0xD4, 0xD9,
	0xDD, 0xE1, 0xE5, 0xE9, 0xEC, 0xEF, 0xF2, 0xF5,
	0xF7, 0xF9, 0xFB, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF
};

#define SIN2_LUT_NSAMPLES	(sizeof(sin2_lut))

// Status of the LED group
typedef enum {
	STAT_IDLE = 0,		// Idle status, LEDs OFF
	STAT_UP,			// Increasing LED brightness
	STAT_DOWN			// Decreasing LED brightness
} Status;

// Static data used to control a single LED group
typedef struct {
	volatile Status s;	// System status
	uint8_t idx;		// LUT index
	uint8_t cyc;		// Number of pending cycles
} StatData;

// Data corresponding to LD1 LEDs
static StatData ld1;
// Data corresponding to LD2 LEDs
static StatData ld2;

// PWM timer configuration, uses TIMER1 with outputs on OC1A and OC1B
// Note timer is configured to always run, it is stopped only when
// entering Power Down mode.
static inline void PwmConfig(void) {
	// OC1[AB] clear on match, set on overflow/OC1C match, /64 prescaler
	TCCR1 = (1<<PWM1A) | (1<<COM1A1) | (1<<CS12) | (1<<CS11) | (1<<CS10);
	GTCCR = (1<<PWM1B) | (1<<COM1B1);
	// Set OCR1C to max value
	OCR1C = 0xFF;
	// Enable overflow interrupt
	TIMSK = (1<<TOIE1);
}

// System initialization: static data, I/O and interrupts.
static inline void SysInit(void) {
	// Static data initialization
	memset((void*)&ld1, 0, sizeof(ld1));
	memset((void*)&ld2, 0, sizeof(ld2));
	// Disable clock for all peripherals excepting TIMER1 to save power.
	PRR = (1<<PRADC) | (1<<PRUSI) | (1<<PRTIM0);
	// Set Sleep Mode to Power Down
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	// Configure PWM outputs
	DDRB = (1<<LD1_PIN) | (1<<LD2_PIN);
	// Turn off LEDs, enable pushbutton pin pull-up
	PORTB = (1<<PUSHBUTTON_PIN);
	// Configure TIMER1 for PWM operation
	PwmConfig();
}

static inline void SysPowerDown(void) {
	cli();					// Disable interrupts for atomic execution
	sleep_enable();			// Enable SLEEP instruction
	// Ensure LEDs are OFF
	PORTB &= ~((1<<LD1_PIN) | (1<<LD2_PIN));
	GIMSK = (1<<INT0);		// Enable pushbutton interrupt
	// WARNING: BOD disable is only implemented for ATtiny85 devices
	// revision C and newer
	sleep_bod_disable();	// Disable BOD during sleep
	sei();					// Enable global interrupts
	sleep_cpu();			// Go to sleep mode
	sleep_disable();		// Disable SLEEP instruction
}

// Configures LEDs to start the blinking sequence
static inline void SysLedBlinkStart(void) {
	// Just set indexes, cycles  and status. Interrupts will do the hard work
	ld1.cyc = 8;
	ld2.cyc = 4;
	ld1.idx = ld2.idx = 0;
	ld1.s = ld2.s = STAT_UP;
}

// Test if any of the LEDs are blinking
#define LedsBlinking()	(ld1.s || ld2.s)

// Main program
int main(void) {
	// Configure system
	SysInit();

	// Main loop, one iteration per complete LED sequence.
	while (1) {
		// Power down until button pressed
		SysPowerDown();
		// Pushbutton interrupt has awakened the system here
		set_sleep_mode(SLEEP_MODE_IDLE);
		// Start LEDs blinking sequence
		SysLedBlinkStart();
		// Enter IDLE state while LEDs blinking
		while (LedsBlinking()) sleep_cpu();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	}
	return 0;
}

// Pushbutton interrupt, used to wake from sleep mode.
ISR(INT0_vect) {
	// Disable pushbutton interrupt to prevent the CPU from being locked if
	// user keeps the button pressed
	GIMSK &= ~(1<<INT0);
}

// Timer overflow interrupt, update PWM values depending on LED group status.
ISR(TIMER1_OVF_vect) {
	// NOTE: These values will not be effective until next overflow occurs.
	// Update OCR1A for LED group 1.
	switch (ld1.s) {
		case STAT_IDLE:
			break;

		case STAT_UP:
			OCR1A = sin2_lut[ld1.idx++];
			if (SIN2_LUT_NSAMPLES == ld1.idx) {
				ld1.s = STAT_DOWN;
			}
			break;

		case STAT_DOWN:
			OCR1A = sin2_lut[--ld1.idx];
			if (0 == ld1.idx) {
				ld1.s = --ld1.cyc?STAT_UP:STAT_IDLE;
			}
			break;
	}

	// Update OCR1B for LED group 2
	switch (ld2.s) {
		case STAT_IDLE:
			break;

		case STAT_UP:
			OCR1B = sin2_lut[ld2.idx++>>1];
			if ((SIN2_LUT_NSAMPLES<<1) == ld2.idx) {
				ld2.s = STAT_DOWN;
			}
			break;

		case STAT_DOWN:
			OCR1B = sin2_lut[--ld2.idx>>1];
			if (0 == ld2.idx) {
				ld2.s = --ld2.cyc?STAT_UP:STAT_IDLE;
			}
			break;
	}
}

