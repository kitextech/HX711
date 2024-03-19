/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#include <Arduino.h>
#include "HX717Parallel.h"

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )

#if HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif

#if FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
void shiftInSlow(uint8_t *dataPin, uint8_t *clockPin, uint8_t *resultOffset, uint8_t bitOrder, byte count) {
    
    
    
    for(uint8_t i = 0; i < 8; ++i) {
        
        for (byte j = 0; j < count; j++) {
            digitalWrite(clockPin[j], HIGH);
        }
        delayMicroseconds(1);

        for (byte j = 0; j < count; j++) {
            if(bitOrder == LSBFIRST)
                resultOffset[j*count] |= digitalRead(dataPin[j]) << i;
            else
                resultOffset[j*count] |= digitalRead(dataPin[j]) << (7 - i);
            digitalWrite(clockPin[j], LOW);
        }
        delayMicroseconds(1);
    }
}
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,result,order,count) shiftInSlow(data,clock,result,order,count)
#else
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftIn(data,clock,order)
#endif

#if ARCH_ESPRESSIF
// ESP8266 doesn't read values between 0x20000 and 0x30000 when DOUT is pulled up.
#define DOUT_MODE INPUT
#else
#define DOUT_MODE INPUT_PULLUP
#endif


HX717Parallel::HX717Parallel() {
}

HX717Parallel::~HX717Parallel() {
}

void HX717Parallel::begin(byte *_dout, byte *_pd_sck, long *_offset, float *_scale, const byte _count) {

    // Initialize library with data output pin, clock input pin and gain factor.

	dout = _dout;
    pd_sck = _pd_sck;
    offset = _offset;
    scale = _scale;
    count = _count;


    for (byte i = 0; i < count; i++) {
        pinMode(pd_sck[i], OUTPUT);
        pinMode(dout[i], DOUT_MODE);
    }

	// pinMode(PD_SCK, OUTPUT);
	// pinMode(DOUT, DOUT_MODE);

	// set_gain(gain);
}

bool HX717Parallel::is_ready() {

    // return true if all are true
    for (byte i = 0; i < count; i++) {
        if (digitalRead(dout[i]) == HIGH) {
            return false;
        }
    }
    return true;
}


void HX717Parallel::read(long *result) {

	// Wait for the chip to become ready.
	// wait_ready();

	// Define structures for reading data into.
	uint8_t *data = (uint8_t *)result; 
	uint8_t filler = 0x00;

    // set result to 0
    for (byte i = 0; i < count; i++) {
        result[i] = 0;
    }

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
	// interrupts during the sequence and then restores the interrupt mask to its previous
	// state after the sequence completes, insuring that the entire read-and-gain-set
	// sequence is not interrupted.  The macro has a few minor advantages over bracketing
	// the sequence between `noInterrupts()` and `interrupts()` calls.
	#if HAS_ATOMIC_BLOCK
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

	#elif IS_FREE_RTOS
	// Begin of critical section.
	// Critical sections are used as a valid protection method
	// against simultaneous access in vanilla FreeRTOS.
	// Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
	// context switches and servicing of ISRs during a critical section.
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

	#else
	// Disable interrupts.
	noInterrupts();
	#endif

    // uint8_t *dataPin, uint8_t *clockPin, uint8_t *resultOffset, uint8_t bitOrder, byte count
	// Pulse the clock pin 24 times to read the data.
	// data[2] = SHIFTIN_WITH_SPEED_SUPPORT(dout, PD_SCK, MSBFIRST);
	// data[1] = SHIFTIN_WITH_SPEED_SUPPORT(dout, PD_SCK, MSBFIRST);
	// data[0] = SHIFTIN_WITH_SPEED_SUPPORT(dout, PD_SCK, MSBFIRST);

    // shift in 3 bytes for all HX717s
    // for (byte i = 0; i < 3; i++) {

#if BYTE_ORDER == LITTLE_ENDIAN
      // Little endian system
    SHIFTIN_WITH_SPEED_SUPPORT(dout, pd_sck, &data[2], MSBFIRST, count);
    SHIFTIN_WITH_SPEED_SUPPORT(dout, pd_sck, &data[1], MSBFIRST, count);
    SHIFTIN_WITH_SPEED_SUPPORT(dout, pd_sck, &data[0], MSBFIRST, count);
#else
    // throw compile error
    #error "Byte order not supported"

    SHIFTIN_WITH_SPEED_SUPPORT(dout, pd_sck, &data[1], MSBFIRST, count);
    SHIFTIN_WITH_SPEED_SUPPORT(dout, pd_sck, &data[2], MSBFIRST, count);
    SHIFTIN_WITH_SPEED_SUPPORT(dout, pd_sck, &data[3], MSBFIRST, count);
  // Endianness unknown or unsupported preprocessor directive
#endif



    // }

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < 1; i++) {

        for (byte j = 0; j < count; j++) {
            digitalWrite(pd_sck[j], HIGH);
        }
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
		
        for (byte j = 0; j < count; j++) {
            digitalWrite(pd_sck[j], LOW);
        }
        
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
	}

	#if IS_FREE_RTOS
	// End of critical section.
	portEXIT_CRITICAL(&mux);

	#elif HAS_ATOMIC_BLOCK
	}

	#else
	// Enable interrupts again.
	interrupts();
	#endif

	// Replicate the most significant bit to pad out a 32-bit signed integer
#if BYTE_ORDER == LITTLE_ENDIAN
    for (byte i = 0; i < count; i++) {
        if (data[2+i*4] & 0x80) {
            data[3+i*4] = 0xFF;
        } else {
            data[3+i*4] = 0x00;
        }
    }
#else
    // throw compile error
    #error "Byte order not supported"
    for (byte i = 0; i < count; i++) {
        if (data[1+i*4] & 0x80) {
            data[0+i*4] = 0xFF;
        } else {
            data[0+i*4] = 0x00;
        }
    }

#endif

}


int HX717Parallel::wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
	int count = 0;
	while (count < retries) {
		if (is_ready()) {
			return count;
		}
		delay(delay_ms);
		count++;
	}
	return -1; // didn't succeed
}

// long HX717Parallel::read_average(byte times) {
// 	long sum = 0;
// 	for (byte i = 0; i < times; i++) {
// 		sum += read();
// 		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
// 		// https://github.com/bogde/HX711/issues/73
// 		delay(0);
// 	}
// 	return sum / times;
// }

// double HX717Parallel::get_value(byte times) {
// 	return read_average(times) - OFFSET;
// }

// float HX717Parallel::get_units(byte times) {
// 	return get_value(times) / SCALE;
// }

// void HX717Parallel::tare(byte times) {
// 	double sum = read_average(times);
// 	set_offset(sum);
// }

// void HX717Parallel::set_scale(float scale) {
// 	SCALE = scale;
// }

// float HX717Parallel::get_scale() {
// 	return SCALE;
// }

// void HX717Parallel::set_offset(long offset) {
// 	OFFSET = offset;
// }

// long HX717Parallel::get_offset() {
// 	return OFFSET;
// }

// void HX717Parallel::power_down() {
// 	digitalWrite(PD_SCK, LOW);
// 	digitalWrite(PD_SCK, HIGH);
// }

// void HX717Parallel::power_up() {
// 	digitalWrite(PD_SCK, LOW);
// }
