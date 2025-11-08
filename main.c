/* 
 * File:   main.c
 * Author: mcboatface
 *
 * Created on November 6, 2025, 9:20 PM
 */

#include <stdint.h>
#include <time.h>     // for time()
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL  // 20 MHz internal oscillator
#include <util/delay.h>

#define MOTOR_PIN PIN2_bm  // Bit mask for PA2

#define MOTOR_PORT VPORTA
#define MOTOR_PIN_INTERNAL 2
#define MOTOR_ON()     (MOTOR_PORT.OUT |= (1 << MOTOR_PIN_INTERNAL))
#define MOTOR_OFF()    (MOTOR_PORT.OUT &= ~(1 << MOTOR_PIN_INTERNAL))
// #define MOTOR_TOGGLE() (MOTOR_PORT.OUT ^= (1 << MOTOR_PIN_INTERNAL))
void do_three_times(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        MOTOR_ON();
        _delay_ms(100);
        MOTOR_OFF();
        _delay_ms(100);
    }
}
static uint32_t seed = 12345;  // you can initialize this with any nonzero value
void prng_seed_from_timer(void); 

// get a seed for pseudo-random number generator (PRNG) using chip's timer)
void prng_seed_from_timer(void) {
    // Enable TCA0 with no prescaling, normal mode
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

    // Wait a little to let the counter run
    for (volatile uint16_t i = 0; i < 1000; i++);

    // Read the timer count (varies unpredictably)
    seed = TCA0.SINGLE.CNT;

    // Optionally mix in a fixed offset to avoid zero seed
    if (seed == 0)
        seed = 0xACE1;
}

// Basic pseudorandom generator - linear congruential generator (LCG)
uint32_t pseudo_random(void)
{
    seed = (1103515245 * seed + 12345);
    return (seed >> 8);   // shift down to remove low-bit patterns
}

// Return a number between min and max (inclusive)
uint32_t random_range(uint32_t min, uint32_t max)
{
    uint32_t r = pseudo_random();
    return min + (r % (max - min + 1));
}

void variable_delay_ms(uint32_t ms)
// because _delay_ms requires compile time constant
{
    while (ms--) {
        _delay_ms(1);
    }
}


int main(void) {
    // get a better seed from the timer
    prng_seed_from_timer();
    // Set PA2 as output
    PORTA.DIRSET = MOTOR_PIN;

    // brief pulses to verify code flashed correctly
    do_three_times();
    
    while (1) {
        // Toggle motor on/off
        // TODO: ensure the range of time motor is on for is defined separately
        // and much less than the range of time motor may be OFF for 
        PORTA.OUTTGL = MOTOR_PIN;

        // pseudorandom delay
        uint32_t value = random_range(100, 2000);

        variable_delay_ms(value);
    }
}
