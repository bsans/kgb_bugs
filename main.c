/* 
 * File:   main.c
 * Author: mcboatface
 * 
 * ATtiny202: toggle PA2 with long random delays/sleeps
 */

#include <stdint.h>
#include <time.h>     // for time()
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 312500UL // internal 20 MHz prescaled by /64, set below in code
//#define F_CPU 3333333UL // default value, after reset has /6 prescaler
#include <util/delay.h>

void do_three_times(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        VPORTA.OUT |= PIN2_bm; // high
        _delay_ms(100);
        VPORTA.OUT &= ~PIN2_bm; // low
        _delay_ms(100);
    }
}

volatile uint32_t target_seconds = 0u;  // defined by PRNG

// ---------------- PRNG (xorshift32) ----------------
static uint32_t rng_state = 0x12345678u;

static void rng_seed(uint32_t s) {
    if (s == 0) s = 0xCAFEBABE;   // avoid zero state
    rng_state = s;
}
static uint32_t xorshift32(void) {
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}
static uint32_t rand_range_u32(uint32_t lo, uint32_t hi_inclusive) {
    // ?modulo? mapping is fine here (long waits, not cryptographic)
    uint32_t span = (hi_inclusive - lo) + 1u;
    return lo + (xorshift32() % span);
}

// --------------- Clock + low-power setup ---------------
static void clock_slow_down(void) {
    // Use 20 MHz internal oscillator with /64 prescaler => ~312.5 kHz CPU
    // do this to save power
    // Lowest prescale available on tinyAVR 1-series is /64.
    // Write-protected: need the 'change enable' signature.
    // Disable prescaler first (ensures known state), then enable with DIV64.
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0; // prescaler off
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm;
}

static void gpio_lowpower_init(void) {
    // Leave PA0 (UPDI) alone so you don't lock yourself out.
    // Drive PA1 and PA3 low as outputs. PA2 is our controlled pin.
    // (If your board uses PA1/PA3, adjust accordingly.)
    // The value in having these all as outputs is they will not be floating and
    // thus will not draw power
    VPORTA.DIR |= (PIN1_bm | PIN2_bm | PIN3_bm); // outputs for PA1,PA2,PA3
    VPORTA.OUT &= ~(PIN1_bm | PIN3_bm);          // hold unused low
    // PA2 starts low; we'll set it high/low in the loop
    VPORTA.OUT &= ~PIN2_bm;

    // Enable pull-ups nowhere (we're driving), avoid floating inputs.
    // If any remaining pins are inputs on your PCB, add pull-ups there.
}

static void peripherals_off_after_seed(void) {
    // ADC off
    ADC0.CTRLA = 0;
    // Analog comparator off
    AC0.CTRLA = 0;
    // USART0 off
    USART0.CTRLA = 0;
    USART0.CTRLB = 0;
    // TWI (I2C) off (if present)
    // TWI0.MCTRLA = 0;  // Uncomment if your toolchain header defines TWI0
    // SPI off (if present)
    // SPI0.CTRLA = 0;

    // Disable Timers after using for seeding
    TCA0.SINGLE.CTRLA = 0;
    TCB0.CTRLA = 0x00; 
}

// --------------- Random seed from timer jitter ---------------
static void seed_from_timers(void) {
    // Start TCA0 as a free-running 16-bit counter at the (now prescaled) clk_per.
    // The exact value when we read it will vary based on start-time jitter.
    TCA0.SINGLE.CNT = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

#ifdef TCB0
    // Also start TCB0 (8/16-bit depending on mode) to mix in more bits.
    TCB0.CNT = 0;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
#endif

    // Burn a few cycles unpredictably
    for (volatile uint16_t i = 0; i < 1234; ++i) { __asm__ __volatile__("nop"); }

    uint32_t s = (uint32_t)TCA0.SINGLE.CNT;
#ifdef TCB0
    s ^= ((uint32_t)TCB0.CNT << 16);
#endif
    s ^= (uint32_t)(VPORTA.IN); // mix current pin states (noise from environment minimal but harmless)
    rng_seed(s);
}

static void sleep_seconds(uint32_t target) {
    // sleep this many seconds, one second at a time, due to compile time
    // constant required for argument to _delay_ms
    for (uint32_t i = 0; i < target; i++) {
        _delay_ms(1000);
    }
    
}

// -------------------- Main --------------------
int main(void) {
     /* this can Disable the main clock prescaler (run at full speed) */
//    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0); 
    
    clock_slow_down();  // first because delay depends on correct F_CPU
    gpio_lowpower_init();
    // fire the LED so we know something worked
    do_three_times();

    seed_from_timers();          // uses TCA/TCB briefly
    peripherals_off_after_seed();// turn off what we can

    while (1) {
        // TODO: pick real values
        
        // high first
        VPORTA.OUT |= PIN2_bm;   // PA2 high
        // get a new pseudo-random value for seconds to delay
        target_seconds = rand_range_u32(1u, 4u);
        sleep_seconds(target_seconds);
        
        VPORTA.OUT &= ~PIN2_bm;   // PA2 low
        target_seconds = rand_range_u32(5u, 20u);
        sleep_seconds(target_seconds);
//         (Optional) Stir the RNG state a bit more from the timer
//         to reduce any long-term patterns
        TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
        for (volatile uint16_t i = 0; i < 100; ++i) { __asm__ __volatile__("nop"); }
        rng_seed(rng_state ^ TCA0.SINGLE.CNT);
        TCA0.SINGLE.CTRLA = 0;
    }
}
