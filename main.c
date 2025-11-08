/* 
 * File:   main.c
 * Author: mcboatface and Peter and chatGPT
 * ATtiny202: toggle PA2 with long random sleeps
 * Created on November 7, 2025, 11:02 PM
 */

#define F_CPU 3333333UL // because after reset, main clock prescaled down from 20/6 = 3.33
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h> 
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <avr/builtins.h> // For _PROTECTED_WRITE

volatile uint16_t seconds_counter = 0u; // incremented by ISR
volatile uint16_t target_seconds = 0u;  // defined by PRNG

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

// --------------- Timer TCB0 1s tick sleep ---------------
// Watchdog on ATtiny202 can only reset MCU, cannot interrupt

// === Initialize the 32 kHz oscillator and route it to TCB0 ===
void CLK_init(void)
{
    CCP = CCP_IOREG_gc; // Unlock protected I/O register
    // Enable the 32 kHz oscillator (default config = 0)
    
    CLKCTRL.OSC32KCTRLA = CLKCTRL_CLKSEL_OSCULP32K_gc;     // write the value

    while (!(CLKCTRL_MCLKSTATUS & CLKCTRL_OSC32KS_bm)); // wait until stable
}

// === Initialize TCB0 to use 32 kHz clock and generate periodic interrupts ===
void TCB0_init(void)
{
    // Route 32 kHz oscillator to TCB0 (instead of main clock)
    TCB0.CTRLA = (0b111 << 0) | TCB_RUNSTDBY_bm | TCB_ENABLE_bm;   // will use 32 kHz low power oscillator
//    TCB0.CTRLA = (0b000 << 0) | TCB_ENABLE_bm;  // main clock (20 MHz)
    // the CLKSEL bits = 0b111 corresponds to OSC32K
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;      // periodic interrupt mode
    TCB0.CCMP = 32768;                    // compare value for ~1 second
                                          // 32768 counts @ 32.768 kHz = 1 s
    TCB0.INTCTRL = TCB_CAPT_bm;           // enable interrupt
//    TCB0.CTRLA |= TCB_ENABLE_bm;          // enable timer
    // should this be TCB_RUNSTDBY_bm ? it seems to work the same either way
    sei();
}

// === Interrupt Service Routine ===
ISR(TCB0_INT_vect)
{
    PORTA.DIRSET = PIN2_bm; //debug empirically
    PORTA.OUT |= PIN2_bm; // pin high
//    PORTA |= 1 << 2; // set bit 2
//    seconds_counter++;
    TCB0.INTFLAGS = TCB_CAPT_bm;  // clear interrupt flag, writes a 0x01
}

// --------------- Clock + low-power setup ---------------
static void clock_slow_down(void) {
    // Use 20 MHz internal oscillator with /64 prescaler => ~312.5 kHz CPU
    // Lowest prescale available on tinyAVR 1-series is /64.
    // Write-protected: need the ?change enable? signature.
    // Disable prescaler first (ensures known state), then enable with DIV64.
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0; // prescaler off
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;
//    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm;
}

static void gpio_lowpower_init(void) {
    // Leave PA0 (UPDI) alone so you don?t lock yourself out.
    // Drive PA1 and PA3 low as outputs. PA2 is our controlled pin.
    // (If your board uses PA1/PA3, adjust accordingly.)
    // The value in having these all as outputs is they will not be floating and
    // thus will not draw power
    VPORTA.DIR |= (PIN1_bm | PIN2_bm | PIN3_bm); // outputs for PA1,PA2,PA3
    VPORTA.OUT &= ~(PIN1_bm | PIN3_bm);          // hold unused low
    // PA2 starts low; we?ll set it high/low in the loop
    VPORTA.OUT &= ~PIN2_bm;

    // Enable pull-ups nowhere (we?re driving), avoid floating inputs.
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

    // Disable TCA after using for seeding. Can't turn off TCB b/c we need it
    TCA0.SINGLE.CTRLA = 0;
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


static void sleep(void) {
    __asm__ __volatile__("sleep");  // put CPU to sleep
}

// -------------------- Main --------------------
int main(void) {
     /* this can Disable the main clock prescaler (run at full speed) */
//    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0); 
    
    // fire the LED so we know something worked
    
    PORTA.DIRSET = PIN2_bm;	
    PORTA.OUT |= PIN2_bm;
    _delay_ms(3000); // wait for sanity
    PORTA.OUT &= ~PIN2_bm;
    _delay_ms(1000); // wait off before seeing ISR work
    // fuckin sanity yay
    
    cli(); // clear interrupts during critical section so it is not interrupted

//    clock_slow_down();
//    gpio_lowpower_init();
//    seed_from_timers();          // uses TCA/TCB briefly
//    peripherals_off_after_seed();// turn off what we can
//    CLK_init();
//    TCB0_init();

    CCP = CCP_IOREG_gc;
//    // disable prescaler
    ccp_write_io( (void *) &CLKCTRL.MCLKCTRLA , (0 << CLKCTRL_PEN_bp));
//    // switch Timer B clock to 32kHz internal oscillator)
    ccp_write_io( (void *) &CLKCTRL.MCLKCTRLA, (CLKCTRL_CLKSEL_OSCULP32K_gc));
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm){;}
//    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm;
//    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm;
////    CLKCTRL.OSC32KCTRLA = CLKCTRL_CLKSEL_OSCULP32K_gc;
    TCB0.CCMP = 0x7fff; // 32767
    TCB0.CTRLA = TCB_ENABLE_bm | TCB_RUNSTDBY_bm;
//    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm;
////    TCB0.CTRLB = TCB_CNTMODE_INT_gc;  hmmmmm
    TCB0.INTCTRL = TCB_CAPT_bm;

    
//    while (!(CLKCTRL_MCLKSTATUS & CLKCTRL_SOSC_bm));  whaaaat?
    sei(); // set interrupts
    
    // keep
//    TCB0.CTRLA = (0b000 << 0) | TCB_ENABLE_bm;
//    TCB0.CTRLA = (0b111 << 0) | TCB_RUNSTDBY_bm | TCB_ENABLE_bm;
//    TCB0.CCMP = 20000; again whaaa?
    //
    
//    // todo: use SLEEP_MODE_STANDBY
//    SLPCTRL.CTRLA = SLEEP_MODE_IDLE;  // select sleep mode

//    while (1) {
        // TODO: pick real values
        
//        // high first
//        VPORTA.OUT |= PIN2_bm;   // PA2 high
//        // get a new pseudo-random value for seconds to sleep
//        target_seconds = rand_range_u32(1u, 4u);
//        while (seconds_counter < target_seconds) {
//            // HIGH phase: 1 - 4 seconds
//            sleep();
//        }
//        seconds_counter = 0;          // reset counter
//        target_seconds = rand_range_u32(5u, 20u);
//        VPORTA.OUT &= ~PIN2_bm;   // PA2 low
//        while (seconds_counter < target_seconds) {
//            // LOW phase: 5 - 20 seconds
//            sleep();
//        }
//        seconds_counter = 0;          // reset counter
        // (Optional) Stir the RNG state a bit more from the timer
        // to reduce any long-term patterns
//        TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
//        for (volatile uint16_t i = 0; i < 100; ++i) { __asm__ __volatile__("nop"); }
//        rng_seed(rng_state ^ TCA0.SINGLE.CNT);
//        TCA0.SINGLE.CTRLA = 0;
//    }
}
