#include <stdbool.h>

#include <avr/wdt.h> 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "kicker_commands.h"
#include "pins.h"

#define NO_COMMAND 0

#define TIMING_CONSTANT 125
#define VOLTAGE_READ_DELAY_MS 40

// Used to time kick and chip durations
volatile unsigned millis_left_ = 0;

// Used to keep track of current button state
volatile int kick_db_held_down_ = 0;
volatile int chip_db_down_ = 0;
volatile int charge_db_down_ = 0;

volatile uint8_t byte_cnt = 0;

volatile uint8_t cur_command_ = NO_COMMAND;

// always up-to-date voltage so we don't have to get_voltage() inside interupts
volatile uint8_t last_voltage_ = 0;

volatile bool charge_allowed_ = false;

// executes a command coming from SPI
uint8_t execute_cmd(uint8_t, uint8_t);

void init();

/* Voltage Function */
uint8_t get_voltage() {
    // Start conversation by writing to start bit
    ADCSRA |= _BV(ADSC);

    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC))
        ;

    // ADHC will range from 0 to 255 corresponding to 0 through VCC
    return ADCH;
}

/*
 * Returns true if charging is currently active
 */
bool is_charging() { return PORTB & _BV(CHARGE_PIN); }

void main() {
    init();

    // needs to be int to force voltage_accum calculation to use ints
    const int kalpha = 32;

    // We handle voltage readings here
    while (true) {
        // get a voltage reading by weighing in a new reading, same concept as
        // TCP RTT estimates (exponentially weighted sum)
        last_voltage_ = get_voltage();
        int voltage_accum =
            (255 - kalpha) * last_voltage_ + kalpha * get_voltage();
        last_voltage_ = voltage_accum / 255;

        /* Don't allow charging during a kick */
        if (last_voltage_ < 230 && millis_left_ == 0) {
            PORTB |= _BV(CHARGE_PIN);
        } else if (last_voltage_ > 240) {
            PORTB &= ~(_BV(CHARGE_PIN));
        }

        if (!(PORTB & _BV(N_KICK_CS_PIN))) {
            byte_cnt = 0;
        }

        _delay_ms(VOLTAGE_READ_DELAY_MS);
    }
}

void init() {
    cli(); // disable interrupts

    // disable watchdog
    wdt_reset();
    MCUSR &= ~(_BV(WDRF));
    WDTCR |= (_BV(WDCE)) | (_BV(WDE));
    WDTCR = 0x00;
    /* Outputs */
    DDRA |= _BV(KICK_MISO_PIN);
    DDRB |= _BV(KICK_PIN) | _BV(CHARGE_PIN);

    /* Inputs */
    DDRA &= ~(_BV(N_KICK_CS_PIN) | _BV(V_MONITOR_PIN) | _BV(KICK_MOSI_PIN));

    /* SPI Init */
    SPCR = _BV(SPE) | _BV(SPIE);
    SPCR &= ~(_BV(MSTR)); // ensure we are a slave SPI device

    // enable interrupts for PCINT0-PCINT7
    PCICR |= _BV(PCIE0);

    // enable interrupts on debug buttons
    PCMSK0 = _BV(INT_DB_KICK) | _BV(INT_DB_CHG);


    // Set low bits corresponding to pin we read from
    ADMUX |= _BV(ADLAR) | 0x00; // connect PA0 (V_MONITOR_PIN) to ADC

    // Interrupt on TIMER 0
    TIMSK0 |= _BV(OCIE0A);

    // CTC - Clear Timer on Compare Match
    TCCR0A |= _BV(WGM01);

    // OCR0A is max val of timer before reset
    // we need 1000 clocks at 1 Mhz to get 1 millisecond
    // if we prescale by 8, then we need 125 on timer to get 1 ms exactly
    OCR0A = TIMING_CONSTANT;  // reset every millisecond

    // ensure ADC isn't shut off
    PRR &= ~_BV(PRADC);
    ADCSRA |= _BV(ADEN);   // enable the ADC - Pg. 133

    // enable global interrupts
    sei();
}

/*
 * SPI Interrupt. Triggers when we have a new byte available, it'll be
 * stored in SPDR. Writing a response also occurs using the SPDR register.
 */
ISR(SPI_STC_vect) {
    // SPDR contains the data
    uint8_t recv_data = SPDR;

    SPDR = 0xFF;
    // increment our received byte count and take appropriate action
    if (byte_cnt == 0) {
        // we don't have a command already, set the response
        // buffer to the command we received to let the
        // master confirm the given command if desired, top
        // bit is set if currently charging
        cur_command_ = recv_data;
        SPDR = 0x20;
    } else if (byte_cnt == 1) {
        // execute the currently set command with
        // the newly given argument, set the response
        // buffer to our return value
        SPDR = execute_cmd(cur_command_, recv_data);
    } else if (byte_cnt == 2) {
        SPDR = is_charging();
    }
    int NUM_BYTES = 3;
    byte_cnt++;
    byte_cnt %= NUM_BYTES;
}

/*
 * Interrupt if the state of any button has changed
 * Every time a button goes from LOW to HIGH, we will execute a command
 *
 * ISR for PCINT8 - PCINT11
 */
ISR(PCINT0_vect) {
    // First we get the current state of each button, active low
    int kick_db_pressed = !(PINA & _BV(DB_KICK_PIN));
    int charge_db_pressed = !(PINA & _BV(DB_CHG_PIN));

    if (!kick_db_held_down_ && kick_db_pressed)
        execute_cmd(KICK_CMD, DB_KICK_TIME);

    // toggle charge
    if (charge_db_pressed)
        execute_cmd(SET_CHARGE_CMD, ON_ARG);
    else
        execute_cmd(SET_CHARGE_CMD, OFF_ARG);
    if (!charge_db_down_ && charge_db_pressed) {
        // check if charge is already on, toggle appropriately
        if (PINA & _BV(CHARGE_PIN)) {
            execute_cmd(SET_CHARGE_CMD, OFF_ARG);
        } else {
            execute_cmd(SET_CHARGE_CMD, ON_ARG);
        }
    }

    // Now our last state becomes the current state of the buttons
    kick_db_held_down_ = kick_db_pressed;
    charge_db_down_ = charge_db_pressed;
}

/*
 * Timer interrupt for chipping/kicking - called every millisecond by timer
 *
 * ISR for TIMER 0
 */
ISR(TIMER0_COMPA_vect) {
    // decrement ms counter
    millis_left_--;

    // if the counter hits 0, clear the kick/chip pin state
    if (!millis_left_) {
        PORTB &= ~_BV(KICK_PIN);
        // stop prescaled timer
        TCCR0B &= ~_BV(CS01);
    }
}

/*
 * Executes a command that can come from SPI or a debug button
 *
 * WARNING: This will be called from an interrupt service routines, keep it
 * short!
 */
uint8_t execute_cmd(uint8_t cmd, uint8_t arg) {
    // if we don't change ret_val by setting it to voltage or
    // something, then we'll just return the command we got as
    // an acknowledgement.
    uint8_t ret_val = BLANK;

    switch (cmd) {
        case KICK_CMD:
            millis_left_ = arg;

            /* Turn off charging during kick */
            PORTB &= ~(_BV(CHARGE_PIN));

            PORTB |= _BV(KICK_PIN);  // set KICK pin
            TCCR0B |= _BV(CS01);     // start timer /8 prescale
            break;

        case CHIP_CMD:
            millis_left_ = arg;
            //PORTA |= _BV(CHIP_PIN);  // set CHIP pin
            TCCR0B |= _BV(CS01);     // start timer /8 prescale
            break;

        case SET_CHARGE_CMD:
            // set state based on argument
            if (arg == ON_ARG) {
                ret_val = 1;
                charge_allowed_ = true;
            } else if (arg == OFF_ARG) {
                ret_val = 0;
                charge_allowed_ = false;
            }
            break;

        case GET_VOLTAGE_CMD:
            ret_val = last_voltage_;
            break;

        case PING_CMD:
            ret_val = 0xFF;
            // do nothing, ping is just a way to check if the kicker
            // is connected by checking the returned command ack from
            // earlier.
            break;

        default:
            // return error value to show arg wasn't recognized
            ret_val = 0xCC;  // return a weird value to show arg wasn't recognized
            break;
    }

    return ret_val;
}
