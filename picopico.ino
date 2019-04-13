/*
 * picopico
 * Copyright (C) 2017  Damián Silvani
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Original code by David Johnson-Davies
 * www.technoblogy.com - 27th March 2016
 *
 * ATtiny85 @ 8MHz (internal oscillator; BOD disabled)
 *
*/

#include <avr/sleep.h>
#include "player.h"
#include "tune.h"
#include <wiring.c>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define NUM_VOICES      4
#define DEFAULT_OCTAVE  4
#define DEFAULT_NLEN    16
#define DEFAULT_VOL     15
#define DEFAULT_PW      0x80

const int PIN_LED = 13;

// Note buffer
volatile uint16_t lfsr = 1;
volatile char lfsrOut = 0;
volatile signed char oldTemp = 0; // FIXME change variable name

// Global tick counter
volatile unsigned int ticks = 0;
volatile bool nextTick = false;

Voice voices[NUM_VOICES] = {};
byte playingVoices = 0;

bool playing = false, wantsToStop = false;
volatile bool justAwoke = false;


/*// External interrupt 0
ISR(INT0_vect) {
    GIMSK = 0;            // Disable interrupt because this routine may fire multiple times
                          // while pin is held low (button pressed)
    justAwoke = true;
}*/

volatile unsigned long int timer0_compa_tick = 0;

// Watchdog interrupt counts ticks (every 16ms)
ISR(WDT_vect) {
    #if defined(WDTCSR)
        WDTCSR |= 1<<WDIE;
    #elif defined(WDTCR)
        WDTCR |= 1<<WDIE;
    #endif
    ticks++;
    nextTick = true;
}

ISR(TIMER0_COMPA_vect) {
    //timer0_compa_tick++;
    unsigned char temp;
    signed char stemp, mask, out = 0;
    Voice* v;

    // Voice 1 and 2: Pulses
    for (int c = 0; c < 2; c++) {
        v = &voices[c];
        v->acc += v->freq;
        temp = (v->acc >> 8) & v->pw;
        out += (temp ? v->amp : 0) >> 2;
    }

    // Voice 3: Triangle
    v = &voices[2];
    v->acc += v->freq;
    stemp = v->acc >> 8;
    mask = stemp >> 7;
    if (v->amp != 0) out += (stemp ^ mask) >> 1;

    // Voice 4: Noise
    //
    // This noise generator is somewhat based on the mechanism found in the NES APU.
    // The NES has a linear-feedback shift register for generating pseudorandom numbers.
    // It starts with a register set to 1, and when the period counter reaches 0, it
    // clocks the shift register.
    // The LFSR performs an Exclusive OR between bit 0 and bit 1, then shifts to the
    // right, and sets/resets bit 15 based on the exclusive OR result.
    //
    v = &voices[3];
    v->acc += v->freq;
    stemp = (v->acc >> 8) & 0x80;
    // if temp != oldTemp, trigger the LFSR to generate a new pseudorandom value
    if (stemp != oldTemp) {
        lfsrOut = (lfsr & 1) ^ ((lfsr & 2) >> 1);  // output is bit 0 XOR bit 1
        lfsr = (lfsr >> 1) | (lfsrOut << 14);      // shift and include output on bit 15
        oldTemp = stemp;
    }
    out += (lfsrOut ? v->amp : 0) >> 2;

    /*
    // for ATmega16u4-32u4
    #if defined(OCR4A)
        OCR4A = out;
    // for ATtiny25-45-85
    #elif defined(OCR1B)
        OCR1B = out;
    #endif
    */
    OCR1B = out;
}

// Setup **********************************************

void setup() {
    // set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // // Set all pins as input and enable internal pull-up resistor
    // for (int i = 0; i <= 3; i++) {
        // pinMode(i, INPUT_PULLUP);
    // }

    // Enable 64 MHz PLL and use as source for Timer1/Timer4

    // for ATtiny25-45-85
    #if defined(PLLCSR) && defined(PCKE)
        PLLCSR = 1 << PLLE; //Enable PLL
        _delay_us(100); // Wait for a sync;
        while (!(PLLCSR & (1 << PLOCK))) {
        };
        // PLL Synced
        PLLCSR |= (1 << PCKE); // Enable Asynchronous PCK mode.
    #endif
    /*
    // for ATmega16u4-32u4
    #if defined(PLLCSR) && defined(PLLFRQ)
        sbi(PLLCSR, PINDIV); // When using a 16MHz clock source, this bit must be set to 1 before enabling PLL
        sbi(PLLCSR, PLLE); //Enable PLL
        _delay_us(100); // Wait for a sync;
        while (!(PLLCSR & (1 << PLOCK))) {
        };
        // PLL Synced
        sbi(PLLFRQ, PLLTM1); // PLL Postcaler Factor for High-Speed Timer: 1.5 division
        cbi(PLLFRQ, PLLTM0);
        sbi(PLLFRQ, PDIV3); // PLL Output Frequency 96MHz
        cbi(PLLFRQ, PDIV3);
        sbi(PLLFRQ, PDIV3);
        cbi(PLLFRQ, PDIV3);
    #endif
    */
    // for ATtiny25-45-85
    #if defined(TCCR1)
        // TCCR1 => CTC1 PWM1A COM1A1 COM1A0 CS13 CS12 CS11 CS10
        // GTCCR => TSM PWM1B COM1B1 COM1B0 FOC1B FOC1A PSR1 PSR0
        
        // Set up Timer/Counter1 for PWM output
        TCCR1 = 1<<CS10;               // 1:1 prescale
        GTCCR = 1<<PWM1B | 2<<COM1B0;  // PWM B, clear on match

        OCR1B = 128;
        pinMode(4, OUTPUT);            // Enable PWM output pin
    #endif
    
    /*
    // for ATmega16u4-32u4
    #if defined(TCCR4A)
        // TCCR4A => COM4A1 COM4A0 COM4B1 COM4B0 FOC4A FOC4B PWM4A PWM4B
        // TCCR4B => PWM4X PSR4 DTPS41 DTPS40 CS43 CS42 CS41 CS40
        // TCCR4C => COM4A1S COM4A0S COM4B1S COMAB0S COM4D1 COM4D0 FOC4D PWM4D
        // TCCR4D => FPIE4 FPEN4 FPNC4 FPES4 FPAC4 FPF4 WGM41 WGM40
        // TCCR4E => TLOCK4 ENHC4 OC4OE5 OC4OE4 OC4OE3 OC4OE2 OC4OE1 OC4OE0
        sbi(TCCR4B, CS40); // set prescaler 1:1
        cbi(TCCR4B, CS41);
        cbi(TCCR4B, CS42);
        cbi(TCCR4B, CS43);
        sbi(TCCR4A, PWM4A); // enable PWM A
        cbi(TCCR4D, WGM40); // Fast PWM (non PWM6)
        cbi(TCCR4D, WGM41);
        cbi(TCCR4A, COM4A0); // compare output mode: clear on compare match
        sbi(TCCR4A, COM4A1);
        
        OCR4A = 128;
        pinMode(5, OUTPUT);            // Enable PWM output pin
    #endif
    */
    // for ATmega328
    //#if defined(TCCR1A) && !defined(TCCR4A)
    //#if defined(TCCR1A)
        // TCCR1A => COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10
        // TCCR1B => ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
        
        // Set up Timer/Counter1 for PWM output
        sbi(TCCR1B, CS10); // 1:1 prescale
        cbi(TCCR1B, CS11);
        cbi(TCCR1B, CS12);
        sbi(TCCR1A, WGM10); // Fast PWM Mode, 8bit
        cbi(TCCR1A, WGM11);
        sbi(TCCR1B, WGM12);
        cbi(TCCR1B, WGM13);
        cbi(TCCR1A, COM1B0); // Clear OC1B on Compare Match
        sbi(TCCR1A, COM1B1);
        
        OCR1B = 128;
        pinMode(10, OUTPUT);            // Enable PWM output pin
    //#endif
    
    // Set up Timer/Counter0 for 20kHz interrupt to output samples.
    TCCR0A = 3<<WGM00;             // Fast PWM
    TCCR0B = 1<<WGM02 | 2<<CS00;   // 1/8 prescale
    OCR0A = 49;                    // Divide by 400
    
    // On Timer0, enable timer compare match, disable overflow
    #if defined(TIMSK)
        TIMSK = 1 << OCIE0A | 0 << TOIE0;
    #elif defined(TIMSK0)
        TIMSK0 = 1 << OCIE0A | 0 << TOIE0;
    #endif
    
    // Enable Watchdog timer for 128Hz interrupt
    #if defined(WDTCSR)
        WDTCSR |= 1<<WDIE;
    #elif defined(WDTCR)
        WDTCR |= 1<<WDIE;
    #endif

    //GIMSK = 0;                     // Disable INT0
    _delay_ms(1000);
}

/*void goToSleep(void) {
    byte adcsra, mcucr1, mcucr2, wdtcr;

    WDTCR = 0;                                // Disable Watchdog timer
    TIMSK = 0;                                // Disable all timers

    sleep_enable();
    adcsra = ADCSRA;                          // Save ADCSRA
    ADCSRA &= ~_BV(ADEN);                     // Disable ADC

    cli();                                    // Stop interrupts to ensure the BOD timed sequence executes as required
    GIMSK = _BV(INT0);                        // Enable INT0
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  // Turn off the brown-out detector
    mcucr2 = mcucr1 & ~_BV(BODSE);            // If the MCU does not have BOD disable capability,
    MCUCR = mcucr1;                           //   this code has no effect
    MCUCR = mcucr2;

    sei();                                    // Ensure interrupts enabled so we can wake up again
    sleep_cpu();                              // Go to sleep
    // Now asleep ...

    // ... awake again
    sleep_disable();                          // Wake up here
    ADCSRA = adcsra;                          // Restore ADCSRA

    PLLCSR = 1<<PCKE | 1<<PLLE;               // Re-enable PLL (for some reason, this is needed after sleeping...)
    TIMSK = 1<<OCIE0A;                        // Enable timer compare match, disable overflow
    WDTCR = 1<<WDIE;                          // Enable Watchdog timer for 128Hz interrupt
}*/


// Main loop

bool playVoice(Voice& voice) {
    if (voice.finished) return false;

    if (voice.playing) {
        if (voice.qlen_c == 0) {
            voice.gate = false;
            voice.amp = 0;
        }
        if (voice.nlen_c == 0) voice.playing = false;
        voice.qlen_c--;
        voice.nlen_c--;
    }

    if (voice.playing) {
        playSequences(voice);
        return true;
    }

    while (true) {
        const byte cmd = fetchNextByte(voice);

        if (!cmd) {
            if (voice.track_loop_ptr) {
                voice.ptr = voice.track_loop_ptr;
            } else {
                voice.gate = false;
                voice.finished = true;
                break;
            }
        } else if (cmd < 0x80) {
            playNote(voice, cmd);
            playSequences(voice);
            break;
        } else {
            executeCommand(voice, cmd);
        }
    }

    return true;
}

byte fetchNextByte(Voice& voice) {
    return pgm_read_byte(voice.ptr++);
}

inline void playNote(Voice& voice, byte note) {
    voice.playing = true;

    // If note is not a rest, set frequency based on note and current octave
    if (note != REST) {
        voice.note = (note & 0xf) - 2;
        voice.amp = amp[voice.volume];
        voice.gate = true;
    }

    // Set note length counter
    if (note & WITH_LEN) {
        // If note command has a length counter parameter, use it (word or byte)
        if (note & WORD) {
            voice.nlen_c = fetchNextByte(voice) | (fetchNextByte(voice) << 8);
        } else {
            voice.nlen_c = fetchNextByte(voice);
        }
    } else {
        // Otherwise, use default note length counter
        voice.nlen_c = voice.nlen;
    }

    // Set quantization length counter
    if (note & WITH_Q) {
        if (note & WORD) {
            voice.qlen_c = fetchNextByte(voice) | (fetchNextByte(voice) << 8);
        } else {
            voice.qlen_c = fetchNextByte(voice);
        }
    } else {
        voice.qlen_c = voice.qlen;
    }

    // Reset sequence pointers
    resetSequences(voice);
}

inline byte fetchSeqValue(const Envelope& env) {
    return pgm_read_byte(Seqs[env.id - 1] + env.i - 1);
}

byte playSequence(Voice& voice, Envelope& env) {
    byte value = fetchSeqValue(env);

    // [1 2 3 4 5 | 7 6 = 4 3 2 1 0]  1 2 3 4 5 7 6 [7 6 ...] 4 3 2 1 0
    // [1 2 3 4 5 | 7 6]              1 2 3 4 5 [7 6 7 6 ...] 0
    // [1 2 3 4 5 = 4 3 2 1 0]        1 2 3 4 [5 5 ...] 4 3 2 1 0
    // [1 2 3 4 5]                    1 2 3 4 [5 5 ...] 0

    if (voice.gate) {
        if (value == SEQ_LOOP) {
            // Found a Loop Marker, advance and save it for later
            env.i++;
            env.loop_i = env.i;
            value = fetchSeqValue(env);
        } else if (value == SEQ_END || value == SEQ_REL) {
            // Found a Release Marker, save it for later
            if (value == SEQ_REL) {
                env.rel_i = env.i + 1;
            }
            // Go back to Loop Marker if there is one
            if (env.loop_i) {
                env.i = env.loop_i;
                value = fetchSeqValue(env);
            }
        } else {
            env.i++;
        }
    } else {
        if (env.rel_i && env.i < env.rel_i) {
            env.i = env.rel_i;
            value = fetchSeqValue(env);
        } else if (value != SEQ_END) {
            env.i++;
        }
    }

    if (value == SEQ_LOOP || value == SEQ_REL || value == SEQ_END) return 0;

    return value;
}

inline void playSequences(Voice& voice) {
    // Volume Envelope
    if (voice.volume_env.id) {
        const byte value = playSequence(voice, voice.volume_env);
        if (value) {
            voice.amp = amp[MAX(0, ((value - 1) + (voice.volume - 15)))];
        }
    }

    // Note Envelope
    const byte value = voice.note + voice.transpose + (voice.note_env.id ? playSequence(voice, voice.note_env) : 0);
    const byte rel_note = value % 12;
    const byte rel_octave = value / 12;
    if (voice.waveform == NOISE) {
        voice.freq = noisePeriods[rel_note];
    } else {
        voice.freq = scale[rel_note] >> (8 - ((voice.octave + rel_octave) % 8));
    }

    // Timbre Envelope
    if (voice.timbre_env.id) {
        const byte value = playSequence(voice, voice.timbre_env);
        if (value) {
            voice.pw = value;
        }
    }

    // Pitch Envelope
    // TODO...
}

void resetSequence(Envelope& env) {
    if (env.id) {
        env.i = 1;
        env.loop_i = env.rel_i = 0;
    }
}

inline void resetSequences(Voice& voice) {
    resetSequence(voice.volume_env);
    resetSequence(voice.note_env);
    resetSequence(voice.timbre_env);
    resetSequence(voice.pitch_env);
}

inline void executeCommand(Voice& voice, const byte cmd) {
    switch (cmd) {
        case TRACK_LOOP:
            voice.track_loop_ptr = voice.ptr;
            break;
        case LOOP_START: {
            const byte times = fetchNextByte(voice);
            const byte i = ++voice.loops_idx;
            voice.loops_ptr[i] = voice.ptr;
            voice.loops_c[i] = times;
            break;
        }
        case LOOP_END: {
            const byte i = voice.loops_idx;
            voice.loops_c[i]--;
            if (voice.loops_c[i] == 0) {
                voice.loops_idx--;
            } else {
                voice.ptr = voice.loops_ptr[i];
            }
            break;
        }
        case NOTE_LEN:
            voice.nlen = fetchNextByte(voice);
            break;
        case NOTE_LEN_WORD:
            voice.nlen = fetchNextByte(voice) | (fetchNextByte(voice) << 8);
            break;
        case QUANT_LEN:
            voice.qlen = fetchNextByte(voice);
            break;
        case QUANT_LEN_WORD:
            voice.qlen = fetchNextByte(voice) | (fetchNextByte(voice) << 8);
            break;
        case OCTAVE:
            voice.octave = fetchNextByte(voice);;
            break;
        case INC_OCTAVE:
            if (voice.octave < 8) voice.octave++;
            break;
        case DEC_OCTAVE:
            if (voice.octave > 0) voice.octave--;
            break;
        case TRANSPOSE: {
            voice.transpose = (signed char) fetchNextByte(voice);
            break;
        }
        case DETUNE: {
            // TODO
            break;
        }
        case TIMBRE:
            voice.pw = fetchNextByte(voice);
            break;
        case VOLUME:
            voice.volume = fetchNextByte(voice);
            break;
        case INC_VOLUME:
            if (voice.volume < 16) voice.volume++;
            break;
        case DEC_VOLUME:
            if (voice.volume > 0) voice.volume--;
            break;
        case PITCH_SWEEP: {
            // TODO
            break;
        }
        // Select Envelope commands
        case VOLUME_ENV:
            voice.volume_env.id = fetchNextByte(voice);
            break;
        case NOTE_ENV:
            voice.note_env.id = fetchNextByte(voice);
            break;
        case TIMBRE_ENV:
            voice.timbre_env.id = fetchNextByte(voice);
            break;
        case PITCH_ENV:
            voice.pitch_env.id = fetchNextByte(voice);
            break;
    }
}

void loop() {
    /*_delay_ms(300);
    Serial.print("ticks=");
    Serial.print(ticks);
    Serial.print(", timer0_compa_tick=");
    Serial.print(timer0_compa_tick);
    Serial.print('\n');
    digitalWrite(PIN_LED, abs(digitalRead(PIN_LED) - 1));*/
    
    /*// Check if button is pressed for stoping song, taking care of debouncing
    byte buttonPressed = !digitalRead(2);
    if (buttonPressed) {
        if (!justAwoke) {
            // Song was playing, user is pressing button. Prepare for stop
            wantsToStop = true;
        }
    } else {
        if (justAwoke) {
            // User left button after waking up.
            justAwoke = false;
        } else if (wantsToStop) {
            // User was pressing button while song was playing,
            // now stopped pressing.  Stop playing.
            playing = false;
            wantsToStop = false;
        }
    }*/

    // If we are not playing, go to sleep.
    // After waking up, reset state.
    if (!playing) {
        //goToSleep();

        Serial.println("START");
        playing = true;
        for (int i = 0; i < NUM_VOICES; i++) {
            Voice* v = &voices[i];
            v->ptr = SongData[i];
            v->playing = false;
            v->finished = false;
            v->nlen = DEFAULT_NLEN;
            v->qlen = v->nlen;
            v->octave = DEFAULT_OCTAVE;
            v->transpose = 0;
            v->volume = DEFAULT_VOL;
            v->track_loop_ptr = NULL;
            v->loops_idx = -1;
            v->gate = false;
            v->pw = DEFAULT_PW;
        }
        voices[0].waveform = PULSE;
        voices[1].waveform = PULSE;
        voices[2].waveform = TRI;
        voices[3].waveform = NOISE;
        lfsrOut = 0;
        return;
    }

    if (nextTick) {
        nextTick = false;
        bool anyVoicePlaying = false;
        for (int i = 0; i < NUM_VOICES; i++) {
            anyVoicePlaying |= playVoice(voices[i]);
        }
        if (!anyVoicePlaying) {
            playing = false;
            Serial.println("END");
        }
    }
}
