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

#include "player.h"
#include "tune.h"
#include <MIDIUSB.h>
#include <pitchToNote.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define NUM_VOICES         5
#define MANUAL_VOICE_INDEX 4
#define DEFAULT_OCTAVE     4
#define DEFAULT_NLEN       4
#define DEFAULT_VOL        15
#define DEFAULT_PW         0xA0

const int PIN_LED = 13;
const bool useMIDI = true;

// midi constants
const unsigned char MESSAGE_NOTE_OFF = 0b1000;
const unsigned char MESSAGE_NOTE_ON = 0b1001;
const unsigned char MASK_MESSAGE_CHANNEL = 0b00001111;

// Note buffer
volatile uint16_t lfsr = 1;
volatile char lfsrOut = 0;
volatile signed char oldTemp = 0; // FIXME change variable name

// Global tick counter
volatile uint16_t timer0_tick = 0;
volatile uint8_t ticks = 0;
volatile bool nextTick = false;
volatile bool nextBigTick = false;

Voice voices[NUM_VOICES] = {};
byte playingVoices = 0;

volatile bool playing = false;
volatile bool mustRestartSong = false;

char debugStringBuffer[20];
bool debugMode = false;
// sprintf + serial of 20 bytes takes ~200us
// sprintf + serial of 10 bytes takes ~144us
// sprintf + serial of  5 bytes takes ~108us
#define serialDebug(...) \
    if (debugMode) { \
        sprintf(debugStringBuffer, __VA_ARGS__); \
        Serial.print(debugStringBuffer); \
    }
  
ISR(TIMER0_COMPA_vect) {
    // tick counter for time keeping
    timer0_tick++;
    if (timer0_tick > 333) { // 333 = 16.65ms, 111 = 5.55ms
        timer0_tick = 0;
        // if the tick counter fires before the last tick was completed,
        // while still playing, toggle the led
        if (nextTick && playing) {
            digitalWrite(PIN_LED, abs(digitalRead(PIN_LED) - 1));
        }
        // ticks and big ticks (every 1 second)
        nextTick = true;
        ticks++;
        if (ticks > 60) { // 16.65ms * 60 = 999ms
            ticks = 0;
            nextBigTick = true;
            // digitalWrite(PIN_LED, abs(digitalRead(PIN_LED) - 1));
        }
    }
    
    unsigned char temp;
    signed char stemp, mask, out = 0;
    Voice* v;

    for (int i = 0; i < NUM_VOICES; i++) {
        v = &voices[i];
        switch (v->waveform) {
            case PULSE:
                v->acc += v->freq;
                temp = (v->acc >> 8) & v->pw;
                out += (temp ? v->amp : 0) >> 2;
                break;
            case TRI:
                v->acc += v->freq / 2;
                stemp = v->acc >> 8;
                mask = stemp >> 7;
                if (v->amp != 0) {
                    if (v->volume > 12) { // 13 14 15
                        out += (stemp ^ mask) >> 1;
                    } else if (v->volume > 8) { // 9 10 11 12
                        out += (stemp ^ mask) >> 2;
                    } else if (v->volume > 4) { // 5 6 7 8
                        out += (stemp ^ mask) >> 3;
                    } else if (v->volume > 0) { // 1 2 3 4
                        out += (stemp ^ mask) >> 4;
                    }
                }
                break;
            case SAW:
                v->acc += v->freq;
                temp = v->acc >> 8;
                out += (temp ? v->amp : 0) >> 2;
                break;
            case NOISE:
                v->acc += v->freq * 8;
                stemp = (v->acc >> 8) & 0x80;
                // if temp != oldTemp, trigger the LFSR to generate a new pseudorandom value
                if (stemp != oldTemp) {
                    lfsrOut = (lfsr & 1) ^ ((lfsr & 2) >> 1);  // output is bit 0 XOR bit 1
                    lfsr = (lfsr >> 1) | (lfsrOut << 14);      // shift and include output on bit 15
                    oldTemp = stemp;
                }
                out += (lfsrOut ? v->amp : 0) >> 2;
                break;
        }
    }

    // notes on the noise channels:
    // This noise generator is somewhat based on the mechanism found in the NES APU.
    // The NES has a linear-feedback shift register for generating pseudorandom numbers.
    // It starts with a register set to 1, and when the period counter reaches 0, it
    // clocks the shift register.
    // The LFSR performs an Exclusive OR between bit 0 and bit 1, then shifts to the
    // right, and sets/resets bit 15 based on the exclusive OR result.

    OCR1B = out;
}

// Setup **********************************************

void setup() {
    Serial.begin(115200);
    cli();

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
    // The resulting frequency is calculated similar to the CTC mode.
    // F_CPU / (2 * PRESCALER * (1 + OCR0A))
    TCCR0A = 0;
    TCCR0B = 0;
    sbi(TCCR0A, WGM00);
    sbi(TCCR0A, WGM01);
    sbi(TCCR0B, WGM02); // Fast PWM
    sbi(TCCR0B, CS01); // 1/8 prescale
    // OCR0A = 49; // Divide by 800 (40KHz)
    OCR0A = 99; // Divide by 1600 (20KHz)
    
    // On Timer0, enable timer compare match, disable overflow
    #if defined(TIMSK)
        TIMSK = 1 << OCIE0A | 0 << TOIE0;
    #elif defined(TIMSK0)
        TIMSK0 = 1 << OCIE0A | 0 << TOIE0;
    #endif
    
    sei();

    // voice initialization
    for (int i = 0; i < NUM_VOICES; i++) {
        Voice* v = &voices[i];
        // the manual voice is only played through midi
        if (i != MANUAL_VOICE_INDEX) {
            v->ptr = SongData[i];
        }
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
    // voices
    voices[0].waveform = PULSE;
    voices[1].waveform = PULSE;
    voices[2].waveform = TRI;
    voices[3].waveform = NOISE;
    // manual voices
    voices[MANUAL_VOICE_INDEX].waveform = PULSE;
    voices[MANUAL_VOICE_INDEX].pw = 0x80;

    playing = true;
    serialDebug("START\n");
    
}

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

inline byte fetchNextByte(Voice& voice) {
    // return pgm_read_byte(voice.ptr++);
    return *(voice.ptr++);
}

inline void playNote(Voice& voice, byte note) {
    voice.playing = true;

    // If note is not a rest, set frequency based on note and current octave
    if ((note & NOTE_MASK) == REST) {
        voice.amp = 0;
    } else {
        voice.note = (note & NOTE_MASK) - 2;
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
    // return pgm_read_byte(Seqs[env.id - 1] + env.i - 1);
    return *(Seqs[env.id - 1] + env.i - 1);
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
    Voice* v;
    
    if (playing && nextTick) {
        // debug voices via serial
        if (ticks == 0) {
            for (int i = 0; i < NUM_VOICES; i++) {
                v = &voices[i];
                if (i != MANUAL_VOICE_INDEX) {
                    serialDebug("v: %.1i ptr: %.3u | ", i, (unsigned int)(v->ptr - SongData[i]));
                }
            }
            serialDebug("\n");
        }
        
        // play recorded song data
        bool anyVoicePlaying = false;
        for (int i = 0; i < NUM_VOICES; i++) {
            anyVoicePlaying |= playVoice(voices[i]);
        }
        // after each voice finishes playing, restore the flag
        nextTick = false;
        
        // stop playback after all voices have finished playing
        if (!anyVoicePlaying) {
            for (int i = 0; i < NUM_VOICES; i++) {
                v = &voices[i];
                // the manual voice can continue playing after the song ends
                if (i != MANUAL_VOICE_INDEX) {
                    v->amp = 0;
                }
            }
            playing = false;
            ticks = 0;
            nextBigTick = false;
            mustRestartSong = true;
            serialDebug("END\n");
            digitalWrite(PIN_LED, LOW);
        }
    }
    
    // restart song when flag is raised
    if (mustRestartSong && nextBigTick) {
        nextBigTick = false;
        mustRestartSong = false;
        for (int i = 0; i < NUM_VOICES; i++) {
            v = &voices[i];
            // the manual voice is only played through midi
            if (i != MANUAL_VOICE_INDEX) {
                v->ptr = SongData[i];
                v->finished = false;
                v->playing = false;
                v->track_loop_ptr = NULL;
                v->loops_ptr[v->loops_idx] = v->ptr;
                v->loops_idx = -1;
                resetSequences(voices[i]);
            }
        }
        lfsr = 1;
        lfsrOut = 0;
        playing = true;
        serialDebug("START\n");
    }

    // midi usb playback using the param MANUAL_VOICE_INDEX
    if (useMIDI) {
        midiEventPacket_t rx;
        do {
            rx = MidiUSB.read();
            if (rx.header != 0) {
                v = &voices[MANUAL_VOICE_INDEX];
                const byte message = (rx.byte1 >> 4) & MASK_MESSAGE_CHANNEL;
                // const byte channel = (rx.byte1 & MASK_MESSAGE_CHANNEL) + 1;
                const byte midiNote = rx.byte2;
                // const byte velocity = rx.byte3;
                if (message == MESSAGE_NOTE_ON) {
                    if (midiNote == pitchC1 || midiNote == pitchD1b || midiNote == pitchD1 || midiNote == pitchE1b || midiNote == pitchE1) continue;
                    v->octave = ((midiNote - midiNote % 12) / 12) - 1;
                    v->note = ((midiNote % 12) + 2) - 2;
                    v->amp = amp[v->volume];
                    v->gate = false;
                    v->freq = scale[v->note] >> (8 - (v->octave % 8));

                    // Serial.print("midi note: ");
                    // Serial.print(midiNote);
                    // Serial.print(" - v note: ");
                    // Serial.print(v->note);
                    // Serial.print(" - v octave: ");
                    // Serial.print(v->octave);
                    // Serial.print("\n");
                }
                if (message == MESSAGE_NOTE_OFF) {
                    const uint8_t noteOffOctave = ((midiNote - midiNote % 12) / 12) - 1;
                    const uint8_t noteOffNote = ((midiNote % 12) + 2) - 2;
                    if (v->note == noteOffNote && v->octave == noteOffOctave) {
                        v->amp = 0;
                    }
                    // special note messages
                    if (midiNote == pitchC1) {
                        debugMode = !debugMode;
                    }
                    if (midiNote == pitchD1b) {
                        v->waveform = TRI;
                    }
                    if (midiNote == pitchD1) {
                        v->waveform = PULSE;
                    }
                    if (midiNote == pitchE1b) {
                        playing = !playing;
                        for (int i = 0; i < NUM_VOICES; i++) {
                            v = &voices[i];
                            v->amp = 0;
                        }
                        serialDebug(!playing ? "PAUSE\n" : "UNPAUSE\n");
                        digitalWrite(PIN_LED, LOW);
                    }
                    if (midiNote == pitchE1) {
                        nextBigTick = true;
                        serialDebug("RESTART\n");
                        mustRestartSong = true;
                    }
                }
            }
        } while (rx.header != 0);
    }
}
