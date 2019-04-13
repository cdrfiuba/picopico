#ifndef __PLAYER_H__
#define __PLAYER_H__

#define MAX_LOOPS 4

// End of sequence
const byte END = 0;

// Note commands
const byte REST    = 1;
const byte NOTE_C  = 2;
const byte NOTE_CS = 3;
const byte NOTE_D  = 4;
const byte NOTE_DS = 5;
const byte NOTE_E  = 6;
const byte NOTE_F  = 7;
const byte NOTE_FS = 8;
const byte NOTE_G  = 9;
const byte NOTE_GS = 10;
const byte NOTE_A  = 11;
const byte NOTE_AS = 12;
const byte NOTE_B  = 13;

// Note flags
enum NoteFlag {
    WITH_LEN = 0x10,
    WITH_Q   = 0x20,
    WORD     = 0x40,
};

// Other commands (start at 0x80)
const byte TRACK_LOOP     = 0x80;
const byte LOOP_START     = 0x81;
const byte LOOP_END       = 0x82;
const byte NOTE_LEN       = 0x83;
const byte NOTE_LEN_WORD  = 0x84;
const byte QUANT_LEN      = 0x85;
const byte QUANT_LEN_WORD = 0x86;
const byte OCTAVE         = 0x87;
const byte INC_OCTAVE     = 0x88;
const byte DEC_OCTAVE     = 0x89;
const byte TRANSPOSE      = 0x8a;
const byte DETUNE         = 0x8b;
const byte TIMBRE         = 0x8c;
const byte VOLUME         = 0x8d;
const byte INC_VOLUME     = 0x8e;
const byte DEC_VOLUME     = 0x8f;
const byte PITCH_SWEEP    = 0x90;

// Select Envelope commands
const byte VOLUME_ENV = 0x91;
const byte NOTE_ENV   = 0x92;
const byte TIMBRE_ENV = 0x93;
const byte PITCH_ENV  = 0x94;

// Sequence commands
const byte SEQ_END  = 0x00;
const byte SEQ_LOOP = 0xfe;
const byte SEQ_REL  = 0xff;

// Note frequencies for 8th octave (highest):
//   freqs = [4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902]
//   [int((2**16 / 20000.0) * f) for f in freqs]
const uint16_t scale[] = {13716, 14532, 15397, 16311, 17281, 18310, 19398, 20552, 21774, 23068, 24441, 25893};

// Volume amplitude lookup table (16 entries)
const uint8_t amp[] = {0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240};

// Noise period counter (12 entries)
const uint16_t noisePeriods[] = {1200, 1760, 2360, 2960, 3760, 4720, 7080, 9440, 14160, 18880, 37800, 65535};

enum Waveform { PULSE, SAW, TRI, NOISE };

struct Envelope {
    byte      id;                     // Index into Seqs array (1-index, 0 = null)
    byte      i;                      // Current index (1-index)
    byte      loop_i;                 // Loop Marker index (1-index, 0 = null)
    byte      rel_i;                  // Release Marker index (1-index, 0 = null)
};

struct Voice {
    // Player-related registers
    // (const byte* because it's a variable pointer to an immutable object)
    const byte*     ptr;                    // Voice track pointer
    bool      gate;
    uint16_t  nlen;                   // Note length (in ticks)
    uint16_t  qlen;                   // Quantization length (in ticks)
    uint16_t  nlen_c;                 // Note length counter (in ticks)
    uint16_t  qlen_c;                 // Quantization length counter (in ticks)
    bool      playing;                // Whether voice is currently playing a note command
    bool      finished;               // Whether voice has finished (EOF)
    uint8_t   note;                   // Note (0-11)
    uint8_t   octave;                 // Octave (0-7)
    int8_t    transpose;              // Note transpose (-127 - 128)
    uint8_t   volume;                 // Volume (0-15)
    const  byte*     track_loop_ptr;         // Track loop (set by TRACK_LOOP command)
    uint8_t   loops_idx;              // Current loop index
    const  byte*     loops_ptr[MAX_LOOPS];   // Loop pointers
    uint8_t   loops_c[MAX_LOOPS];     // Loop repetion counters

    // For each envelope, there's an index to the envelope itself,
    // and the pointer within the envelope sequence.
    Envelope  volume_env;
    Envelope  note_env;
    Envelope  timbre_env;
    Envelope  pitch_env;

    // Internal registers for sound generation
    volatile Waveform waveform;
    volatile int16_t  acc;            // Phase accumulator
    volatile uint16_t freq;           // Frequency delta
    volatile uint8_t  amp;            // Amplitude
    volatile int8_t   pw;             // Pulse width
};

#endif /* ifndef __PLAYER_H__ */
