#ifndef __TUNE_H__
#define __TUNE_H__

#include "player.h"

const byte Seq1[] PROGMEM = { 8, 9, 10, 11, 12, 14, 16, 12, 10, SEQ_REL, 6, 5, 4, 3, 2, 1, SEQ_END };
const byte Seq2[] PROGMEM = { SEQ_LOOP, 1, 5, 8, SEQ_END };
const byte Seq3[] PROGMEM = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, SEQ_END };
const byte Seq4[] PROGMEM = { 15, 14, 13, 10, 7, 4, 2, 1, SEQ_END };

const byte* Seqs[] = { Seq1, Seq2, Seq3 };

const byte SongData0[] PROGMEM = {
    NOTE_LEN, 15,
    QUANT_LEN, 12,
    VOLUME_ENV, 1,
    VOLUME, 15,
    INC_OCTAVE,

    TRACK_LOOP,

    LOOP_START, 2, NOTE_C, NOTE_DS, NOTE_G, NOTE_AS, DEC_VOLUME, LOOP_END,
    LOOP_START, 2, NOTE_D, NOTE_G, NOTE_A, NOTE_AS, DEC_VOLUME, LOOP_END,

    END
};

const byte SongData1[] PROGMEM = {
    NOTE_LEN, 15,
    QUANT_LEN, 12,
    VOLUME_ENV, 1,
    VOLUME, 10,
    REST|WITH_LEN|WITH_Q, 12, 12,
    INC_OCTAVE,

    TRACK_LOOP,

    LOOP_START, 2, NOTE_AS, NOTE_AS, NOTE_A, NOTE_A, DEC_VOLUME, LOOP_END,
    LOOP_START, 2, NOTE_AS, NOTE_A, NOTE_G, NOTE_D, DEC_VOLUME, LOOP_END,

    END
};

const byte SongData2[] PROGMEM = {
    NOTE_LEN, 7,
    QUANT_LEN, 7,
    OCTAVE, 3,

    LOOP_START, 8,

    LOOP_START, 4, NOTE_C, LOOP_END,
    INC_OCTAVE, NOTE_C, DEC_OCTAVE, NOTE_C, NOTE_G, DEC_OCTAVE, NOTE_G, INC_OCTAVE,

    LOOP_END,

    END
};
const byte SongData3[] PROGMEM = {
    VOLUME_ENV, 2,
    OCTAVE, 0,

    TRACK_LOOP,

    LOOP_START, 2, NOTE_C, NOTE_DS, NOTE_G, NOTE_AS, DEC_VOLUME, LOOP_END,
    LOOP_START, 2, NOTE_D, NOTE_G, NOTE_A, NOTE_AS, DEC_VOLUME, LOOP_END,
    END
};

const byte* SongData[] = { SongData0, SongData1, SongData2, SongData3 };

#endif /* ifndef __TUNE_H__ */
