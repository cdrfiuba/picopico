#ifndef __TUNE_H__
#define __TUNE_H__

#include "player.h"

const byte* Seqs[] = {{}};

const byte SongData0[] = { 
LOOP_START, 1, VOLUME, 4, OCTAVE, 4, NOTE_LEN, 30, QUANT_LEN, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_A, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_A, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_A, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_F, NOTE_A, NOTE_F|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_F, NOTE_A, NOTE_F|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_F, NOTE_GS, NOTE_F|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_F, NOTE_GS, NOTE_F|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, NOTE_C, NOTE_E, NOTE_G, NOTE_E|WITH_LEN|WITH_Q, 31, 30, LOOP_END, VOLUME, 3, NOTE_LEN, 62, QUANT_LEN, 62, NOTE_C|WITH_LEN|WITH_Q, 63, 62, END
};
const byte SongData1[] = {
LOOP_START, 1, VOLUME, 4, OCTAVE, 4, NOTE_LEN, 30, QUANT_LEN, 30, NOTE_D, NOTE_C, NOTE_D, NOTE_C|WITH_LEN|WITH_Q, 31, 30, NOTE_D, NOTE_C, NOTE_D, NOTE_C|WITH_LEN|WITH_Q, 31, 30, INC_OCTAVE, NOTE_LEN, 62, QUANT_LEN, 62, NOTE_A, NOTE_G|WITH_LEN|WITH_Q, 61, 61, NOTE_E, NOTE_D|WITH_LEN|WITH_Q, 61, 61, NOTE_C, DEC_OCTAVE, NOTE_A|WITH_LEN|WITH_Q, 61, 61, NOTE_B, INC_OCTAVE, NOTE_C|WITH_LEN|WITH_Q, 61, 61, NOTE_E, REST|WITH_LEN|WITH_Q, 61, 61, NOTE_D, DEC_OCTAVE, NOTE_B|WITH_LEN|WITH_Q, 61, 61, NOTE_B, NOTE_B|WITH_LEN|WITH_Q, 61, 61, NOTE_B, NOTE_C|WITH_LEN|WITH_Q, 61, 61, REST, INC_OCTAVE, NOTE_C|WITH_LEN|WITH_Q, 61, 61, NOTE_D, NOTE_E|WITH_LEN|WITH_Q, 61, 61, NOTE_D, NOTE_C|WITH_LEN|WITH_Q, 61, 61, REST, REST|WITH_LEN|WITH_Q, 61, 61, REST, REST|WITH_LEN|WITH_Q, 61, 61, LOOP_END, END
};
const byte SongData2[] = {
LOOP_START, 1, VOLUME, 14, OCTAVE, 4, NOTE_LEN, 124, QUANT_LEN, 124, NOTE_C, REST, NOTE_C, REST, NOTE_C, REST, DEC_OCTAVE, NOTE_A, REST, NOTE_A, NOTE_F, NOTE_F, NOTE_GS, NOTE_GS, REST, REST, LOOP_END, END
};
const byte SongData3[] = {
LOOP_START, 16, VOLUME, 4, NOTE_LEN, 3, QUANT_LEN, 3, NOTE_C, REST, VOLUME, 2, NOTE_C, REST, VOLUME, 4, NOTE_C, REST, VOLUME, 2, NOTE_C, REST, REST, REST, VOLUME, 4, NOTE_C|WITH_LEN|WITH_Q, 2, 2, REST, VOLUME, 2, NOTE_C, REST, VOLUME, 4, NOTE_C, REST, VOLUME, 6, NOTE_E, REST, VOLUME, 2, NOTE_C, REST, NOTE_LEN, 15, QUANT_LEN, 15, REST|WITH_LEN|WITH_Q, 14, 14, REST, NOTE_LEN, 7, QUANT_LEN, 7, REST, NOTE_G|WITH_LEN|WITH_Q, 6, 6, LOOP_END, END
};


const byte* SongData[] = {SongData0, SongData1, SongData2, SongData3};

#endif /* ifndef __TUNE_H__ */