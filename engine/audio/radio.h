/**
 * @file radio.h
 * @brief The car radio: three synthesised stations, each a small band
 * playing a loop (drums, bass, chords and a tune) with no sample assets. All
 * of it is kept soft: nothing sits above a few kilohertz to whistle.
 */
#ifndef ENGINE_AUDIO_RADIO_H_
#define ENGINE_AUDIO_RADIO_H_

#define RADIO_STATIONS 3

typedef struct {
  int sample_rate;
  int station; /* 0 is off */
  long sample; /* position in the loop */
  unsigned noise;
  float phase[4];
  float kick_phase;
  float lowpass[4];
  float hat_previous;
} radio_t;

void radio_init(radio_t* radio, int sample_rate);
/* Tunes in: 0 turns it off, 1..RADIO_STATIONS pick a station. The song
 * restarts when the station changes. */
void radio_tune(radio_t* radio, int station);
/* The next station, wrapping through off: off, 1, 2, 3, off, ... */
int radio_next_station(int station);
const char* radio_station_name(int station);
/* One mono sample in about -1..1 (0 when off). */
float radio_next(radio_t* radio);

#endif  // ENGINE_AUDIO_RADIO_H_
