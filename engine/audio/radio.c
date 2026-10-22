#include "radio.h"

#include <math.h>
#include <string.h>

#define TAU 6.28318530718f

static float midi_hz(float note) {
  return 440.0f * powf(2.0f, (note - 69.0f) / 12.0f);
}

static float white(radio_t* radio) {
  radio->noise = radio->noise * 1664525u + 1013904223u;
  return (float)(radio->noise >> 8) / 8388608.0f - 1.0f;
}

static float one_pole(float* state, float x, float coefficient) {
  *state += (x - *state) * coefficient;
  return *state;
}

static float triangle(float phase) {
  float p = phase - floorf(phase);
  return 4.0f * fabsf(p - 0.5f) - 1.0f;
}

static float saw(float phase) { return 2.0f * (phase - floorf(phase)) - 1.0f; }

void radio_init(radio_t* radio, int sample_rate) {
  memset(radio, 0, sizeof(*radio));
  radio->sample_rate = sample_rate;
  radio->noise = 12345u;
}

void radio_tune(radio_t* radio, int station) {
  if (station < 0 || station > RADIO_STATIONS) station = 0;
  if (station == radio->station) return;
  int rate = radio->sample_rate;
  radio_init(radio, rate);
  radio->station = station;
}

int radio_next_station(int station) {
  return station >= RADIO_STATIONS ? 0 : station + 1;
}

const char* radio_station_name(int station) {
  switch (station) {
    case 1:
      return "RADIO ADRIATICA";
    case 2:
      return "NOTTE JAZZ";
    case 3:
      return "COSTA DANCE";
    default:
      return "RADIO OFF";
  }
}

/* A thump: a sine that falls in pitch. */
static float kick(radio_t* radio, float t) {
  radio->kick_phase += (42.0f + 110.0f * expf(-t * 35.0f)) / radio->sample_rate;
  return sinf(TAU * radio->kick_phase) * expf(-t * 7.0f);
}

/* A soft snare or clap: low-passed noise with a short body. */
static float snare(radio_t* radio, float t) {
  float n = one_pole(&radio->lowpass[1], white(radio), 0.35f);
  return n * expf(-t * 16.0f) * 1.6f +
         sinf(TAU * 190.0f * t) * expf(-t * 25.0f) * 0.4f;
}

/* A hi-hat, kept dull: noise with the lowest part taken off, then rounded. */
static float hat(radio_t* radio, float t, float decay) {
  float n = white(radio);
  float high = n - radio->hat_previous;
  radio->hat_previous = n;
  return one_pole(&radio->lowpass[2], high, 0.45f) * expf(-t * decay);
}

/* Pop: 118 beats a minute, four chords, a bass, a pad and a lead tune. */
static float pop(radio_t* radio, double beat) {
  static const int roots[4] = {45, 41, 48, 43};
  static const int thirds[4] = {3, 4, 4, 4};
  static const int lead[16] = {12, 7, 0, 7, 12, 7, 3, 7,
                               12, 7, 0, 7, 10, 7, 3, 7};
  int bar = ((int)(beat / 4.0)) & 3;
  double in_bar = fmod(beat, 4.0);
  double beat_t = (in_bar - floor(in_bar)) * 60.0 / 118.0;
  int step = (int)(in_bar * 4.0);
  double step_t = (in_bar * 4.0 - step) * 60.0 / 118.0 / 4.0;
  float out = 0;
  out += kick(radio, (float)beat_t) * 0.9f;
  if ((int)in_bar == 1 || (int)in_bar == 3)
    out += snare(radio, (float)beat_t) * 0.5f;
  if (step % 2 == 1) out += hat(radio, (float)step_t, 70.0f) * 0.12f;
  /* Bass on the eighths, jumping an octave on the third. */
  int eighth = step / 2;
  float bass_note = (float)roots[bar] + ((eighth % 4) == 3 ? 12.0f : 0.0f);
  double eighth_t = (in_bar * 2.0 - eighth) * 60.0 / 118.0 / 2.0;
  radio->phase[0] += midi_hz(bass_note) / radio->sample_rate;
  out += one_pole(&radio->lowpass[0], saw(radio->phase[0]), 0.06f) *
         (0.35f + 0.65f * expf(-(float)eighth_t * 6.0f)) * 0.55f;
  /* Pad: a triad of soft triangles. */
  float pad = 0;
  int intervals[3] = {0, thirds[bar], 7};
  double seconds = (double)radio->sample / radio->sample_rate;
  for (int i = 0; i < 3; i++)
    pad += triangle((float)fmod(
        seconds * midi_hz((float)(roots[bar] + 12 + intervals[i])), 1.0));
  out += pad * 0.07f;
  /* Lead: a rounded saw playing the tune, a note on each sixteenth. */
  float note = (float)roots[bar] + 24.0f + (float)lead[step];
  radio->phase[2] += midi_hz(note) / radio->sample_rate;
  float tone = one_pole(&radio->lowpass[3], saw(radio->phase[2]), 0.09f);
  out += tone * expf(-(float)step_t * 9.0f) * 0.32f;
  return out;
}

/* Jazz: 92 beats a minute with a swung ride, a walking bass and chords. */
static float jazz(radio_t* radio, double beat) {
  static const int roots[4] = {38, 43, 36, 45};
  static const int walk[4][4] = {
      {0, 4, 7, 9}, {0, 2, 4, 7}, {0, 4, 7, 11}, {0, 3, 7, 5}};
  int bar = ((int)(beat / 4.0)) & 3;
  double in_bar = fmod(beat, 4.0);
  int quarter = (int)in_bar;
  double beat_t = (in_bar - quarter) * 60.0 / 92.0;
  float out = 0;
  /* Walking bass: a note on each beat, plucked. */
  float note = (float)(roots[bar] + walk[bar][quarter]);
  radio->phase[0] += midi_hz(note) / radio->sample_rate;
  float bass =
      sinf(TAU * radio->phase[0]) + 0.35f * sinf(TAU * 2.0f * radio->phase[0]);
  out += bass * expf(-(float)beat_t * 2.5f) * 0.55f;
  /* Ride cymbal on the swung pattern: 1, 2 and the lilt after 2, 3, 4. */
  double third = in_bar * 3.0;
  int slot = (int)third;
  static const int ride[12] = {1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1};
  double slot_t = (third - slot) * 60.0 / 92.0 / 3.0;
  if (ride[slot]) out += hat(radio, (float)slot_t, 18.0f) * 0.22f;
  /* Brushes on the backbeat. */
  if (quarter == 1 || quarter == 3) out += snare(radio, (float)beat_t) * 0.12f;
  /* Piano chords comped on the first beat and the "and" of two. */
  double comp_t = -1;
  if (in_bar < 1.5)
    comp_t = in_bar * 60.0 / 92.0;
  else if (in_bar >= 2.0 && in_bar < 3.5)
    comp_t = (in_bar - 2.0) * 60.0 / 92.0;
  if (comp_t >= 0) {
    static const int shape[4] = {0, 3, 7, 10};
    float chord = 0;
    for (int i = 0; i < 4; i++) {
      float hz = midi_hz((float)(roots[bar] + 24 + shape[i]));
      chord += sinf(TAU * hz * (float)comp_t) +
               0.3f * sinf(TAU * 2.0f * hz * (float)comp_t);
    }
    out += chord * expf(-(float)comp_t * 3.0f) * 0.06f;
  }
  return out;
}

/* Dance: 126 beats a minute, four on the floor with a rolling bass line. */
static float dance(radio_t* radio, double beat) {
  static const int line[16] = {0, 0, 12, 0, 0, 10, 0, 7,
                               0, 0, 12, 0, 3, 0,  7, 0};
  double in_bar = fmod(beat, 4.0);
  double beat_t = (in_bar - floor(in_bar)) * 60.0 / 126.0;
  int step = (int)(in_bar * 4.0);
  double step_t = (in_bar * 4.0 - step) * 60.0 / 126.0 / 4.0;
  float out = kick(radio, (float)beat_t) * 1.0f;
  if ((int)in_bar == 1 || (int)in_bar == 3)
    out += snare(radio, (float)beat_t) * 0.45f;
  if (step % 4 == 2) out += hat(radio, (float)step_t, 40.0f) * 0.22f;
  /* The bass line, opening and closing its filter over the bars. */
  radio->phase[0] += midi_hz(33.0f + (float)line[step]) / radio->sample_rate;
  float open = 0.04f + 0.06f * (0.5f + 0.5f * sinf((float)beat * 0.4f));
  float bass = one_pole(&radio->lowpass[0], saw(radio->phase[0]), open);
  float gate = step_t < 0.1 ? 1.0f : 0.6f;
  /* Keep the bass out of the kick's way. */
  float duck = 1.0f - 0.7f * expf(-(float)beat_t * 12.0f);
  out += bass * gate * duck * 0.7f;
  /* A pad stab on the offbeats. */
  if (step % 4 == 2) {
    float stab =
        triangle(radio->phase[1] += midi_hz(57.0f) / radio->sample_rate) +
        triangle(radio->phase[2] += midi_hz(60.0f) / radio->sample_rate);
    out += stab * expf(-(float)step_t * 12.0f) * 0.12f;
  }
  return out;
}

float radio_next(radio_t* radio) {
  if (radio->station == 0) return 0;
  double beat = 0;
  float out = 0;
  switch (radio->station) {
    case 1:
      beat = (double)radio->sample / radio->sample_rate * 118.0 / 60.0;
      out = pop(radio, beat);
      break;
    case 2:
      beat = (double)radio->sample / radio->sample_rate * 92.0 / 60.0;
      out = jazz(radio, beat);
      break;
    default:
      beat = (double)radio->sample / radio->sample_rate * 126.0 / 60.0;
      out = dance(radio, beat);
      break;
  }
  radio->sample++;
  for (int i = 0; i < 4; i++) radio->phase[i] -= floorf(radio->phase[i]);
  radio->kick_phase -= floorf(radio->kick_phase);
  return tanhf(out * 0.9f);
}
