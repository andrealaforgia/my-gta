/**
 * @file audio_mixer.h
 * @brief Synthesised car audio with no sample assets (ADR-006): an
 * engine driven by rpm and throttle, tyre skid, impact thumps and wind.
 * Pure: renders float frames from a state snapshot, so the device layer
 * and the tests both call it.
 */
#ifndef ENGINE_AUDIO_AUDIO_MIXER_H_
#define ENGINE_AUDIO_AUDIO_MIXER_H_

#include <stdbool.h>

typedef struct {
  bool engine_running;
  float rpm;
  float throttle;          /* 0..1 */
  float skid;              /* 0..1 */
  float speed_mps;         /* for wind */
  float listener_distance; /* m from the car */
  float master_volume;     /* 0..1 */
} audio_state_t;

typedef struct {
  float frequency;
  float q;
  float x1, x2, y1, y2;
  float b0, b1, b2, a1, a2;
} bandpass_t;

typedef struct {
  int sample_rate;
  audio_state_t state;
  audio_state_t target;
  unsigned noise;
  float phase; /* firing phase, 0..1 */
  float pulse; /* current exhaust pulse envelope */
  float pulse_jitter;
  float rpm_smoothed;
  float throttle_smoothed;
  float skid_smoothed;
  float wind_smoothed;
  float impact; /* impact envelope */
  float impact_strength;
  bandpass_t pipe_low;
  bandpass_t pipe_high;
  bandpass_t skid_filter;
  float wind_lowpass;
  float intake_lowpass;
  float impact_lowpass;
} audio_mixer_t;

audio_state_t audio_state_default(void);
audio_mixer_t audio_mixer_create(int sample_rate);
void audio_mixer_set_state(audio_mixer_t* mixer, audio_state_t state);
void audio_mixer_trigger_impact(audio_mixer_t* mixer, float strength);
/* Mono frames in -1..1. */
void audio_mixer_render(audio_mixer_t* mixer, float* out, int frames);

#endif  // ENGINE_AUDIO_AUDIO_MIXER_H_
