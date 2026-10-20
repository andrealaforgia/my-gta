#include <assert.h>
#include <math.h>
#include <string.h>

#include "audio_mixer.h"
#include "test_groups.h"
#include "test_support.h"

#define RATE 48000

/* Zero crossings of the signal below about 300 Hz, where the exhaust
 * firing rate lives, so the noise bursts do not swamp the count. */
static int zero_crossings(const float* samples, int count) {
  int crossings = 0;
  float low = 0, previous = 0;
  float coefficient = 1.0f - expf(-2.0f * (float)M_PI * 300.0f / RATE);
  for (int i = 0; i < count; i++) {
    low += (samples[i] - low) * coefficient;
    if (i > 0 && (previous < 0) != (low < 0)) crossings++;
    previous = low;
  }
  return crossings;
}

static float peak(const float* samples, int count) {
  float largest = 0;
  for (int i = 0; i < count; i++)
    if (fabsf(samples[i]) > largest) largest = fabsf(samples[i]);
  return largest;
}

static void render(audio_mixer_t* mixer, audio_state_t state, float* out,
                   int frames) {
  audio_mixer_set_state(mixer, state);
  audio_mixer_render(mixer, out, frames);
}

static void the_engine_is_audible_at_idle_and_never_clips(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = true;
  state.rpm = 900;
  float out[RATE];
  render(&mixer, state, out, RATE);
  float loudest = peak(out, RATE);
  assert(loudest > 0.02f && loudest <= 1.0f);
}

static void higher_revs_raise_the_pitch(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = true;
  float low[RATE], high[RATE];
  state.rpm = 1000;
  render(&mixer, state, low, RATE);
  render(&mixer, state, low, RATE);
  state.rpm = 5000;
  render(&mixer, state, high, RATE);
  render(&mixer, state, high, RATE);
  assert(zero_crossings(high, RATE) > zero_crossings(low, RATE) * 2);
}

static void throttle_makes_the_engine_louder(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = true;
  state.rpm = 3000;
  float quiet[RATE], loud[RATE];
  state.throttle = 0;
  render(&mixer, state, quiet, RATE);
  render(&mixer, state, quiet, RATE);
  state.throttle = 1;
  render(&mixer, state, loud, RATE);
  render(&mixer, state, loud, RATE);
  assert(peak(loud, RATE) > peak(quiet, RATE) * 1.2f);
}

static void silence_when_the_engine_is_off(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  float out[RATE / 10];
  render(&mixer, state, out, RATE / 10);
  assert(peak(out, RATE / 10) < 1e-6f);
}

static void skid_and_impact_voices_start_from_state_and_fade(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.skid = 1.0f;
  float out[RATE / 10];
  render(&mixer, state, out, RATE / 10);
  float with_skid = peak(out, RATE / 10);
  assert(with_skid > 0.05f);
  state.skid = 0;
  render(&mixer, state, out, RATE / 10);
  render(&mixer, state, out, RATE / 10);
  assert(peak(out, RATE / 10) < with_skid * 0.2f);

  audio_mixer_trigger_impact(&mixer, 1.0f);
  render(&mixer, state, out, RATE / 10);
  float thump = peak(out, RATE / 10);
  assert(thump > 0.1f);
  for (int i = 0; i < 10; i++) render(&mixer, state, out, RATE / 10);
  assert(peak(out, RATE / 10) < 0.01f);
}

static void distance_attenuates_the_engine(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = true;
  state.rpm = 3000;
  float near_out[RATE / 4], far_out[RATE / 4];
  state.listener_distance = 2;
  render(&mixer, state, near_out, RATE / 4);
  render(&mixer, state, near_out, RATE / 4);
  state.listener_distance = 40;
  render(&mixer, state, far_out, RATE / 4);
  render(&mixer, state, far_out, RATE / 4);
  assert(peak(far_out, RATE / 4) < peak(near_out, RATE / 4) * 0.5f);
}

void run_audio_tests(void) {
  RUN(the_engine_is_audible_at_idle_and_never_clips);
  RUN(higher_revs_raise_the_pitch);
  RUN(throttle_makes_the_engine_louder);
  RUN(silence_when_the_engine_is_off);
  RUN(skid_and_impact_voices_start_from_state_and_fade);
  RUN(distance_attenuates_the_engine);
}
