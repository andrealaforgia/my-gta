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

/* Energy above about 2 kHz as a share of the whole: hiss and rustle live
 * up there, an engine note does not. */
static float treble_share(const float* samples, int count) {
  /* Four cascaded poles at 2 kHz: the residual is what lies above, with
   * the engine note's low harmonics leaking through 24 dB per octave down. */
  float low[4] = {0, 0, 0, 0};
  float total = 0, treble = 0;
  float coefficient = 1.0f - expf(-2.0f * (float)M_PI * 2000.0f / RATE);
  for (int i = 0; i < count; i++) {
    float x = samples[i];
    low[0] += (x - low[0]) * coefficient;
    low[1] += (low[0] - low[1]) * coefficient;
    low[2] += (low[1] - low[2]) * coefficient;
    low[3] += (low[2] - low[3]) * coefficient;
    float high = x - low[3];
    total += x * x;
    treble += high * high;
  }
  return total > 0 ? treble / total : 0;
}

static float tonality_at(const float* samples, int count, int lag) {
  double num = 0, den = 0;
  for (int i = lag; i < count; i++) {
    num += samples[i] * samples[i - lag];
    den += samples[i] * samples[i];
  }
  return den > 0 ? (float)(num / den) : 0;
}

/* The signal one firing period later should look like the signal now:
 * that is a note. Noise does not repeat. */
static void the_engine_is_not_mostly_hiss(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = true;
  float out[RATE];
  float rpms[3] = {900, 3000, 5500};
  for (int r = 0; r < 3; r++) {
    state.rpm = rpms[r];
    state.throttle = r == 0 ? 0.0f : 1.0f;
    render(&mixer, state, out, RATE);
    render(&mixer, state, out, RATE);
    int lag = (int)(RATE / (rpms[r] / 60.0f * 2.0f));
    assert(tonality_at(out, RATE, lag) > 0.55f);
    assert(treble_share(out, RATE) < 0.2f);
  }
}

/* Cornering hard scrapes the tyres. It must not be a clean sung note, and
 * a light touch of skid must stay nearly silent. */
static void tyre_scrape_is_rough_not_a_sung_note(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = false;
  state.skid = 1.0f;
  float out[RATE];
  render(&mixer, state, out, RATE);
  render(&mixer, state, out, RATE);
  assert(peak(out, RATE) > 0.03f);
  /* No lag near the old 780 Hz squeal repeats itself. */
  float best = 0;
  for (int lag = RATE / 1000; lag < RATE / 400; lag++)
    best = fmaxf(best, tonality_at(out, RATE, lag));
  assert(best < 0.5f);
  state.skid = 0.2f;
  render(&mixer, state, out, RATE);
  render(&mixer, state, out, RATE);
  assert(peak(out, RATE) < 0.02f);
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

static float rms(const float* samples, int count) {
  double sum = 0;
  for (int i = 0; i < count; i++) sum += (double)samples[i] * samples[i];
  return (float)sqrt(sum / count);
}

static void the_town_has_a_background_that_is_quiet_and_goes_with_the_level(
    void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = false;
  float out[RATE];
  render(&mixer, state, out, RATE);
  assert(peak(out, RATE) < 1e-4f);
  state.city = 1.0f;
  render(&mixer, state, out, RATE);
  render(&mixer, state, out, RATE);
  float level = rms(out, RATE);
  assert(level > 0.002f && level < 0.08f); /* there, but never loud */
  state.traffic = 1.0f;
  render(&mixer, state, out, RATE);
  render(&mixer, state, out, RATE);
  assert(rms(out, RATE) > level * 1.5f); /* cars close by add to it */
}

/* Power at one frequency, by the Goertzel recurrence. */
static float power_at(const float* samples, int count, float frequency) {
  float omega = 2.0f * (float)M_PI * frequency / RATE;
  float coefficient = 2.0f * cosf(omega);
  float s1 = 0, s2 = 0;
  for (int i = 0; i < count; i++) {
    float s0 = samples[i] + coefficient * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - coefficient * s1 * s2;
}

/* How much stronger the cricket's pitch is than the pitches either side. */
static float cricket_prominence(float night) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = false;
  state.city = 1.0f;
  state.night = night;
  static float out[RATE];
  for (int i = 0; i < 4; i++) render(&mixer, state, out, RATE); /* settle */
  float at = power_at(out, RATE, RATE / 11.0f);
  float around =
      0.5f * (power_at(out, RATE, 3300.0f) + power_at(out, RATE, 5600.0f));
  return at / (around + 1e-9f);
}

static void crickets_sing_by_night_in_a_high_steady_note(void) {
  float night = cricket_prominence(1.0f), day = cricket_prominence(0.0f);
  assert(night > 30.0f && night > day * 10.0f);
}

static int onsets(const float* samples, int count, float threshold) {
  int found = 0;
  bool quiet = true;
  for (int start = 0; start + 2400 <= count; start += 2400) {
    float level = peak(samples + start, 2400);
    if (level > threshold && quiet) found++;
    quiet = level < threshold * 0.5f;
  }
  return found;
}

static void the_church_bell_strikes_the_hour_in_a_ringing_tone(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = false;
  audio_mixer_set_state(&mixer, state);
  float out[RATE * 12];
  audio_mixer_trigger_bell(&mixer, 3);
  audio_mixer_render(&mixer, out, RATE * 12);
  assert(peak(out, RATE) > 0.05f);
  /* Three strikes, each a new onset, then silence again. */
  assert(onsets(out, RATE * 12, 0.04f) == 3);
  assert(peak(out + RATE * 11, RATE) < 0.01f);
  /* A bell rings: the first strike repeats itself at its pitch. */
  float tonal = 0;
  for (int lag = RATE / 330; lag < RATE / 190; lag++)
    tonal = fmaxf(tonal, tonality_at(out + 4800, RATE / 2, lag));
  assert(tonal > 0.5f);
  /* Asking again while it rings does not stack up more strikes. */
  audio_mixer_trigger_bell(&mixer, 1);
  audio_mixer_trigger_bell(&mixer, 1);
}

static void rain_is_a_soft_wash_that_is_louder_in_heavier_rain(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = false;
  float out[RATE];
  render(&mixer, state, out, RATE);
  assert(peak(out, RATE) < 1e-4f);
  state.rain = 0.4f;
  render(&mixer, state, out, RATE);
  render(&mixer, state, out, RATE);
  float light = rms(out, RATE);
  state.rain = 1.0f;
  render(&mixer, state, out, RATE);
  render(&mixer, state, out, RATE);
  float heavy = rms(out, RATE);
  assert(light > 0.002f && heavy > light * 1.5f && heavy < 0.12f);
  /* It is mostly high, hushing sound, with nothing sung in it. */
  assert(treble_share(out, RATE) > 0.1f);
  float best = 0;
  for (int lag = RATE / 1200; lag < RATE / 200; lag++)
    best = fmaxf(best, tonality_at(out, RATE, lag));
  assert(best < 0.6f);
}

static void the_horn_is_a_two_note_chord_that_starts_and_stops_quickly(void) {
  audio_mixer_t mixer = audio_mixer_create(RATE);
  audio_state_t state = audio_state_default();
  state.engine_running = false;
  static float out[RATE];
  render(&mixer, state, out, RATE);
  assert(peak(out, RATE) < 1e-4f);
  state.horn = true;
  render(&mixer, state, out, RATE);
  float loud = rms(out + RATE / 2, RATE / 2);
  assert(loud > 0.05f && loud < 0.4f);
  /* Two pitches, a third apart: strong at 420 and at 530 Hz, weak between
   * and well above. */
  float a = power_at(out + RATE / 4, RATE / 2, 420.0f);
  float b = power_at(out + RATE / 4, RATE / 2, 530.0f);
  float between = power_at(out + RATE / 4, RATE / 2, 700.0f);
  assert(a > between * 20.0f && b > between * 20.0f);
  /* It cuts off when let go, in a few hundredths of a second. */
  state.horn = false;
  render(&mixer, state, out, 4800);
  render(&mixer, state, out, RATE / 2);
  assert(peak(out + RATE / 4, RATE / 4) < 0.003f);
}

void run_audio_tests(void) {
  RUN(the_horn_is_a_two_note_chord_that_starts_and_stops_quickly);
  RUN(rain_is_a_soft_wash_that_is_louder_in_heavier_rain);
  RUN(the_town_has_a_background_that_is_quiet_and_goes_with_the_level);
  RUN(crickets_sing_by_night_in_a_high_steady_note);
  RUN(the_church_bell_strikes_the_hour_in_a_ringing_tone);
  RUN(the_engine_is_audible_at_idle_and_never_clips);
  RUN(higher_revs_raise_the_pitch);
  RUN(throttle_makes_the_engine_louder);
  RUN(the_engine_is_not_mostly_hiss);
  RUN(silence_when_the_engine_is_off);
  RUN(skid_and_impact_voices_start_from_state_and_fade);
  RUN(tyre_scrape_is_rough_not_a_sung_note);
  RUN(distance_attenuates_the_engine);
}
